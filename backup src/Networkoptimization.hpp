// ============================================================================
// NETWORK_OPTIMIZATION.hpp
// Optimasi pengiriman data ke website & server
// Ready to use - copy dan modify sesuai kebutuhan Anda
// ============================================================================

#ifndef NETWORK_OPTIMIZATION_HPP
#define NETWORK_OPTIMIZATION_HPP

#include <Arduino.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ============================================================================
// 1. COMPRESSION MODULE
// ============================================================================

#ifdef ESP32
#include <zlib.h>

String compressDataGZIP(String original)
{
    // Skip compression jika data terlalu kecil (overhead tidak worth it)
    if (original.length() < 100)
        return original;

    uLongf compressedSize = compressBound(original.length());
    uint8_t *compressed = new uint8_t[compressedSize];

    int result = compress2(
        compressed,
        &compressedSize,
        (uint8_t *)original.c_str(),
        original.length(),
        6 // Level 6 = optimal balance antara speed & compression
    );

    if (result != Z_OK)
    {
        delete[] compressed;
        Serial.println("[COMPRESS] Failed - sending uncompressed");
        return original;
    }
    // Base64 encode untuk transmission
    String encoded = "";
    const char base64_chars[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    uint8_t byte_a, byte_b, byte_c;
    int i = 0;

    while (i < compressedSize)
    {
        byte_a = compressed[i++];
        byte_b = (i < compressedSize) ? compressed[i++] : 0;
        byte_c = (i < compressedSize) ? compressed[i++] : 0;

        uint32_t triple = (byte_a << 16) + (byte_b << 8) + byte_c;

        encoded += base64_chars[(triple >> 18) & 0x3F];
        encoded += base64_chars[(triple >> 12) & 0x3F];
        encoded += (i - 1 < compressedSize) ? base64_chars[(triple >> 6) & 0x3F] : '=';
        encoded += (i < compressedSize) ? base64_chars[triple & 0x3F] : '=';
    }

    float ratio = (100.0 * (compressedSize + encoded.length())) / original.length();
    Serial.printf("[COMPRESS] %d -> %d bytes (%.1f%% of original)\n",
                  original.length(), encoded.length(), ratio);

    delete[] compressed;
    return encoded;
}

#else
// Fallback untuk non-ESP32 platforms
String compressDataGZIP(String original)
{
    return original;
}
#endif

// ============================================================================
// 2. SMART BATCHING MODULE
// ============================================================================

struct DataBatch
{
    DynamicJsonDocument json;
    uint32_t count;
    unsigned long firstTimestamp;
    size_t byteSize;

    DataBatch() : json(4096), count(0), firstTimestamp(0), byteSize(0) {}

    void clear()
    {
        json.clear();
        count = 0;
        firstTimestamp = 0;
        byteSize = 0;
    }

    void addData(JsonObject data)
    {
        if (count == 0)
            firstTimestamp = millis();

        json["data"][count] = data;

        String temp;
        serializeJson(data, temp);
        byteSize += temp.length();
        count++;
    }

    String serialize()
    {
        String result;
        json["count"] = count;
        json["timestamp"] = millis();
        serializeJson(json, result);
        return result;
    }
};

class SmartBatcher
{
private:
    DataBatch currentBatch;
    const int MIN_BATCH_SIZE = 5;
    const size_t MAX_BATCH_BYTES = 2048;
    const unsigned long BATCH_TIMEOUT = 10000; // 10 seconds
    SemaphoreHandle_t batchMutex;

public:
    SmartBatcher()
    {
        batchMutex = xSemaphoreCreateMutex();
    }

    bool shouldSendBatch()
    {
        if (currentBatch.count >= MIN_BATCH_SIZE)
            return true;

        if (currentBatch.count > 0 &&
            (millis() - currentBatch.firstTimestamp > BATCH_TIMEOUT))
            return true;

        if (currentBatch.byteSize >= MAX_BATCH_BYTES)
            return true;

        return false;
    }

    void addData(JsonObject data)
    {
        if (xSemaphoreTake(batchMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            currentBatch.addData(data);
            xSemaphoreGive(batchMutex);
        }
    }

    String getBatchAndClear()
    {
        if (xSemaphoreTake(batchMutex, pdMS_TO_TICKS(100)) != pdTRUE)
            return "";

        String result = currentBatch.serialize();
        currentBatch.clear();

        xSemaphoreGive(batchMutex);
        return result;
    }

    int getBatchCount()
    {
        return currentBatch.count;
    }
};

SmartBatcher globalBatcher;

// ============================================================================
// 3. ADAPTIVE SAMPLING MODULE
// ============================================================================

struct SensorValue
{
    float currentValue;
    float lastSentValue;
    unsigned long lastSendTime;
    float changeThreshold;
    const unsigned long HEARTBEAT_INTERVAL = 60000; // 1 minute
};

class AdaptiveSampler
{
private:
    SensorValue sensors[16]; // Up to 16 sensors
    uint8_t sensorCount;

public:
    AdaptiveSampler(uint8_t count = 16) : sensorCount(count)
    {
        for (int i = 0; i < sensorCount; i++)
        {
            sensors[i].currentValue = 0;
            sensors[i].lastSentValue = 0;
            sensors[i].lastSendTime = 0;
            sensors[i].changeThreshold = 0.5; // Default 0.5
        }
    }

    void setSensor(uint8_t index, float threshold)
    {
        if (index < sensorCount)
            sensors[index].changeThreshold = threshold;
    }

    bool shouldSend(uint8_t index, float newValue)
    {
        if (index >= sensorCount)
            return false;
        sensors[index].currentValue = newValue;
        float delta = abs(newValue - sensors[index].lastSentValue);
        unsigned long timeSinceLastSend = millis() - sensors[index].lastSendTime;

        // Kirim jika:
        // 1. Perubahan > threshold
        // 2. Atau sudah HEARTBEAT_INTERVAL tanpa update
        if (delta > sensors[index].changeThreshold ||
            timeSinceLastSend > HEARTBEAT_INTERVAL)
        {
            sensors[index].lastSentValue = newValue;
            sensors[index].lastSendTime = millis();
            return true;
        }
        return false;
    }
};

// ============================================================================
// 4. RETRY MANAGER WITH EXPONENTIAL BACKOFF
// ============================================================================

struct RetryConfig
{
    uint8_t maxRetries = 3;
    uint32_t initialDelay = 500; // ms
    uint32_t maxDelay = 30000;   // ms
    float backoffMultiplier = 2.0f;
};

class SmartRetryManager
{
private:
    uint8_t retryCount = 0;
    uint32_t nextRetryTime = 0;
    RetryConfig config;

public:
    SmartRetryManager(const RetryConfig &cfg) : config(cfg) {}

    bool canRetry()
    {
        if (retryCount >= config.maxRetries)
        {
            Serial.printf("[RETRY] Max retries (%d) reached\n", config.maxRetries);
            return false;
        }

        if (millis() >= nextRetryTime)
            return true;

        return false;
    }

    uint32_t millisecondsUntilRetry()
    {
        if (nextRetryTime <= millis())
            return 0;
        return nextRetryTime - millis();
    }

    void recordFailure()
    {
        retryCount++;

        // Calculate exponential backoff
        uint32_t delay = config.initialDelay;
        for (int i = 1; i < retryCount; i++)
        {
            delay = (uint32_t)(delay * config.backoffMultiplier);
        }
        delay = min((uint32_t)delay, config.maxDelay);

        nextRetryTime = millis() + delay;

        Serial.printf("[RETRY] Attempt %d/%d - Wait %lu ms\n",
                      retryCount, config.maxRetries, delay);
    }

    void recordSuccess()
    {
        if (retryCount > 0)
            Serial.printf("[RETRY] Success on attempt %d\n", retryCount);

        retryCount = 0;
        nextRetryTime = 0;
    }

    void reset()
    {
        retryCount = 0;
        nextRetryTime = 0;
    }
};

// ============================================================================
// 5. NETWORK METRICS & MONITORING
// ============================================================================

struct NetworkMetrics
{
    uint32_t totalBytesSent = 0;
    uint32_t totalBytesReceived = 0;
    uint32_t totalRequests = 0;
    uint32_t successfulRequests = 0;
    uint32_t failedRequests = 0;
    uint32_t totalResponseTime = 0; // Cumulative
    float successRate = 0.0;

    unsigned long lastPrintTime = 0;
    unsigned long lastResetTime = 0;
};

class NetworkMonitor
{
private:
    NetworkMetrics metrics;
    const unsigned long PRINT_INTERVAL = 300000; // 5 minutes

public:
    void recordRequest(uint32_t sentBytes, uint32_t receivedBytes,
                       unsigned long responseTime, bool success)
    {
        metrics.totalBytesSent += sentBytes;
        metrics.totalBytesReceived += receivedBytes;
        metrics.totalRequests++;
        metrics.totalResponseTime += responseTime;

        if (success)
            metrics.successfulRequests++;
        else
            metrics.failedRequests++;

        updateSuccessRate();
    }

    void updateSuccessRate()
    {
        if (metrics.totalRequests > 0)
            metrics.successRate =
                (float)metrics.successfulRequests / metrics.totalRequests * 100.0f;
    }

    void checkAndPrint()
    {
        if (millis() - metrics.lastPrintTime > PRINT_INTERVAL)
        {
            printStats();
            metrics.lastPrintTime = millis();
        }
    }

    void printStats()
    {
        unsigned long uptime = (millis() - metrics.lastResetTime) / 1000;

        Serial.println("\n╔═══════════════════════════════════╗");
        Serial.println("║   NETWORK STATISTICS (5 min)      ║");
        Serial.println("╠═══════════════════════════════════╣");
        Serial.printf("║ Requests  : %d (%.1f%% success)     ║\n",
                      metrics.totalRequests, metrics.successRate);
        Serial.printf("║ Sent      : %.1f KB                ║\n",
                      metrics.totalBytesSent / 1024.0f);
        Serial.printf("║ Received  : %.1f KB                ║\n",
                      metrics.totalBytesReceived / 1024.0f);

        if (metrics.totalRequests > 0)
        {
            uint32_t avgTime = metrics.totalResponseTime / metrics.totalRequests;
            Serial.printf("║ Avg Time  : %lu ms                  ║\n", avgTime);
        }

        Serial.printf("║ Uptime    : %lu seconds            ║\n", uptime);
        Serial.println("╚═══════════════════════════════════╝\n");
    }

    void reset()
    {
        metrics = NetworkMetrics();
        metrics.lastResetTime = millis();
    }
};

// ============================================================================
// 6. PAYLOAD DELTA OPTIMIZATION
// ============================================================================

class DeltaPayloadOptimizer
{
private:
    DynamicJsonDocument lastSentPayload;
    unsigned long lastSentTime = 0;
    const unsigned long FULL_PAYLOAD_INTERVAL = 60000; // Full payload setiap 1 menit

public:
    DeltaPayloadOptimizer() : lastSentPayload(1024) {}

    String createOptimalPayload(DynamicJsonDocument &current)
    {
        DynamicJsonDocument delta(512);
        bool sendFull = false;

        // Check if should send full payload (heartbeat)
        if ((millis() - lastSentTime) > FULL_PAYLOAD_INTERVAL)
        {
            sendFull = true;
        }

        if (sendFull)
        {
            lastSentPayload = current.as<JsonObject>();
            String result;
            serializeJson(current, result);
            lastSentTime = millis();
            Serial.printf("[DELTA] Sending FULL payload (%d bytes)\n", result.length());
            return result;
        }

        // Create delta payload
        bool hasDelta = false;

        for (JsonPair pair : current.as<JsonObject>())
        {
            const char *key = pair.key().c_str();

            // Always include time-based fields
            if (strcmp(key, "timestamp") == 0)
            {
                delta[key] = current[key];
                hasDelta = true;
            }
            // Include if value changed
            else if (!lastSentPayload.containsKey(key) ||
                     lastSentPayload[key] != current[key])
            {
                delta[key] = current[key];
                hasDelta = true;
            }
        }

        if (!hasDelta)
            return ""; // Nothing to send

        lastSentPayload = current.as<JsonObject>();
        lastSentTime = millis();

        String result;
        serializeJson(delta, result);

        String full;
        serializeJson(current, full);

        Serial.printf("[DELTA] %d -> %d bytes (%.1f%% reduction)\n",
                      full.length(), result.length(),
                      100.0f * (1.0f - (float)result.length() / full.length()));

        return result;
    }
};

// ============================================================================
// 7. RING BUFFER FOR QUEUE MANAGEMENT
// ============================================================================

struct DataPoint
{
    char json[512];
    uint32_t timestamp;
    uint8_t priority; // 0=normal, 1=high, 2=critical
};

template <size_t BufferSize = 100>
class RingBuffer
{
private:
    DataPoint buffer[BufferSize];
    volatile int writeIndex = 0;
    volatile int readIndex = 0;
    volatile int count = 0;
    SemaphoreHandle_t bufferMutex;

public:
    RingBuffer()
    {
        bufferMutex = xSemaphoreCreateMutex();
    }

    bool push(const DataPoint &point)
    {
        if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(100)) != pdTRUE)
            return false;

        if (count >= BufferSize)
        {
            xSemaphoreGive(bufferMutex);
            Serial.printf("[RingBuffer] FULL! Dropping oldest data\n");

            // Drop oldest (non-critical) data
            if (buffer[readIndex].priority == 0)
            {
                readIndex = (readIndex + 1) % BufferSize;
                count--;
            }
            else
            {
                xSemaphoreGive(bufferMutex);
                return false; // Don't drop critical data
            }
        }

        buffer[writeIndex] = point;
        buffer[writeIndex].timestamp = millis();
        writeIndex = (writeIndex + 1) % BufferSize;
        count++;

        xSemaphoreGive(bufferMutex);
        return true;
    }

    bool pop(DataPoint &point)
    {
        if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(100)) != pdTRUE)
            return false;

        if (count == 0)
        {
            xSemaphoreGive(bufferMutex);
            return false;
        }

        // Priority sorting: send critical first
        int indexToSend = readIndex;
        int searchCount = 0;

        while (searchCount < count && buffer[indexToSend].priority < 2)
        {
            indexToSend = (indexToSend + 1) % BufferSize;
            searchCount++;
        }

        point = buffer[indexToSend];

        // Shift if not first
        if (indexToSend != readIndex)
        {
            buffer[indexToSend] = buffer[readIndex];
        }

        readIndex = (readIndex + 1) % BufferSize;
        count--;

        xSemaphoreGive(bufferMutex);
        return true;
    }

    int getCount() const
    {
        return count;
    }

    int getCapacity() const
    {
        return BufferSize;
    }
};

#endif // NETWORK_OPTIMIZATION_HPP

// // ============================================================================
// // EXAMPLE USAGE
// // ============================================================================

// #include "NetworkOptimization.hpp"

// // Global instances
// SmartBatcher batcher;
// AdaptiveSampler sampler(4);  // 4 sensors
// SmartRetryManager retryManager({.maxRetries = 3, .initialDelay = 500});
// NetworkMonitor netMonitor;
// DeltaPayloadOptimizer deltaOptimizer;
// RingBuffer<100> dataBuffer;

// void setup()
// {
//     Serial.begin(115200);

//     // Configure samplers (threshold per sensor)
//     sampler.setSensor(0, 0.5);   // Sensor 0: threshold 0.5°C
//     sampler.setSensor(1, 1.0);   // Sensor 1: threshold 1.0
//     sampler.setSensor(2, 2.0);   // Sensor 2: threshold 2.0
//     sampler.setSensor(3, 0.1);   // Sensor 3: threshold 0.1
// }

// void collectAndBatchData()
// {
//     // Read sensors
//     float temp = readTemperature();
//     float humidity = readHumidity();
//     float pressure = readPressure();
//     float flow = readFlow();

//     // Check if should send (adaptive sampling)
//     DynamicJsonDocument doc(256);
//     if (sampler.shouldSend(0, temp))
//         doc["temp"] = temp;
//     if (sampler.shouldSend(1, humidity))
//         doc["humidity"] = humidity;
//     if (sampler.shouldSend(2, pressure))
//         doc["pressure"] = pressure;
//     if (sampler.shouldSend(3, flow))
//         doc["flow"] = flow;

//     if (doc.size() > 0)
//     {
//         // Add to batch
//         batcher.addData(doc.as<JsonObject>());

//         // Check if should send batch
//         if (batcher.shouldSendBatch())
//         {
//             String batchData = batcher.getBatchAndClear();
//             sendBatchWithRetry(batchData);
//         }
//     }
// }

// void sendBatchWithRetry(String data)
// {
//     retryManager.reset();
//     bool success = false;

//     while (retryManager.canRetry())
//     {
//         // Compress data
//         String compressed = compressDataGZIP(data);

//         // Try send
//         unsigned long startTime = millis();
//         bool result = sendHTTPRequest(compressed);
//         unsigned long responseTime = millis() - startTime;

//         if (result)
//         {
//             retryManager.recordSuccess();
//             success = true;

//             // Record metrics
//             netMonitor.recordRequest(
//                 compressed.length(),
//                 256,  // Assume 256 bytes response
//                 responseTime,
//                 true
//             );

//             break;
//         }
//         else
//         {
//             retryManager.recordFailure();

//             // Wait before retry
//             vTaskDelay(pdMS_TO_TICKS(retryManager.millisecondsUntilRetry()));
//         }
//     }

//     if (!success)
//     {
//         // Failed all retries - save to SD
//         saveToSD(data);
//         netMonitor.recordRequest(data.length(), 0, 0, false);
//     }

//     // Check and print metrics
//     netMonitor.checkAndPrint();
// }
