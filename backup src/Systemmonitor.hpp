#ifndef SYSTEM_MONITOR_HPP
#define SYSTEM_MONITOR_HPP
#include <Arduino.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_chip_info.h"
#include "soc/rtc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct SystemMetrics
{
    uint32_t freeHeap;
    uint32_t totalHeap;
    uint32_t minFreeHeap;
    uint32_t usedHeap;
    float heapUsagePercent;
    uint32_t freePSRAM;
    uint32_t totalPSRAM;
    uint32_t cpuFreqMHz;
    uint32_t apbFreqHz;
    uint8_t numCores;
    uint32_t numTasks;
    uint32_t stackHighWaterMark;
    uint32_t uptimeSeconds;
    uint32_t freeSketchSpace;
    uint32_t sketchSize;
};

class SystemMonitor
{
private:
    SystemMetrics beforeMetrics;
    SystemMetrics afterMetrics;
    unsigned long startTime;
    bool isMonitoring;

    void captureMetrics(SystemMetrics &metrics)
    {
        metrics.freeHeap = ESP.getFreeHeap();
        metrics.totalHeap = ESP.getHeapSize();
        metrics.minFreeHeap = ESP.getMinFreeHeap();
        metrics.usedHeap = metrics.totalHeap - metrics.freeHeap;
        metrics.heapUsagePercent = ((float)metrics.usedHeap / (float)metrics.totalHeap) * 100.0f;
        metrics.freePSRAM = ESP.getFreePsram();
        metrics.totalPSRAM = ESP.getPsramSize();
        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);
        metrics.cpuFreqMHz = ESP.getCpuFreqMHz();
        metrics.apbFreqHz = rtc_clk_apb_freq_get();
        metrics.numCores = chip_info.cores;
        metrics.numTasks = uxTaskGetNumberOfTasks();
        metrics.stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        metrics.uptimeSeconds = millis() / 1000;
        metrics.freeSketchSpace = ESP.getFreeSketchSpace();
        metrics.sketchSize = ESP.getSketchSize();
    }

    String formatBytes(uint32_t bytes)
    {
        if (bytes < 1024)
        {
            return String(bytes) + " B";
        }
        else if (bytes < 1024 * 1024)
        {
            return String(bytes / 1024.0, 2) + " KB";
        }
        else
        {
            return String(bytes / (1024.0 * 1024.0), 2) + " MB";
        }
    }

public:
    SystemMonitor() : isMonitoring(false) {}

    void begin()
    {
        startTime = millis();
        captureMetrics(beforeMetrics);
        isMonitoring = true;
    }

    void end()
    {
        if (!isMonitoring)
            return;
        captureMetrics(afterMetrics);
        isMonitoring = false;
        printComparison();
    }

    void printCurrent()
    {
        SystemMetrics c;
        captureMetrics(c);

        Serial.println("\n[SYSTEM STATUS]");
        // CPU & Chip Info
        Serial.printf("CPU: %u MHz | APB: %.1f MHz | Cores: %u | Rev: %u\n",
                      c.cpuFreqMHz, c.apbFreqHz / 1000000.0f, c.numCores, ESP.getChipRevision());

        // Memory (Heap & PSRAM)
        Serial.printf("Heap: %s/%s (%.1f%%) | Min Free: %s\n",
                      formatBytes(c.freeHeap).c_str(), formatBytes(c.totalHeap).c_str(), c.heapUsagePercent, formatBytes(c.minFreeHeap).c_str());

        if (c.totalPSRAM > 0)
        {
            float psramUsed = ((float)(c.totalPSRAM - c.freePSRAM) / c.totalPSRAM) * 100.0f;
            Serial.printf("PSRAM: %s/%s (%.1f%%)\n",
                          formatBytes(c.freePSRAM).c_str(), formatBytes(c.totalPSRAM).c_str(), psramUsed);
        }

        // Tasks & Flash
        Serial.printf("Tasks: %u | Stack: %u B | Uptime: %us | Flash: %s/%s\n",
                      c.numTasks, c.stackHighWaterMark, c.uptimeSeconds, formatBytes(c.sketchSize).c_str(), formatBytes(ESP.getFlashChipSize()).c_str());
    }

    void printComparison()
    {
        int32_t heapDelta = (int32_t)afterMetrics.freeHeap - (int32_t)beforeMetrics.freeHeap;
        int32_t tasksDelta = (int32_t)afterMetrics.numTasks - (int32_t)beforeMetrics.numTasks;
        int32_t stackDelta = (int32_t)afterMetrics.stackHighWaterMark - (int32_t)beforeMetrics.stackHighWaterMark;

        Serial.printf("\n[MONITOR RESULT] Duration: %lu ms\n", millis() - startTime);
        Serial.printf("Heap  : %s -> %s (Delta: %+d B)\n",
                      formatBytes(beforeMetrics.freeHeap).c_str(), formatBytes(afterMetrics.freeHeap).c_str(), heapDelta);
        Serial.printf("Tasks : %u -> %u (Delta: %+d)\n",
                      beforeMetrics.numTasks, afterMetrics.numTasks, tasksDelta);
        Serial.printf("Stack : %u -> %u (Delta: %+d B)\n",
                      beforeMetrics.stackHighWaterMark, afterMetrics.stackHighWaterMark, stackDelta);
    }

    SystemMetrics getCurrentMetrics()
    {
        SystemMetrics current;
        captureMetrics(current);
        return current;
    }

    void printQuickStats()
    {
        SystemMetrics current;
        captureMetrics(current);

        Serial.printf("[SYS] CPU:%uMHz | RAM:%s/%s (%.1f%%) | Tasks:%u | Uptime:%us\n",
                      current.cpuFreqMHz,
                      formatBytes(current.freeHeap).c_str(),
                      formatBytes(current.totalHeap).c_str(),
                      current.heapUsagePercent,
                      current.numTasks,
                      current.uptimeSeconds);
    }

    bool isMemoryLow(uint32_t thresholdBytes = 10240)
    {
        return (ESP.getFreeHeap() < thresholdBytes);
    }

    void checkAndPrintWarnings()
    {
        SystemMetrics c;
        captureMetrics(c);

        // Thresholds: Heap < 10KB, Stack < 512B, Usage > 80%
        if (c.freeHeap < 10240)
            Serial.printf("[WARN] Low Heap: %s\n", formatBytes(c.freeHeap).c_str());
        if (c.stackHighWaterMark < 512)
            Serial.printf("[WARN] Low Stack: %u B\n", c.stackHighWaterMark);
        if (c.heapUsagePercent > 80.0f)
            Serial.printf("[WARN] High Memory Usage: %.1f%%\n", c.heapUsagePercent);
    }
};

#endif