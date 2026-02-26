#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "AsyncJson.h"
#include <SPIFFS.h>
#include <Update.h>
#include <WiFiClientSecure.h>
// #include <ADS1X15.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Ethernet.h> //ethernet w5500
#include <esp_wifi.h>
#include "driver/spi_master.h"
#include <time.h>
#include <config.hpp>
#include <ModbusIP_ESP8266.h>
#include <ModbusRTU.h>
#include <DNSServer.h>
#include "NetworkFunctions.hpp"
#include <esp_task_wdt.h>
#include "SystemMonitor.hpp"

// Create instance
SystemMonitor sysMonitor;

// #define DEBUG
// ============================================================================
// TASK HANDLES
// ============================================================================
TaskHandle_t Task_Core0_Network = NULL;
TaskHandle_t Task_Core1_DataAcquisition = NULL;
TaskHandle_t Task_Core1_ModbusClient = NULL;
TaskHandle_t Task_Core1_DataLogger = NULL;
TaskHandle_t Task_Core0_HTTPSend = NULL;
// QUEUE HANDLES untuk komunikasi antar task
struct HttpSendPacket
{
  char data[512];
  char url[128];
  char username[64];
  char password[64];
};

QueueHandle_t queueSensorData = NULL;
QueueHandle_t queueModbusData = NULL;
QueueHandle_t queueLogData = NULL;
QueueHandle_t queueHttpSend = NULL;

// ============================================================================
// MUTEX untuk resource sharing
// ============================================================================
SemaphoreHandle_t spiMutex;
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t sdMutex;
SemaphoreHandle_t jsonMutex;
SemaphoreHandle_t modbusMutex;
// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
ModbusIP mbIP;
ModbusRTU mbRTU;
RTC_DS3231 rtc;
AsyncWebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;
bool dnsStarted = false;
WiFiClient esp32;
PubSubClient mqtt;
// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
Network networkSettings; // Allocate memory here
AnalogInput analogInput[jumlahInputAnalog + 1];
DigitalInput digitalInput[jumlahInputDigital + 1];
DigitalOutput digitalOutput[jumlahOutputDigital + 1];
ModbusParam modbusParam;
unsigned long timeElapsed;
unsigned long lastSDSaveTime = 0;
// ADS1115 ads;
Adafruit_ADS1115 ads;
ErrorBlinker errorBlinker(SIG_LED_PIN, 800);
ErrorMessages errorMessages("/debugStream");

String stringParam, sendString;
int numOfParam, modbusCount;
bool flagSend = false;
unsigned long printTime, checkTime, sendTime, sendTimeModbus;
HardwareSerial SerialModbus(2);

DynamicJsonDocument doc(4096), jsonParam(4096), jsonSend(4096);
bool flagGetJobNum = 1;
String jobNum;

// ============================================================================
// STRUKTUR DATA untuk Queue
// ============================================================================
struct SensorDataPacket
{
  float analogValues[jumlahInputAnalog + 1];
  float digitalValues[jumlahInputDigital + 1];
  unsigned long timestamp;
};

struct ModbusDataPacket
{
  String paramName;
  float value;
  unsigned long timestamp;
};

struct LogDataPacket
{
  String jsonData;
  unsigned long timestamp;
};

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void readConfig();
void saveToJson(const char *dir, const char *configType);
void saveToSDConfig(const char *dir, const char *configType);
void updateJson(const char *dir, const char *jsonKey, int jsonValue);
void updateJson(const char *dir, const char *jsonKey, const char *jsonValue);
void handleFileRequest(AsyncWebServerRequest *request, const char *filePath, const char *mimeType);
String getTimeDateNow();
String getTimeNow();
void modbusSlaveSetup();
void setupWebServer();
void setupInterrupts();
bool isAuthenticated(AsyncWebServerRequest *request);
void authenthicateUser(AsyncWebServerRequest *request);
void handleFormSubmit(AsyncWebServerRequest *request);
void printConfigurationDetails();
int countJsonKeys(const JsonDocument &doc);
float filterSensor(float filterVar, float filterResult_1, float fc);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
float calculate_Measurement(float mA, float minRange, float maxRange);

// Update deklarasi agar cocok dengan definisi di bawah (tambah bool &success)
unsigned int readModbusNonBlocking(unsigned int modbusAddress, unsigned int funCode, unsigned int regAddress, unsigned int timeoutMs, bool &success);
unsigned int readModbus(unsigned int modbusAddress, unsigned int funCode, unsigned int regAddress);
unsigned int crcModbus(unsigned int crc[], byte start, byte sizeArray);
unsigned int parseByte(unsigned int bytes, bool byteOrder);

// ISR DECLARATIONS
#define DEBOUNCE_TIME 5
void IRAM_ATTR isrDI1()
{
  digitalInput[1].millisNow = millis(); // selalu update timestamp
  digitalInput[1].flagInt = 1;
}

void IRAM_ATTR isrDI2()
{
  digitalInput[2].millisNow = millis();
  digitalInput[2].flagInt = 1;
}

void IRAM_ATTR isrDI3()
{
  digitalInput[3].millisNow = millis();
  digitalInput[3].flagInt = 1;
}

void IRAM_ATTR isrDI4()
{
  digitalInput[4].millisNow = millis();
  digitalInput[4].flagInt = 1;
}

void (*isrArray[])(void) = {nullptr, isrDI1, isrDI2, isrDI3, isrDI4};

void attachDigitalInputInterrupt(int index)
{
  if (digitalInput[index].taskMode == "Cycle Time" || digitalInput[index].taskMode == "Counting")
  {
    if (index < sizeof(isrArray) / sizeof(isrArray[0]))
    {
      attachInterrupt(digitalPinToInterrupt(digitalInput[index].pin), isrArray[index], RISING);
    }
  }
  else if (digitalInput[index].taskMode == "Pulse Mode")
  {
    if (index < sizeof(isrPulseMode) / sizeof(isrPulseMode[0]))
    {
      attachInterrupt(digitalPinToInterrupt(digitalInput[index].pin), isrPulseMode[index], RISING);
    }
  }
  else
  {
    detachInterrupt(digitalPinToInterrupt(digitalInput[index].pin));
  }
}

void configureSendTriggerInterrupt(Network networkSettings)
{
  if (networkSettings.sendTrig != "Timer/interval")
  {
    int pinTrig = networkSettings.sendTrig.substring(2, 3).toInt();
    if (pinTrig >= 1 && pinTrig < sizeof(isrArray) / sizeof(isrArray[0]))
    {
      attachInterrupt(digitalPinToInterrupt(digitalInput[pinTrig].pin), isrArray[pinTrig], RISING);
    }
  }
}

// ETHERNET WEB SERVER HANDLER
String getContentType(String filename)
{
  filename.toLowerCase();
  static const struct
  {
    const char *ext;
    const char *mime;
  } table[] = {
      {".html", "text/html; charset=utf-8"},
      {".htm", "text/html; charset=utf-8"},
      {".css", "text/css; charset=utf-8"},
      {".js", "application/javascript; charset=utf-8"},
      {".json", "application/json"},
      {".png", "image/png"},
      {".jpg", "image/jpeg"},
      {".jpeg", "image/jpeg"},
      {".gif", "image/gif"},
      {".ico", "image/x-icon"},
      {".svg", "image/svg+xml"},
      {".woff", "font/woff"},
      {".woff2", "font/woff2"},
      {".ttf", "font/ttf"},
      {".eot", "application/vnd.ms-fontobject"},
  };
  for (auto &e : table)
  {
    if (filename.endsWith(e.ext))
      return e.mime;
  }
  return "text/plain";
}

// 1. HELPER FUNCTIONS
unsigned char h2int(char c)
{
  if (c >= '0' && c <= '9')
    return ((unsigned char)c - '0');
  if (c >= 'a' && c <= 'f')
    return ((unsigned char)c - 'a' + 10);
  if (c >= 'A' && c <= 'F')
    return ((unsigned char)c - 'A' + 10);
  return 0;
}

String urlDecode(String str)
{
  String encodedString = "";
  char c, code0, code1;
  for (int i = 0; i < str.length(); i++)
  {
    c = str.charAt(i);
    if (c == '+')
    {
      encodedString += ' ';
    }
    else if (c == '%')
    {
      i++;
      code0 = str.charAt(i);
      i++;
      code1 = str.charAt(i);
      c = (h2int(code0) << 4) | h2int(code1);
      encodedString += c;
    }
    else
    {
      encodedString += c;
    }
  }
  return encodedString;
}

String getPostValue(String data, String key)
{
  String keyParam = key + "=";
  int keyStart = data.indexOf(keyParam);
  if (keyStart == -1)
    return "";
  int valStart = keyStart + keyParam.length();
  int valEnd = data.indexOf("&", valStart);
  if (valEnd == -1)
    valEnd = data.length();
  return urlDecode(data.substring(valStart, valEnd));
}

void handleEthernetClient()
{
  EthernetClient client = ethServer.available();
  if (!client)
    return;

  // [SETTING] Timeout & Buffer
  client.setTimeout(100);
  client.setConnectionTimeout(500);

  String req = "";
  String postData = "";
  unsigned long timeout = millis();
  bool headerFinished = false;
  int contentLength = 0;
  uint8_t tempBuf[256];

  // 1. READ REQUEST (Membaca Data dari Browser)
  while (client.connected() && millis() - timeout < 1000)
  {
    if (client.available())
    {
      int len = client.read(tempBuf, sizeof(tempBuf));
      if (len > 0)
      {
        if (!headerFinished)
        {
          for (int i = 0; i < len; i++)
          {
            req += (char)tempBuf[i];
            if (req.endsWith("\r\n\r\n"))
            {
              headerFinished = true;
              String lowerReq = req;
              lowerReq.toLowerCase();
              int clIndex = lowerReq.indexOf("content-length:");
              if (clIndex != -1)
              {
                contentLength = lowerReq.substring(clIndex + 15, lowerReq.indexOf('\n', clIndex)).toInt();
              }
              for (int j = i + 1; j < len; j++)
                postData += (char)tempBuf[j];
              break;
            }
          }
        }
        else
        {
          for (int i = 0; i < len; i++)
            postData += (char)tempBuf[i];
        }
      }
      if (headerFinished)
      {
        if (req.startsWith("GET"))
          break;
        if (req.startsWith("POST") && postData.length() >= contentLength)
          break;
      }
    }
    vTaskDelay(1);
  }

  if (req.length() == 0)
  {
    client.stop();
    return;
  }

  // 2. PARSE URL & METHOD
  int firstSpace = req.indexOf(' ');
  int secondSpace = req.indexOf(' ', firstSpace + 1);
  if (firstSpace == -1 || secondSpace == -1)
  {
    client.stop();
    return;
  }

  String method = req.substring(0, firstSpace);
  String fullUrl = req.substring(firstSpace + 1, secondSpace);
  fullUrl.trim();

  String basePath = fullUrl;
  String queryParams = "";
  if (fullUrl.indexOf("?") > 0)
  {
    basePath = fullUrl.substring(0, fullUrl.indexOf("?"));
    queryParams = fullUrl.substring(fullUrl.indexOf("?") + 1);
  }

  // 3. PARSE JSON BODY (Penting untuk Save Config dari Web Modern)
  bool isJson = false;
  DynamicJsonDocument jsonBody(2048);
  postData.trim();
  if (method == "POST" && (postData.startsWith("{") || postData.startsWith("[")))
  {
    DeserializationError error = deserializeJson(jsonBody, postData);
    if (!error)
      isJson = true;
  }

  // Helper Lambda: Otomatis pilih ambil data dari JSON atau Form Data
  auto getValue = [&](String key) -> String
  {
    if (isJson && jsonBody.containsKey(key))
      return jsonBody[key].as<String>();
    return getPostValue(postData, key);
  };

  // ========================================================================
  // A. POST HANDLING (SAVE CONFIGURATION)
  // ========================================================================
  if (method == "POST")
  {
    client.println("HTTP/1.1 200 OK");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();

    // --- 1. SAVE ANALOG INPUT (FIXED JSON SUPPORT) ---
    if (basePath == "/analog_input" || getValue("inputPin").startsWith("AI"))
    {
      String targetPin = getValue("inputPin");
      if (targetPin.startsWith("AI"))
      {
        int id = targetPin.substring(2).toInt();
        if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(500)))
        {
          analogInput[id].name = getValue("name");
          analogInput[id].inputType = getValue("inputType");

          // [FIX UTAMA] Deteksi Boolean dari JSON vs Form
          if (isJson)
          {
            analogInput[id].filter = jsonBody["filter"];
            analogInput[id].scaling = jsonBody["scaling"];
            analogInput[id].calibration = jsonBody["calibration"];
          }
          else
          {
            // Fallback Form Data (indexOf)
            analogInput[id].filter = (postData.indexOf("filter=") != -1);
            analogInput[id].scaling = (postData.indexOf("scaling=") != -1);
            analogInput[id].calibration = (postData.indexOf("calibration=") != -1);
          }

          // Ambil Angka (getValue otomatis handle JSON)
          analogInput[id].lowLimit = getValue("lowLimit").toFloat();
          analogInput[id].highLimit = getValue("highLimit").toFloat();
          analogInput[id].mValue = getValue("mValue").toFloat();
          analogInput[id].cValue = getValue("cValue").toFloat();
          analogInput[id].filterPeriod = getValue("filterPeriod").toFloat();

          xSemaphoreGive(jsonMutex);
        }
        saveToJson("/configAnalog.json", "analog");
        saveToSDConfig("/configAnalog.json", "analog");
        client.print("Analog Saved");
      }
    }

    // --- 2. SAVE DIGITAL INPUT ---
    else if (basePath == "/digital_IO" || getValue("inputPin").startsWith("DI"))
    {
      String targetPin = getValue("inputPin");
      if (targetPin.startsWith("DI"))
      {
        int id = targetPin.substring(2).toInt();
        if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(500)))
        {
          digitalInput[id].name = getValue("nameDI");
          digitalInput[id].taskMode = getValue("taskMode");
          if (digitalInput[id].taskMode != getValue("taskMode"))
            digitalInput[id].value = 0;
          digitalInput[id].inputState = (getValue("inputState") == "High") ? 1 : 0;

          if (isJson)
            digitalInput[id].inv = jsonBody.containsKey("inputInversion") && jsonBody["inputInversion"];
          else
            digitalInput[id].inv = (postData.indexOf("inputInversion=") != -1);

          digitalInput[id].intervalTime = (long)(getValue("intervalTime").toFloat() * 1000);
          digitalInput[id].conversionFactor = getValue("conversionFactor").toFloat();
          attachDigitalInputInterrupt(id);
          xSemaphoreGive(jsonMutex);
        }
        saveToJson("/configDigital.json", "digital");
        saveToSDConfig("/configDigital.json", "digital");
        client.print("Digital Saved");
      }
    }

    // --- 3. SAVE NETWORK ---
    else if (basePath == "/network" || getValue("networkMode") != "" || getValue("erpUrl") != "")
    {
      if (getValue("erpUrl") != "")
      {
        networkSettings.erpUrl = getValue("erpUrl");
        networkSettings.erpUsername = getValue("erpUsername");
        networkSettings.erpPassword = getValue("erpPassword");
      }
      if (getValue("networkMode") != "")
      {
        networkSettings.networkMode = getValue("networkMode");
        networkSettings.dhcpMode = getValue("dhcpMode");
        networkSettings.ssid = getValue("ssid");
        networkSettings.password = getValue("password");
        networkSettings.apSsid = getValue("apSsid");
        networkSettings.apPassword = getValue("apPassword");
        networkSettings.ipAddress = getValue("ipAddress");
        networkSettings.subnetMask = getValue("subnet");
        networkSettings.ipGateway = getValue("ipGateway");
        networkSettings.ipDNS = getValue("ipDNS");
        networkSettings.protocolMode = getValue("protocolMode");
        networkSettings.endpoint = getValue("endpoint");
        networkSettings.port = getValue("port").toInt();
        networkSettings.sendInterval = getValue("sendInterval").toFloat();
        networkSettings.sendTrig = getValue("sendTrig");
        networkSettings.mqttUsername = getValue("mqttUsername");
        networkSettings.mqttPassword = getValue("mqttPass");
        networkSettings.pubTopic = getValue("pubTopic");
        networkSettings.subTopic = getValue("subTopic");
        networkSettings.loggerMode = getValue("loggerMode").length() > 0 ? getValue("loggerMode") : "Disabled";

        if (getValue("modbusMode") != "")
        {
          networkSettings.protocolMode2 = getValue("protocolMode2");
          modbusParam.port = getValue("modbusPort").toInt();
          modbusParam.slaveID = getValue("slaveID").toInt();
        }
        configureSendTriggerInterrupt(networkSettings);
      }
      saveToJson("/configNetwork.json", "network");
      saveToSDConfig("/configNetwork.json", "network");
      // client.println("HTTP/1.1 200 OK");
      // client.println("Content-Type: text/plain");
      // client.println("Connection: close");
      // client.println();
      client.print("Network saved. Restarting...");
      client.flush();
      client.stop();
      delay(500);
    }

    // --- 4. SAVE MODBUS SETUP ---
    else if (basePath == "/modbus_setup")
    {
      if (isJson)
      {
        jsonParam = jsonBody; // Full copy JSON structure
        modbusParam.baudrate = jsonParam["baudrate"];
        modbusParam.parity = jsonParam["parity"].as<String>();
        modbusParam.stopBit = jsonParam["stopBit"];
        modbusParam.dataBit = jsonParam["dataBit"];
        modbusParam.scanRate = jsonParam["scanRate"];
      }
      else if (getValue("baudrate") != "")
      {
        modbusParam.baudrate = getValue("baudrate").toInt();
        modbusParam.parity = getValue("parity");
        modbusParam.stopBit = getValue("stopBit").toInt();
        modbusParam.dataBit = getValue("dataBit").toInt();
        modbusParam.scanRate = getValue("scanRate").toFloat();
        // Update global JSON
        jsonParam["baudrate"] = modbusParam.baudrate;
        jsonParam["parity"] = modbusParam.parity;
        jsonParam["stopBit"] = modbusParam.stopBit;
        jsonParam["dataBit"] = modbusParam.dataBit;
        jsonParam["scanRate"] = modbusParam.scanRate;
      }
      stringParam = "";
      serializeJson(jsonParam, stringParam);
      saveToJson("/modbusSetup.json", "modbusSetup");
      saveToSDConfig("/modbusSetup.json", "modbusSetup");
      client.print("Modbus Saved");
    }

    // --- 5. SAVE SYSTEM SETTINGS ---
    else if (basePath == "/system_settings" || getValue("username") != "")
    {
      networkSettings.loginUsername = getValue("username");
      networkSettings.loginPassword = getValue("password");
      networkSettings.sdSaveInterval = getValue("sdInterval").toInt();

      String dt = getValue("datetime");
      if (dt.length() >= 16)
      {
        int yy = dt.substring(0, 4).toInt();
        int mm = dt.substring(5, 7).toInt();
        int dd = dt.substring(8, 10).toInt();
        int hh = dt.substring(11, 13).toInt();
        int mn = dt.substring(14, 16).toInt();
        rtc.adjust(DateTime(yy, mm, dd, hh, mn, 0));
      }
      saveToJson("/systemSettings.json", "systemSettings");
      saveToSDConfig("/systemSettings.json", "systemSettings");
      client.print("Settings Saved");
    }
    else
    {
      client.print("OK");
    }
  }

  // ========================================================================
  // B. GET HANDLING (LOAD DATA API) - FIXED REALTIME VALUE
  // ========================================================================
  else if (method == "GET")
  {
    if (basePath.endsWith("Load") || basePath.endsWith("Value") ||
        basePath.endsWith("Status") || basePath == "/getTime")
    {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Cache-Control: no-store, no-cache, must-revalidate");
      client.println("Connection: close");
      client.println();

      // --- 1. GET VALUE
      if (basePath == "/getValue")
      {
        String payload = "[";
        bool first = true;
        bool mutexTaken = false;

        // 1. Generate the JSON String
        if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(500)))
        {
          mutexTaken = true;
          if (!jsonSend.isNull() && jsonSend.size() > 0)
          {
            JsonObject root = jsonSend.as<JsonObject>();
            for (JsonPair kv : root)
            {
              String key = kv.key().c_str();
              if (key == "-" || key == "")
                continue;

              if (!first)
                payload += ",";
              String val = kv.value().as<String>();
              if (val == "null" || val == "" || val == "undefined")
                val = "0";
              payload += "{\"KodeSensor\":\"" + key + "\",\"Value\":\"" + val + "\"}";
              first = false;
            }
          }
          xSemaphoreGive(jsonMutex);
        }
        payload += "]";
        client.print(payload);
      }

      // --- 2. GET CURRENT VALUE (ANALOG/DIGITAL REALTIME) ---
      else if (basePath == "/getCurrentValue")
      {
        if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(100)))
        {
          DynamicJsonDocument docTemp(2048);
          // Analog Input
          for (int i = 1; i <= jumlahInputAnalog; i++)
          {
            docTemp["rawValue" + String(i)] = analogInput[i].adcValue;
            docTemp["scaledValue" + String(i)] = analogInput[i].mapValue;
          }
          // Digital Input
          for (int i = 1; i <= jumlahInputDigital; i++)
          {
            docTemp["diValue" + String(i)] = digitalInput[i].value;
          }
          String dataLoad;
          serializeJson(docTemp, dataLoad);
          xSemaphoreGive(jsonMutex);
          client.print(dataLoad);
        }
        else
        {
          client.print("{}");
        }
      }

      // --- 3. HOME LOAD (DASHBOARD DATA) ---
      else if (basePath == "/homeLoad")
      {
        if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
        {
          doc = DynamicJsonDocument(4096);
          doc["networkMode"] = networkSettings.networkMode;
          doc["ssid"] = networkSettings.ssid;
          doc["ipAddress"] = Ethernet.localIP().toString();
          doc["connStatus"] = networkSettings.connStatus;
          doc["jobNumber"] = jobNum;
          doc["datetime"] = getTimeDateNow();

          // Struktur Data Analog
          JsonObject AI = doc.createNestedObject("AI");
          JsonArray AIRawValue = AI.createNestedArray("rawValue");
          JsonArray AIScaledValue = AI.createNestedArray("scaledValue");
          JsonArray EnAI = AI.createNestedArray("enAI");

          for (int i = 1; i <= jumlahInputAnalog; i++)
          {
            AIRawValue.add(analogInput[i].adcValue);
            AIScaledValue.add(analogInput[i].mapValue);
            EnAI.add(analogInput[i].name != "" ? 1 : 0);
          }

          // Struktur Data Digital
          JsonObject DI = doc.createNestedObject("DI");
          JsonArray DIValue = DI.createNestedArray("value");
          JsonArray DITaskMode = DI.createNestedArray("taskMode");
          for (int i = 1; i <= jumlahInputDigital; i++)
          {
            DIValue.add(digitalInput[i].value);
            DITaskMode.add(digitalInput[i].taskMode);
          }

          String dataLoad;
          serializeJson(doc, dataLoad);
          xSemaphoreGive(jsonMutex);
          client.print(dataLoad);
        }
      }

      // --- 4. ANALOG LOAD (CONFIG) ---
      else if (basePath == "/analogLoad")
      {
        int id = 1;
        if (queryParams.indexOf("input=") >= 0)
          id = queryParams.substring(queryParams.indexOf("input=") + 6).toInt();
        if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
        {
          doc = DynamicJsonDocument(2048);
          doc["name"] = analogInput[id].name;
          doc["inputType"] = analogInput[id].inputType;
          doc["filter"] = analogInput[id].filter;
          doc["filterPeriod"] = String(analogInput[id].filterPeriod, 2);
          doc["scaling"] = analogInput[id].scaling;
          doc["lowLimit"] = String(analogInput[id].lowLimit, 2);
          doc["highLimit"] = String(analogInput[id].highLimit, 2);
          doc["mValue"] = String(analogInput[id].mValue, 6); // Gradient butuh presisi tinggi
          doc["cValue"] = String(analogInput[id].cValue, 4);
          doc["calibration"] = analogInput[id].calibration;
          String res;
          serializeJson(doc, res);
          xSemaphoreGive(jsonMutex);
          client.print(res);
        }
      }

      // --- 5. DIGITAL LOAD (CONFIG) ---
      else if (basePath == "/digitalLoad")
      {
        if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
        {
          doc = DynamicJsonDocument(2048);
          if (queryParams.indexOf("input=") >= 0)
          {
            int id = queryParams.substring(queryParams.indexOf("input=") + 6).toInt();
            doc["nameDI"] = digitalInput[id].name;
            doc["invDI"] = digitalInput[id].inv;
            doc["taskMode"] = digitalInput[id].taskMode;
            doc["inputState"] = digitalInput[id].inputState ? "High" : "Low";
            doc["intervalTime"] = (float)digitalInput[id].intervalTime / 1000;
            doc["conversionFactor"] = digitalInput[id].conversionFactor;
          }
          if (queryParams.indexOf("reset=") >= 0)
          {
            int id = queryParams.substring(queryParams.indexOf("reset=") + 6).toInt();
            digitalInput[id].value = 0;
          }
          String res;
          serializeJson(doc, res);
          xSemaphoreGive(jsonMutex);
          client.print(res);
        }
      }

      // --- 6. NETWORK LOAD ---
      else if (basePath == "/networkLoad")
      {
        if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
        {
          doc = DynamicJsonDocument(4096);
          doc["networkMode"] = networkSettings.networkMode;
          doc["ssid"] = networkSettings.ssid;
          doc["ipAddress"] = networkSettings.ipAddress;
          doc["sendInterval"] = String(networkSettings.sendInterval, 2);
          doc["dhcpMode"] = networkSettings.dhcpMode;
          doc["subnet"] = networkSettings.subnetMask;
          doc["ipGateway"] = networkSettings.ipGateway;
          doc["ipDNS"] = networkSettings.ipDNS;
          doc["protocolMode"] = networkSettings.protocolMode;
          doc["endpoint"] = networkSettings.endpoint;
          doc["port"] = networkSettings.port;
          doc["pubTopic"] = networkSettings.pubTopic;
          doc["subTopic"] = networkSettings.subTopic;
          doc["mqttUsername"] = networkSettings.mqttUsername;
          doc["mqttPass"] = networkSettings.mqttPassword;
          doc["erpUrl"] = networkSettings.erpUrl;
          doc["erpUsername"] = networkSettings.erpUsername;
          doc["erpPassword"] = networkSettings.erpPassword;
          doc["modbusMode"] = modbusParam.mode;
          doc["modbusPort"] = modbusParam.port;
          doc["modbusSlaveID"] = modbusParam.slaveID;
          String res;
          serializeJson(doc, res);
          xSemaphoreGive(jsonMutex);
          client.print(res);
        }
      }

      // --- 7. MODBUS LOAD ---
      else if (basePath == "/modbusLoad")
      {
        client.print(stringParam.length() > 0 ? stringParam : "{}");
      }

      // --- 8. SETTINGS LOAD ---
      else if (basePath == "/settingsLoad")
      {
        String jsonAuth = "{";
        jsonAuth += "\"username\":\"" + networkSettings.loginUsername + "\",";
        jsonAuth += "\"password\":\"" + networkSettings.loginPassword + "\",";
        jsonAuth += "\"sdInterval\":" + String(networkSettings.sdSaveInterval);
        jsonAuth += "}";
        client.print(jsonAuth);
      }

      // --- 9. GET TIME ---
      else if (basePath == "/getTime")
      {
        client.printf("{\"datetime\":\"%s\"}", getTimeDateNow().c_str());
      }

      // --- 10. STATUS ---
      else if (basePath == "/wifiStatus" || basePath == "/ethernetStatus")
      {
        DynamicJsonDocument stat(512);
        stat["mode"] = networkSettings.networkMode;
        stat["ip"] = Ethernet.localIP().toString();
        stat["connected"] = (Ethernet.linkStatus() == LinkON);
        stat["connectionStatus"] = networkSettings.connStatus;
        String res;
        serializeJson(stat, res);
        client.print(res);
      }
    }
    // ========================================================================
    // C. STATIC FILE HANDLER (HTML/CSS/JS)
    // ========================================================================
    else
    {
      String filePath = basePath;
      if (basePath == "/" || basePath == "/home")
        filePath = "/home.html";
      else if (basePath == "/network")
        filePath = "/network.html";
      else if (basePath == "/analog_input")
        filePath = "/analog_input.html";
      else if (basePath == "/digital_IO")
        filePath = "/digital_IO.html";
      else if (basePath == "/system_settings")
        filePath = "/system_settings.html";
      else if (basePath == "/modbus_setup")
        filePath = "/modbus_setup.html";
      else if (basePath == "/updateOTA")
        filePath = "/UpdateOTA.html";
      else if (basePath == "/debug")
        filePath = "/debug-monitor.html";
      String contentType = getContentType(filePath);

      // 2. DETEKSI FILE (GZIP vs NORMAL)
      bool fileFound = false;
      bool isGzip = false;

      // Cek file normal dulu
      if (SPIFFS.exists(filePath))
      {
        fileFound = true;
      }
      // Cek file terkompresi (.gz)
      else if (SPIFFS.exists(filePath + ".gz"))
      {
        filePath += ".gz"; // Ubah path menunjuk ke file .gz
        fileFound = true;
        isGzip = true;
      }

      if (fileFound)
      {
        File file = SPIFFS.open(filePath, "r");

        client.println("HTTP/1.1 200 OK");

        // Kirim Content-Type yang benar (misal: text/css)
        // Walaupun filenya .gz, browser perlu tahu ini aslinya CSS
        client.println("Content-Type: " + contentType);

        // Header Gzip (Wajib ada jika file diambil dari .gz)
        if (isGzip)
        {
          client.println("Content-Encoding: gzip");
        }

        // Cache Control (Penting agar load cepat saat refresh biasa)
        // Kita cek ekstensi asli atau ekstensi .gz
        if (contentType.indexOf("css") >= 0 || contentType.indexOf("javascript") >= 0 || contentType.indexOf("image") >= 0)
        {
          client.println("Cache-Control: public, max-age=31536000");
        }

        client.println("Connection: close");
        client.println();

        // BUFFERED SENDING
        uint8_t buffer[512];
        while (file.available())
        {
          int bytesRead = file.read(buffer, sizeof(buffer));
          client.write(buffer, bytesRead);
        }
        file.close();
      }
      else
      {
        client.println("HTTP/1.1 404 Not Found");
        client.println("Connection: close");
        client.println();
      }
    }
  }

  client.flush();
  vTaskDelay(pdMS_TO_TICKS(10));
  client.stop();
}
// ============================================================================
// CORE 0 TASK: Network Management (FIXED VERSION - ANTI REBOOT)
// ============================================================================
void Task_NetworkManagement(void *parameter)
{
  ESP_LOGI("Core0", "Network Management Task started");

  unsigned long lastNetCheck = 0;
  unsigned long lastDNSProcess = 0;
  unsigned long lastMQTTCheck = 0;
  unsigned long lastStatusPrint = 0;
  unsigned long lastWatchdogFeed = 0;
  unsigned long lastPingCheck = 0;
  const unsigned long PING_INTERVAL = 5000;
  esp_task_wdt_add(NULL);
  vTaskDelay(pdMS_TO_TICKS(1000));

  while (true)
  {
    esp_task_wdt_reset();
    // ============================================================
    // YIELD CPU SETIAP 5 DETIK (BIARKAN IDLE TASK RESET WATCHDOG)
    // ============================================================
    if (millis() - lastWatchdogFeed >= 5000)
    {
      vTaskDelay(pdMS_TO_TICKS(50)); // Beri waktu ke IDLE task
      lastWatchdogFeed = millis();
    }

    // ============================================================
    // 1. HANDLE ETHERNET SERVER
    // ============================================================
    if (networkSettings.networkMode == "Ethernet")
    {
      if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
      {
        EthernetClient client = ethServer.available();
        if (client)
        {
          client.setTimeout(100);
          unsigned long startProcess = millis();
          handleEthernetClient();

          if (millis() - startProcess > 500)
          {
            Serial.printf("⚠️ handleEthernetClient() took %lu ms\n",
                          millis() - startProcess);
          }
        }
        xSemaphoreGive(spiMutex);
        vTaskDelay(1);
      }
    }

    // ============================================================
    // 2. HANDLE DNS SERVER
    // ============================================================
    if (millis() - lastDNSProcess >= 50)
    {
      if (dnsStarted)
      {
        dnsServer.processNextRequest();
      }
      lastDNSProcess = millis();
    }

    // ============================================================
    // 3. WIFI CHECK
    // ============================================================
    if (millis() - lastNetCheck >= 10000)
    {
      checkWiFi(5000);
      lastNetCheck = millis();
    }
    if (millis() - lastPingCheck >= PING_INTERVAL)
    {
      bool linkUp = (networkSettings.networkMode == "Ethernet") ? (Ethernet.linkStatus() == LinkON) : (WiFi.status() == WL_CONNECTED);
      if (linkUp)
      {
        if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(3000)))
        {
          Serial.println("\n------------------------------------------------");
          Serial.println("[ 📡 Connectivity Check ]");

          // --- TARGET 1: IP Local (10.22.7.3) ---
          // Catatan: Ini cek Port 80. Jika target tidak punya Web Server, ini akan Timeout.
          String target1 = "10.22.7.3";
          long lat1 = measureLatency(target1);

          // --- TARGET 2: API Medion (HTTPS Port 443) ---
          String target2 = "https://api-logger-dev2.medionindonesia.com/api/v1/UpdateLoggingRealtime";
          long lat2 = measureLatency(target2);

          // --- TAMPILKAN HASIL ---
          Serial.print("   1. IP Local (10.22.7.3) : ");
          if (lat1 >= 0)
          {
            Serial.print(lat1);
            Serial.println(" ms (ONLINE ✅)");
          }
          else
          {
            Serial.println("TIMEOUT ❌ (Port 80 Closed/Unreachable)");
          }
          Serial.print("   2. API Logger Medion    : ");
          if (lat2 >= 0)
          {
            Serial.print(lat2);
            Serial.println(" ms (ONLINE ✅)");
            // Jika API sukses ping, kita anggap koneksi aman
            networkSettings.connStatus = "Connected";
          }
          else
          {
            Serial.println("TIMEOUT ❌");
            networkSettings.connStatus = "Not Connected";
          }
          Serial.println("------------------------------------------------\n");
          xSemaphoreGive(spiMutex);
        }
        else
        {
          Serial.println("[Ping] Skipped (SPI Busy)");
        }
      }
      else
      {
        Serial.println("[Ping] Skipped (No Physical Link)");
      }
      lastPingCheck = millis();
    }
    // ============================================================
    // 4. MQTT HANDLING
    // ============================================================
    if (networkSettings.protocolMode == "MQTT")
    {
      if (millis() - lastMQTTCheck >= 1000) // Cek setiap 1 detik
      {
        bool isNetworkConnected = (networkSettings.networkMode == "Ethernet") ? (Ethernet.linkStatus() == LinkON) : (WiFi.status() == WL_CONNECTED);

        if (isNetworkConnected)
        {
          if (!mqtt.connected())
          {
            Serial.print("Connecting to MQTT Broker...");
            String clientId = "ESP32-Node-" + networkSettings.macAddress;

            // Coba Connect
            if (mqtt.connect(clientId.c_str(),
                             networkSettings.mqttUsername.c_str(),
                             networkSettings.mqttPassword.c_str()))
            {
              Serial.println(" CONNECTED!");
              if (networkSettings.subTopic.length() > 0)
              {
                mqtt.subscribe(networkSettings.subTopic.c_str());
              }
            }
            else
            {
              Serial.print(" FAILED, rc=");
              Serial.println(mqtt.state());
            }
          }
          else
          {
            mqtt.loop();
          }
        }
        lastMQTTCheck = millis();
      }
    }

    // ============================================================
    // 5. MODBUS TCP
    // ============================================================
    bool useTCP = (networkSettings.protocolMode2.indexOf("TCP") >= 0);

    if (useTCP)
    {
      if (xSemaphoreTake(modbusMutex, pdMS_TO_TICKS(5)) == pdTRUE)
      {
        mbIP.task();
        xSemaphoreGive(modbusMutex);
      }
    }
    // 6. UTILITY & STATUS
    errorBlinker.update();
    sysMonitor.checkAndPrintWarnings();
    if (millis() - lastStatusPrint >= 5000)
    {
      sysMonitor.printCurrent();
      String ethStatus = "Off";
      String wifiStatus = "Off";
      String ethIP = "0.0.0.0";
      String wifiIP = "0.0.0.0";

      // Cek Ethernet
      if (networkSettings.networkMode == "Ethernet")
      {
        if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(20)))
        {
          ethStatus = (Ethernet.linkStatus() == LinkON) ? "LINK ON" : "NO CABLE";
          ethIP = Ethernet.localIP().toString();
          xSemaphoreGive(spiMutex);
        }
      }
      wifiStatus = (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "DISCONNECTED";
      wifiIP = WiFi.localIP().toString();

      Serial.println("\n-----------------[ SYSTEM STATUS ]-----------------");
      Serial.printf("Mode Active : %s\n", networkSettings.networkMode.c_str());
      Serial.printf("Ethernet    : %s | IP: %s\n", ethStatus.c_str(), ethIP.c_str());
      Serial.printf("WiFi (Bg)   : %s | IP: %s | RSSI: %d\n", wifiStatus.c_str(), wifiIP.c_str(), WiFi.RSSI());
      Serial.printf("Free Heap   : %d bytes\n", ESP.getFreeHeap());
      Serial.println("---------------------------------------------------\n");

      lastStatusPrint = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// CORE 1 TASK: Data Acquisition (Analog & Digital Input)
void Task_DataAcquisition(void *parameter)
{
  ESP_LOGI("Core1", "Data Acquisition Task started (Direct Mo257de)");

  SensorDataPacket sensorData;
  unsigned long lastReadAnalog = 0;
  unsigned long lastReadDigital = 0;
  unsigned long lastDebugPrint = 0;
  unsigned long lastRunTimeCheck = 0;

  // Variabel Digital (filter debouncing untuk tombol/digital)
  static int lastRawState[jumlahInputDigital + 1] = {0};
  static unsigned long lastDebounceTime[jumlahInputDigital + 1] = {0};
  static int stableState[jumlahInputDigital + 1] = {0};
  const unsigned long debounceDelay = 1;
  static float prevFilteredValues[jumlahInputAnalog + 1] = {0.0};

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)))
  {
    ads.setDataRate(RATE_ADS1015_3300SPS);
    // ads.setDataRate(RATE_ADS1115_8SPS);
    xSemaphoreGive(i2cMutex);
  }

  while (true)

  {
    bool useTCP = (networkSettings.protocolMode2.indexOf("TCP") >= 0);
    bool useRTU = (networkSettings.protocolMode2.indexOf("RTU") >= 0);

    // ========================================================================
    // A. BACA ANALOG
    // ========================================================================
    if (millis() - lastReadAnalog >= 100)
    {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)))
      {
        for (byte i = 1; i < jumlahInputAnalog + 1; i++)
        {
          // 1. OVERSAMPLING (Bawaan Kode Anda)
          long totalRaw = 0;
          int sampleCount = 10;
          for (int s = 0; s < sampleCount; s++)
          {
            totalRaw += ads.readADC_SingleEnded(i - 1);
          }
          int16_t avgAdc = totalRaw / sampleCount;
          float voltage = ads.computeVolts(avgAdc);

          // 2. HITUNG RAW X
          // Dengan GAIN_ONE (voltage/4.096), range: 15990-62912
          // Mapping ke standar ADS1115: 13107-65535 (1V-5V)
          float raw_x;

          // 1. Handle Deadband (Below 4mA / 0.4V)
          if (voltage <= 0.40)
          {
            raw_x = 13107.0;
          }
          else
          {
            // 2. Linear Interpolation
            // Voltage Range: 2.0 - 0.4 = 1.6V
            // Raw Range: 65535 - 13107 = 52428
            raw_x = ((voltage - 0.40) / 1.60) * 52428.0 + 13107.0;
          }

          // 3. Safety Clamp (Max 20mA / 2.0V)
          if (raw_x > 65535.0)
            raw_x = 65535.0;
          // ============================================================
          // [PERBAIKAN FITUR FILTER PERIODE]
          // ============================================================

          // Cek apakah filter diaktifkan di konfigurasi Analog Input?
          if (analogInput[i].filter == 1)
          {

            // Ambil nilai "Filter Periode" dari config
            // Di rumus filterSensor, ini berfungsi sebagai Cutoff Frequency (Hz)
            float fc = analogInput[i].filterPeriod;

            // Safety: Mencegah error pembagian nol
            if (fc < 0.01)
              fc = 0.1;

            // Panggil fungsi filterSensor yang ada di bawah file main.cpp
            // Input: (NilaiSekarang, NilaiLama, Frekuensi)
            raw_x = filterSensor(raw_x, prevFilteredValues[i], fc);

            // Simpan hasil filter ke memory untuk loop berikutnya
            prevFilteredValues[i] = raw_x;
          }
          else
          {
            // Jika filter mati, reset memory agar sinkron
            prevFilteredValues[i] = raw_x;
          }
          // ============================================================

          // Simpan nilai yang SUDAH TERSARING ke struct
          analogInput[i].adcValue = raw_x;

          // 3. KALKULASI MAP (y = mx + c)
          // Bagian ini akan menggunakan nilai raw_x yang sudah halus
          float finalResult = 0.0;
          float m = analogInput[i].mValue;
          float c = analogInput[i].cValue;

          // Deteksi Kabel Putus (< 0.8V)
          if (analogInput[i].inputType == "4-20 mA")
          {
            if (voltage < 0.2)
            {
              finalResult = analogInput[i].lowLimit;
              analogInput[i].adcValue = 0;
            }
            else
            {
              finalResult = (m * raw_x) + c;
            }
          }
          else
          {
            finalResult = (m * raw_x) + c;
          }

          analogInput[i].mapValue = finalResult;

          // Update Modbus Analog
          if (useTCP)
          {
            mbIP.Ireg(i + 9, (uint16_t)analogInput[i].adcValue);
            mbIP.Ireg(i - 1, (uint16_t)(analogInput[i].mapValue * 100));
          }
          if (useRTU)
          {
            mbRTU.Ireg(i + 9, (uint16_t)analogInput[i].adcValue);
            mbRTU.Ireg(i - 1, (uint16_t)(analogInput[i].mapValue * 100));
          }
          sensorData.analogValues[i] = analogInput[i].mapValue;
        }
        xSemaphoreGive(i2cMutex);
      }
      lastReadAnalog = millis();
    }

    // ========================================================================
    // B. BACA DIGITAL
    // ========================================================================
    if (millis() - lastReadDigital >= 5)
    {
      for (byte i = 1; i < jumlahInputDigital + 1; i++)
      {
        int currentRawReading = digitalRead(digitalInput[i].pin);
        if (currentRawReading != lastRawState[i])
        {
          lastDebounceTime[i] = millis(); // Reset timer
        }
        lastRawState[i] = currentRawReading; // Simpan status raw terakhir

        int cleanInput = stableState[i];

        if ((millis() - lastDebounceTime[i]) > debounceDelay)
        {
          if (currentRawReading != stableState[i])
          {
            stableState[i] = currentRawReading;
            cleanInput = stableState[i];
          }
        }

        static int prevStableState[jumlahInputDigital + 1] = {0};
        bool isRisingEdge = (cleanInput == 1 && prevStableState[i] == 0);
        bool isFallingEdge = (cleanInput == 0 && prevStableState[i] == 1);

        prevStableState[i] = cleanInput;

        // Cek Trigger untuk Data Logger
        int pinTrig = 0;
        if (networkSettings.sendTrig.length() >= 3)
          pinTrig = (networkSettings.sendTrig.substring(2, 3)).toInt();
        if (i == pinTrig && isRisingEdge)
          flagSend = true;

        if (digitalInput[i].taskMode == "Cycle Time")
        {
          if (digitalInput[i].flagInt)
          {
            digitalInput[i].value = (digitalInput[i].millisNow - digitalInput[i].millis_1) / 1000.0;
            digitalInput[i].millis_1 = digitalInput[i].millisNow;
            digitalInput[i].flagInt = 0;
          }
        }
        else if (digitalInput[i].taskMode == "Counting")
        {
          if (isRisingEdge)
          {
            digitalInput[i].value++;
          }
        }
        else if (digitalInput[i].taskMode == "Run Time")
        {
          if (millis() - lastRunTimeCheck >= 60000)
          {
            if (cleanInput == digitalInput[i].inputState)
            {
              digitalInput[i].value++;
              updateJson("/runtimeData.json", String(i).c_str(), digitalInput[i].value);
            }
          }
        }
        else if (digitalInput[i].taskMode == "Pulse Mode")
        {
          if (millis() - digitalInput[i].lastMillisPulseMode > digitalInput[i].intervalTime)
          {
            digitalInput[i].value = (float)digitalInput[i].sumValue * digitalInput[i].conversionFactor;
            digitalInput[i].sumValue = 0;
            digitalInput[i].lastMillisPulseMode = millis();
          }
        }
        else
        {
          int cleanInput = stableState[i];
          digitalInput[i].value = digitalInput[i].inv ? !cleanInput : cleanInput;
        }

        sensorData.digitalValues[i] = digitalInput[i].value;

        if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(10)))
        {
          if (digitalInput[i].name != "")
          {
            jsonSend[digitalInput[i].name] = digitalInput[i].value;
            jsonSend[digitalInput[i].name + "_mode"] = digitalInput[i].taskMode;
          }
          xSemaphoreGive(jsonMutex);
        }

        if (useTCP)
          mbIP.Ireg(i + 19, digitalInput[i].value);
        if (useRTU)
          mbRTU.Ireg(i + 19, digitalInput[i].value);
      }

      if (millis() - lastRunTimeCheck >= 60000)
      {
        lastRunTimeCheck = millis();
      }

      lastReadDigital = millis();
    }
    lastReadDigital = millis();
    sensorData.timestamp = millis();
    xQueueSend(queueSensorData, &sensorData, 0);

    // ========================================================================
    // C. DEBUG MONITOR
    // ========================================================================
    if (millis() - lastDebugPrint >= 2000)
    {
      // --- ANALOG MONITOR ---
      Serial.println("\n=== ANALOG INPUT MONITOR ===");
      // Perlebar kolom Name jadi 20 karakter
      Serial.println("ID | Name                 | Value    | Raw   | Type");
      for (int i = 1; i <= jumlahInputAnalog; i++)
      {
        // Potong nama jika lebih dari 20 karakter agar tabel tidak jebol
        String dispName = analogInput[i].name;
        if (dispName.length() > 20)
          dispName = dispName.substring(0, 20);

        Serial.printf("A%-2d| %-20s | %8.2f | %5.0f | %s\n", i,
                      dispName.c_str(), // Gunakan nama yang sudah dipotong
                      analogInput[i].mapValue,
                      analogInput[i].adcValue,
                      analogInput[i].inputType.c_str());
      }

      // --- DIGITAL MONITOR ---
      Serial.println("\n=== DIGITAL INPUT MONITOR ===");
      // Perlebar kolom Name jadi 20 karakter
      Serial.println("ID | Name                 | Value    | Mode         | Status");
      Serial.println("---|----------------------|----------|--------------|--------");

      for (int i = 1; i <= jumlahInputDigital; i++)
      {
        String statusStr;
        if (digitalInput[i].taskMode == "Counting")
          statusStr = String((int)digitalInput[i].value) + " cnt";
        else if (digitalInput[i].taskMode == "Run Time")
          statusStr = String((int)digitalInput[i].value) + " min";
        else if (digitalInput[i].taskMode == "Cycle Time")
          statusStr = String(digitalInput[i].value, 2) + " s";
        else if (digitalInput[i].taskMode == "Pulse Mode")
          statusStr = String(digitalInput[i].value, 2);
        else
          statusStr = (digitalInput[i].value > 0.5) ? "HIGH" : "LOW";

        // Potong nama jika lebih dari 20 karakter
        String dispName = digitalInput[i].name;
        if (dispName.length() > 20)
          dispName = dispName.substring(0, 20);

        // Ubah %-15s menjadi %-20s
        Serial.printf("D%-2d| %-20s | %-8.2f | %-12s | %s\n", i,
                      dispName.c_str(),
                      digitalInput[i].value,
                      (digitalInput[i].taskMode.length() > 0 ? digitalInput[i].taskMode.c_str() : "Normal"),
                      statusStr.c_str());
      }
      lastDebugPrint = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================================
// CORE 1 TASK: Modbus Client (Master) - DEBUG VERSION
// ============================================================================
void Task_ModbusClient(void *parameter)
{
  ESP_LOGI("Core1", "Modbus Client Task started");
  esp_task_wdt_add(NULL);
  unsigned long lastModbusRead = 0;
  unsigned long lastWatchdogFeed = 0;

  while (true)
  {
    // Reset Watchdog
    if (millis() - lastWatchdogFeed >= 5000)
    {
      esp_task_wdt_reset();
      lastWatchdogFeed = millis();
    }

    // Cek Timer Scan Rate
    if (millis() - lastModbusRead >= (modbusParam.scanRate * 1000))
    {
      int totalParamsToRead = 0;

      // 1. Ambil Jumlah Sensor
      if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(500)))
      {
        if (!jsonParam.containsKey("nameData") && stringParam.length() > 5)
          deserializeJson(jsonParam, stringParam);
        JsonArray nameData = jsonParam["nameData"];
        totalParamsToRead = nameData.isNull() ? 0 : nameData.size();
        xSemaphoreGive(jsonMutex);
      }

      // 2. CETAK HEADER TABEL (Hanya jika ada sensor)
      if (totalParamsToRead > 0)
      {
        Serial.println("\n=== MODBUS DATA MONITOR ===");
        // Format Header mirip Analog/Digital Monitor
        Serial.println("ID | Name            | Value    | Raw   | Addr:Reg  | Status");
      }

      // 3. LOOPING SENSOR & CETAK BARIS
      for (int i = 0; i < totalParamsToRead; i++)
      {
        String currentParamName = "";
        unsigned int addr = 0, fc = 0, reg = 0;
        float multiplier = 1.0;
        bool validParam = false;
        bool readSuccess = false; // Flag status bacaan

        // A. Ambil Config Sensor ke-i
        if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
        {
          JsonArray nameData = jsonParam["nameData"];
          if (i < nameData.size())
          {
            currentParamName = nameData[i].as<String>();
            if (jsonParam.containsKey(currentParamName))
            {
              JsonArray paramArray = jsonParam[currentParamName];
              addr = paramArray[0];
              fc = paramArray[1];
              reg = paramArray[2];
              multiplier = paramArray[3];
              validParam = true;
            }
          }
          xSemaphoreGive(jsonMutex);
        }

        // B. Eksekusi Baca & Cetak Baris
        if (validParam)
        {
          // Panggil fungsi baru dengan parameter '&readSuccess'
          unsigned int rawValue = readModbusNonBlocking(addr, fc, reg, 250, readSuccess);
          float finalValue = rawValue * multiplier;

          // Potong nama jika terlalu panjang agar tabel rapi
          String dispName = currentParamName;
          if (dispName.length() > 15)
            dispName = dispName.substring(0, 15);

          Serial.printf("M%-2d| %-15s | %8.2f | %-5u | %02d:%-5d | %s\n",
                        i + 1,
                        dispName.c_str(),
                        readSuccess ? finalValue : 0.00, // Tampilkan 0 jika gagal
                        rawValue,
                        addr, reg,
                        readSuccess ? "OK" : "TIMEOUT"); // Status Text

          // Update ke JSON Send untuk Web/MQTT
          if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(100)))
          {
            if (currentParamName.length() > 0)
            {
              // Jika timeout, opsional: jangan update atau update 0
              if (readSuccess)
                jsonSend[currentParamName] = String(finalValue, 2);
              else if (!jsonSend.containsKey(currentParamName))
                jsonSend[currentParamName] = String(0.0f, 2); // inisialisasi default
            }
            xSemaphoreGive(jsonMutex);
          }
        }

        // Jeda antar sensor agar stabil
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      lastModbusRead = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================================
// CORE 1 TASK: Data Logger & HTTP Sender (VERSI FINAL - ANTI CRASH)
// ============================================================================
void Task_DataLogger(void *parameter)
{
  ESP_LOGI("Core1", "Data Logger Task started");
  esp_task_wdt_add(NULL);
  SensorDataPacket sensorData;
  ModbusDataPacket modbusData;
  unsigned long lastSendTime = 0;
  unsigned long lastSDSave = 0;
  unsigned long lastPrint = 0;
  unsigned long lastWatchdogFeed = 0;
  DynamicJsonDocument docNew(2048);
  DynamicJsonDocument docSD(1024);

  while (true)
  {
    if (millis() - lastWatchdogFeed >= 5000)
    {
      esp_task_wdt_reset();
      lastWatchdogFeed = millis();
    }

    // 1. UPDATE DATA JSON
    if (xQueueReceive(queueSensorData, &sensorData, 0) == pdTRUE)
    {
      if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
      {
        for (byte i = 1; i < jumlahInputAnalog + 1; i++)
        {
          if (analogInput[i].name != "")
          {
            jsonSend[analogInput[i].name] = String(sensorData.analogValues[i], 2);
          }
        }
        for (byte i = 1; i < jumlahInputDigital + 1; i++)
        {
          if (digitalInput[i].name != "")
          {
            jsonSend[digitalInput[i].name] = sensorData.digitalValues[i];
          }
        }
        xSemaphoreGive(jsonMutex);
      }
    }

    if (xQueueReceive(queueModbusData, &modbusData, 0) == pdTRUE)
    {
      if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(100)))
      {
        jsonSend[modbusData.paramName] = String(modbusData.value, 2);
        xSemaphoreGive(jsonMutex);
      }
    }

    // 2. PERIODIC DATA SENDING
    // 2. PERIODIC DATA SENDING + SD SAVE (atomik)
    if ((millis() - lastSendTime >= (unsigned long)(networkSettings.sendInterval * 1000)) || flagSend)
    {
      flagSend = false;
      lastSendTime = millis();

      // Siapkan data SD sekali, dipakai untuk send dan save
      String dataToSave = "";
      if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
      {
        docSD.clear();
        JsonArray arraySD = docSD.to<JsonArray>();
        for (JsonPair kv : jsonSend.as<JsonObject>())
        {
          if (kv.key() == "-" || String(kv.key().c_str()).endsWith("_mode"))
            continue;
          JsonObject nestedObj = arraySD.createNestedObject();
          nestedObj["KodeSensor"] = kv.key().c_str();
          if (jobNum.length() > 4)
          {
            JsonObject additional = nestedObj.createNestedObject("additional");
            additional["jobnum"] = jobNum.c_str();
          }
          nestedObj["StringWaktu"] = getTimeDateNow();
          nestedObj["Value"] = String(kv.value().as<float>());
        }
        serializeJson(docSD, dataToSave);
        xSemaphoreGive(jsonMutex);
      }

      // SD Save - setiap kali send
      if (dataToSave.length() > 10)
      {
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500)))
        {
          if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(2000)))
          {
            saveToSD(dataToSave);
            xSemaphoreGive(spiMutex);
            lastSDSave = millis(); // reset timer fallback
          }
          else
            Serial.println("⚠️ SD Save Skipped (SPI Busy)");
          xSemaphoreGive(sdMutex);
        }
      }

      if (networkSettings.loggerMode != "Disabled")
      {
        Serial.printf("Target   : %s\n", networkSettings.endpoint.c_str());

        if (networkSettings.protocolMode == "HTTP")
        {
          struct SensorItem
          {
            String key;
            float value;
          };
          SensorItem dataList[30];
          int dataCount = 0;

          if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
          {
            for (JsonPair kv : jsonSend.as<JsonObject>())
            {
              String key = kv.key().c_str();
              if (key == "-" || key == "" || key.endsWith("_mode"))
                continue;
              if (dataCount < 30)
              {
                dataList[dataCount].key = key;
                dataList[dataCount].value = kv.value().as<String>().toFloat();
                dataCount++;
              }
            }
            xSemaphoreGive(jsonMutex);
          }

          for (int i = 0; i < dataCount; i++)
          {
            docNew.clear();
            docNew["kodeSensor"] = dataList[i].key;
            char valBuf[20];
            dtostrf(dataList[i].value, 1, 2, valBuf);
            docNew["value"] = serialized(String(valBuf));

            sendString = "";
            serializeJson(docNew, sendString);

            // Send data to HTTP queue instead of blocking the task
            HttpSendPacket pkt;
            strlcpy(pkt.data, sendString.c_str(), sizeof(pkt.data));
            strlcpy(pkt.url, networkSettings.endpoint.c_str(), sizeof(pkt.url));
            strlcpy(pkt.username, networkSettings.mqttUsername.c_str(), sizeof(pkt.username));
            strlcpy(pkt.password, networkSettings.mqttPassword.c_str(), sizeof(pkt.password));
            xQueueSend(queueHttpSend, &pkt, portMAX_DELAY);

            vTaskDelay(pdMS_TO_TICKS(300));
          }
        }
        else if (networkSettings.protocolMode == "MQTT")
        {
          String mqttPayload = "";
          if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
          {
            serializeJson(jsonSend, mqttPayload);
            xSemaphoreGive(jsonMutex);
          }

          Serial.print("[SENDER] Sending via MQTT... ");
          if (mqtt.connected() && networkSettings.pubTopic.length() > 0)
          {
            if (mqtt.publish(networkSettings.pubTopic.c_str(), mqttPayload.c_str()))
            {
              Serial.println("✅ SUCCESS");
              networkSettings.connStatus = "Connected";
            }
            else
            {
              Serial.println("❌ FAIL");
            }
          }
          else
          {
            Serial.println("❌ FAIL (Not Connected / No Topic)");
            networkSettings.connStatus = "Not Connected";
          }
        }
      }
    }

    // 3. SD CARD SAVE FALLBACK (jika server tidak aktif / loggerMode Disabled)
    if (millis() - lastSDSave >= (networkSettings.sdSaveInterval * 60000UL))
    {
      String dataToSave = "";
      if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
      {
        docSD.clear();
        JsonArray arraySD = docSD.to<JsonArray>();
        for (JsonPair kv : jsonSend.as<JsonObject>())
        {
          if (kv.key() == "-")
            continue;
          JsonObject nestedObj = arraySD.createNestedObject();
          nestedObj["KodeSensor"] = kv.key().c_str();
          if (jobNum.length() > 4)
          {
            JsonObject additional = nestedObj.createNestedObject("additional");
            additional["jobnum"] = jobNum.c_str();
          }
          nestedObj["StringWaktu"] = getTimeDateNow();
          nestedObj["Value"] = String(kv.value().as<float>());
        }
        serializeJson(docSD, dataToSave);
        xSemaphoreGive(jsonMutex);
      }

      if (dataToSave.length() > 10)
      {
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500)))
        {
          if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(2000)))
          {
            saveToSD(dataToSave);
            xSemaphoreGive(spiMutex);
            ESP_LOGI("SD", "Data saved safely");
          }
          else
          {
            Serial.println("⚠️ SD Save Skipped (SPI Busy)");
          }
          xSemaphoreGive(sdMutex);
        }
      }
      lastSDSave = millis();
    }

    // 4. BACKUP SEND
    if (millis() - lastPrint >= 10000)
    {
      if (networkSettings.connStatus == "Connected")
      {
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500)))
        {
          if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(3000)))
          {
            sendBackupData();
            xSemaphoreGive(spiMutex);
          }
          else
          {
            Serial.println("⚠️ Backup Send Skipped (SPI Busy)");
          }
          xSemaphoreGive(sdMutex);
        }
      }
      lastPrint = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void taskHTTPSend(void *pvParameters)
{
  HttpSendPacket pkt;
  esp_task_wdt_add(NULL);
  for (;;)
  {
    if (xQueueReceive(queueHttpSend, &pkt, portMAX_DELAY) == pdTRUE)
    {
      if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(2000)))
      {
        sendDataHTTP(String(pkt.data), String(pkt.url), String(pkt.username), String(pkt.password), 0);
        xSemaphoreGive(spiMutex);
      }
      else
      {
        Serial.println("❌ SKIP HTTP Task (SPI Busy)");
      }
    }
    esp_task_wdt_reset();
  }
}

void calculateAnalogCalibration(int id)
{
  float lowVal = analogInput[id].lowLimit;   // Contoh: 0
  float highVal = analogInput[id].highLimit; // Contoh: 100
  float x1, x2;

  // Tentukan Titik X (Raw) berdasarkan tipe input
  if (analogInput[id].inputType == "4-20 mA")
  {
    x1 = 13107.0;
    x2 = 65535.0;
  }
  else
  {
    // Default 0-5V
    x1 = 0.0;
    x2 = 65535.0;
  }

  // 1. Hitung Slope (m)
  if ((x2 - x1) != 0)
  {
    analogInput[id].mValue = (highVal - lowVal) / (x2 - x1);
  }
  else
  {
    analogInput[id].mValue = 0;
  }

  // 2. Hitung Intercept (c)
  analogInput[id].cValue = lowVal - (analogInput[id].mValue * x1);

  // Debugging
  Serial.printf("[CALIB ID:%d] Range:%.0f-%.0f | Raw Calib:%.0f-%.0f | m:%.8f c:%.4f\n",
                id, lowVal, highVal, x1, x2, analogInput[id].mValue, analogInput[id].cValue);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("SYSTEM BOOT START");
  sysMonitor.begin();
  sysMonitor.printCurrent();
  esp_task_wdt_init(60, true);
  Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println("[Init] Configuring Watchdog Timer...");
  esp_task_wdt_init(60, true); // 30 detik timeout
  Serial.println("  ✓ Watchdog: 60 seconds");
  // Set CPU frequency
  setCpuFrequencyMhz(240);

  // Reset Pin Ethernet Hardware
  pinMode(ETH_RST, OUTPUT);
  digitalWrite(ETH_RST, HIGH);

  // Setup digital input pins
  for (byte i = 1; i < jumlahInputDigital + 1; i++)
  {
    digitalInput[i].pin = DI_PINS[i - 1];
    pinMode(digitalInput[i].pin, INPUT_PULLDOWN);
  }

  // 1. CREATE MUTEXES & QUEUES
  spiMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  sdMutex = xSemaphoreCreateMutex();
  jsonMutex = xSemaphoreCreateMutex();
  modbusMutex = xSemaphoreCreateMutex();
  queueSensorData = xQueueCreate(10, sizeof(SensorDataPacket));
  queueModbusData = xQueueCreate(10, sizeof(ModbusDataPacket));
  queueLogData = xQueueCreate(10, sizeof(LogDataPacket));
  queueHttpSend = xQueueCreate(10, sizeof(HttpSendPacket));

  if (!spiMutex || !jsonMutex || !queueSensorData || !modbusMutex || !queueHttpSend)
  {
    Serial.println("❌ Critical Error: Failed to create Mutex/Queue!");
    while (1)
      delay(1000);
  }

  // 2. READ CONFIG & INIT BASIC HARDWARE
  // Read configuration (SPIFFS)
  readConfig();
  // Force Ethernet mode (sesuai request Anda)
  networkSettings.networkMode = "Ethernet";
  Serial.println("[FORCE] Mode set to Ethernet");
  printConfigurationDetails();

  modbusSlaveSetup();
  setupInterrupts();

  // Initialize I2C
  Wire.begin();

  // Initialize Time & RTC
  configTime(7 * 3600, 0, "pool.ntp.org");
  if (!rtc.begin())
  {
    Serial.println("❌ RTC Not Found");
    errorMessages.addMessage("RTC Failed");
  }
  else
  {
    Serial.println("✅ RTC Initialized");
    // Sync time logic here if needed...
  }

  // ========================================================================
  // Ini menggantikan blok "SD CARD" yang lama.
  // Tujuannya: Menyalakan SD Card dan Ethernet dengan urutan yang benar (Anti-Bentrok)

  Serial.println("[Init] Configuring SPI Hardware...");

  // A. Matikan (Deselect) kedua chip dulu agar tidak tabrakan di awal
  pinMode(ETH_CS, OUTPUT);
  digitalWrite(ETH_CS, HIGH); // Matikan Ethernet (CS HIGH = OFF)

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH); // Matikan SD Card (CS HIGH = OFF)

  delay(50); // Beri waktu jeda

  // B. Mulai Jalur SPI (Shared Bus)
  // Kita set pin manual agar yakin W5500 dan SD Card pakai jalur yg sama
  // Parameter terakhir -1 artinya "tidak ada CS default", karena kita atur manual
  SPI.begin(ETH_CLK, ETH_MISO, ETH_MOSI, -1);

  // C. Nyalakan SD Card Duluan (Sekali Saja!)
  Serial.print("[Init] Mounting SD Card... ");

  // Gunakan Mutex untuk keamanan
  if (xSemaphoreTake(spiMutex, portMAX_DELAY))
  {
    // Coba mount SD Card
    // Speed 4MHz lebih stabil untuk kabel jumper/panjang
    if (!SD.begin(SD_CS_PIN, SPI, 4000000))
    {
      Serial.println("❌ SD Card Mount Failed!");
      Serial.println("   -> Cek kabel atau format kartu (FAT32)");
      errorMessages.addMessage("SD Init Failed");
    }
    else
    {
      Serial.println("✅ SD Card Mounted Successfully!");
      Serial.print("   -> Size: ");
      Serial.print(SD.totalBytes() / (1024 * 1024));
      Serial.println(" MB");
    }
    xSemaphoreGive(spiMutex);
  }

  // Catatan: Ethernet W5500 belum dinyalakan disini.
  // Ethernet akan dinyalakan nanti di dalam fungsi configNetwork().

  // ========================================================================
  // 4. INIT SENSORS & NETWORK
  // ========================================================================

  // Initialize ADS1115
  if (!ads.begin())
  {
    Serial.println("❌ ADS1115 Failed");
    errorMessages.addMessage("ADS1115 Failed");
  }
  else
  {
    Serial.println("✅ ADS1115 Initialized");
    // int myGainConfig = 10;
    // ads.setGain((adsGain_t)myGainConfig);
    ads.setGain(GAIN_TWO); // +/-6.144V
  }

  printTime = millis();
  sendTimeModbus = millis();
  modbusCount = 0;
  jsonSend = DynamicJsonDocument(4096);
  ESP_LOGI("Network", "Configuring network interface...");
  configNetwork();
  configProtocol();
  delay(1000);
  ESP_LOGI("WebServer", "Starting web server...");
  setupWebServer();

  if (networkSettings.networkMode == "Ethernet")
  {
    ethServer.begin();
    Serial.println("✅ Ethernet Web Server STARTED");
  }

  // Print Access Info
  Serial.println("\n=================================");
  Serial.println("SYSTEM READY");
  if (networkSettings.networkMode == "Ethernet")
  {
    Serial.printf("Ethernet IP: %s\n", Ethernet.localIP().toString().c_str());
  }
  Serial.printf("WiFi AP IP:  %s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("=================================\n");

  // ========================================================================
  // 5. CREATE TASKS
  // ========================================================================
  xTaskCreatePinnedToCore(Task_NetworkManagement, "NetworkTask", 10240, NULL, 3, &Task_Core0_Network, 0);
  xTaskCreatePinnedToCore(Task_DataAcquisition, "DataAcqTask", 8192, NULL, 4, &Task_Core1_DataAcquisition, 1);
  xTaskCreatePinnedToCore(Task_ModbusClient, "ModbusTask", 8192, NULL, 3, &Task_Core1_ModbusClient, 1);
  xTaskCreatePinnedToCore(Task_DataLogger, "LoggerTask", 32768, NULL, 1, &Task_Core1_DataLogger, 0);
  xTaskCreatePinnedToCore(taskHTTPSend, "HTTPSendTask", 8192, NULL, 2, &Task_Core0_HTTPSend, 0);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// SETUP WEB SERVER
void setupWebServer()
{
  // Handle OTA update
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request)
            {
      bool shouldReboot = !Update.hasError();
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", 
        shouldReboot ? "Update Successful. Rebooting..." : "Update Failed. Check error logs.");
      response->addHeader("Connection", "close");
      request->send(response);
      
      if (shouldReboot) {
        ESP_LOGI("OTA", "Update successful, rebooting...");
        delay(2000);
  
      } else {
        ESP_LOGE("OTA", "Update failed");
        errorMessages.addMessage(getTimeNow() + " - OTA Update Failed");
      } }, [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final)
            {
      if (!index) {
        ESP_LOGI("OTA", "Update Start: %s", filename.c_str());
        Serial.printf("Update Start: %s\n", filename.c_str());
        
        int updateType = U_FLASH;
        if (filename.indexOf("spiffs") > -1) {
          updateType = U_SPIFFS;
          ESP_LOGI("OTA", "Update Type: SPIFFS");
        } else {
          ESP_LOGI("OTA", "Update Type: Firmware");
        }
        
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, updateType)) {
          Update.printError(Serial);
          ESP_LOGE("OTA", "Update.begin failed");
          errorMessages.addMessage(getTimeNow() + " - OTA Update.begin failed");
        } else {
          ESP_LOGI("OTA", "Update started successfully");
        }
      }
      
      if (Update.write(data, len) != len) {
        Update.printError(Serial);
        ESP_LOGE("OTA", "Update.write failed");
      } else {
        static unsigned long lastProgress = 0;
        if (millis() - lastProgress > 1000) {
          ESP_LOGI("OTA", "Progress: %d bytes written", index + len);
          lastProgress = millis();
        }
      }
      
      if (final) {
        if (Update.end(true)) {
          ESP_LOGI("OTA", "Update Success: %uB written", index + len);
          Serial.printf("Update Success: %uB\n", index + len);
        } else {
          Update.printError(Serial);
          ESP_LOGE("OTA", "Update.end failed");
          errorMessages.addMessage(getTimeNow() + " - OTA Update.end failed");
        }
      } });

  // OTA status endpoint
  server.on("/updateStatus", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    DynamicJsonDocument statusDoc(256);
    
    if (Update.hasError()) {
      statusDoc["status"] = "error";
      statusDoc["error"] = Update.errorString();
    } else if (Update.isRunning()) {
      statusDoc["status"] = "updating";
      statusDoc["progress"] = Update.progress();
      statusDoc["size"] = Update.size();
    } else {
      statusDoc["status"] = "idle";
    }
    
    statusDoc["freeHeap"] = ESP.getFreeHeap();
    statusDoc["sketchSize"] = ESP.getSketchSize();
    statusDoc["freeSketchSpace"] = ESP.getFreeSketchSpace();
    
    String response;
    serializeJson(statusDoc, response);
    request->send(200, "application/json", response); });

  // Web server routes
  // [3] Root handler untuk captive portal
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/home.html", "text/html"); });
  // [4] Redirect semua request ke root
  server.onNotFound([](AsyncWebServerRequest *request)
                    { request->redirect("/"); });

  server.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request)
            { handleFileRequest(request, "/debug-monitor.html", "text/html"); });

  server.addHandler(&errorMessages._eventSource);

  server.on("/system_settings", HTTP_GET, [](AsyncWebServerRequest *request)
            { handleFileRequest(request, "/system_settings.html", "text/html"); });
  server.on("/system_settings", HTTP_POST, [](AsyncWebServerRequest *request)
            { handleFormSubmit(request); });

  server.on("/settingsLoad", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String jsonAuth = "{";
    jsonAuth += "\"username\":\"" + networkSettings.loginUsername + "\",";
    jsonAuth += "\"password\":\"" + networkSettings.loginPassword + "\",";
    jsonAuth += "\"sdInterval\":" + String(networkSettings.sdSaveInterval);
    jsonAuth += "}";
    request->send(200, "application/json", jsonAuth); });

  server.on("/getTime", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              DynamicJsonDocument jsonDoc(256);
              jsonDoc["datetime"] = getTimeDateNow();
              String jsonTime;
              serializeJson(jsonDoc, jsonTime);
              request->send(200, "application/json", jsonTime); });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { authenthicateUser(request);
              handleFileRequest(request, "/home.html", "text/html"); });
  server.on("/updateOTA", HTTP_GET, [](AsyncWebServerRequest *request)
            { authenthicateUser(request);
              handleFileRequest(request, "/UpdateOTA.html", "text/html"); });
  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
            { handleFormSubmit(request); });
  server.on("/network", HTTP_GET, [](AsyncWebServerRequest *request)
            { authenthicateUser(request);
              handleFileRequest(request, "/network.html", "text/html"); });

  server.on("/networkLoad", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    if (request->hasArg("restart")){
      request->send(200, "text/plain", "OKKK");
      ESP_LOGI("ESP", "ESP will restart");
      delay(2000);

    }
    doc = DynamicJsonDocument(4096);
    doc["networkMode"] = networkSettings.networkMode;
    doc["ssid"] = networkSettings.ssid;
    doc["password"] = networkSettings.password;
    doc["apSsid"] = networkSettings.apSsid;
    doc["apPassword"] = networkSettings.apPassword;
    doc["dhcpMode"] = networkSettings.dhcpMode;
    doc["ipAddress"] = networkSettings.ipAddress;
    doc["subnet"] = networkSettings.subnetMask;
    doc["ipGateway"] = networkSettings.ipGateway;
    doc["ipDNS"] = networkSettings.ipDNS;
    doc["sendInterval"] = String(networkSettings.sendInterval,2);
    doc["protocolMode"] = networkSettings.protocolMode;
    doc["endpoint"] = networkSettings.endpoint;
    doc["port"] = networkSettings.port;
    doc["pubTopic"] = networkSettings.pubTopic;
    doc["subTopic"] = networkSettings.subTopic;
    doc["mqttUsername"] = networkSettings.mqttUsername;
    doc["mqttPass"] = networkSettings.mqttPassword;
    doc["loggerMode"] = networkSettings.loggerMode;
    doc["protocolMode2"] = networkSettings.protocolMode2;
    doc["modbusMode"] = modbusParam.mode;
    doc["modbusPort"] = modbusParam.port;
    doc["modbusSlaveID"] = modbusParam.slaveID;
    doc["sendTrig"] = networkSettings.sendTrig;
    doc["erpUrl"] = networkSettings.erpUrl;
    doc["erpUsername"] = networkSettings.erpUsername;
    doc["erpPassword"] = networkSettings.erpPassword;

    String dataLoad;
    serializeJson(doc, dataLoad);
    request->send(200, "application/json", dataLoad); });

  server.on("/network", HTTP_POST, [](AsyncWebServerRequest *request)
            { handleFormSubmit(request); });

  server.on("/analog_input", HTTP_GET, [](AsyncWebServerRequest *request)
            { authenthicateUser(request);
              handleFileRequest(request, "/analog_input.html", "text/html"); });
  server.on("/analog_input", HTTP_POST, [](AsyncWebServerRequest *request)
            { handleFormSubmit(request); });

  server.on("/analogLoad", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              doc = DynamicJsonDocument(4096);
              if (request->hasArg("input"))
              {
                Serial.println(request->arg("input"));
                unsigned char id = request->arg("input").toInt();
                doc["inputType"] = analogInput[id].inputType;
                doc["filter"] = analogInput[id].filter;
                doc["filterPeriod"] = (String(analogInput[id].filterPeriod, 2));
                doc["scaling"] = analogInput[id].scaling;
                doc["lowLimit"] = String(analogInput[id].lowLimit, 2); 
                doc["highLimit"] = String(analogInput[id].highLimit, 2);
                doc["name"] = analogInput[id].name;
                doc["calibration"] = analogInput[id].calibration;
                doc["mValue"] = String(analogInput[id].mValue, 6);
                doc["cValue"] = String(analogInput[id].cValue, 4);
              }
              String dataLoad;
              serializeJson(doc, dataLoad);
              request->send(200, "application/json", dataLoad); });

  server.on("/digital_IO", HTTP_GET, [](AsyncWebServerRequest *request)
            { authenthicateUser(request);
              handleFileRequest(request, "/digital_IO.html", "text/html"); });
  server.on("/digital_IO", HTTP_POST, [](AsyncWebServerRequest *request)
            { handleFormSubmit(request); });

  server.on("/digitalLoad", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    doc = DynamicJsonDocument(4096);
    if(request->hasArg("input")){
      Serial.println(request->arg("input"));
      unsigned char id = request->arg("input").toInt();
      doc["nameDI"] = digitalInput[id].name;
      doc["invDI"] = digitalInput[id].inv;
      doc["taskMode"] = digitalInput[id].taskMode;
      doc["inputState"] = digitalInput[id].inputState ? "High" : "Low";
      doc["intervalTime"] = (float)digitalInput[id].intervalTime/1000;
      doc["conversionFactor"] = digitalInput[id].conversionFactor;
    }
    if(request->hasArg("reset")){
      unsigned char id = request->arg("reset").toInt();
      digitalInput[id].value = 0;
    }
    if(request->hasArg("output")){
      Serial.println(request->arg("output"));
      unsigned char id = request->arg("output").toInt();
      doc["nameDO"] = digitalOutput[id].name;
      doc["invDO"] = digitalOutput[id].inv;
    }
    String dataLoad;
    serializeJson(doc, dataLoad);
    Serial.println(dataLoad);
    request->send(200, "application/json", dataLoad); });

  server.on("/modbus_setup", HTTP_GET, [](AsyncWebServerRequest *request)
            { authenthicateUser(request);
              handleFileRequest(request, "/modbus_setup.html", "text/html"); });

  AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/modbus_setup", [](AsyncWebServerRequest *request, JsonVariant &json)
                                                                         {
    Serial.println("Masuk JSON");
    jsonParam = DynamicJsonDocument(1024);
    if (json.is<JsonArray>())
    {
      jsonParam = json.as<JsonArray>();
    }
    else if (json.is<JsonObject>())
    {
      jsonParam = json.as<JsonObject>();
    }
    stringParam = "";
    serializeJson(jsonParam, stringParam);
    // jsonSend = DynamicJsonDocument(1024);
    request->send(200, "text/plain", "Succesfull");
    saveToJson("/modbusSetup.json","modbusSetup");
    Serial.println(stringParam); });
  server.addHandler(handler);

  server.on("/modbusLoad", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "application/json", stringParam); });

  server.on("/getValue", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      String realtimeJson;
      // Gunakan Mutex agar tidak crash saat membaca data live
      if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(100))) 
      {
          String realtimeJson = "[";
          bool first = true;
          JsonObject root = jsonSend.as<JsonObject>();
          for (JsonPair kv : root) {
             String key = kv.key().c_str();
             if (key == "-" || key == "") continue;
             
             if (!first) realtimeJson += ",";
             realtimeJson += "{\"KodeSensor\":\"" + key + "\",\"Value\":\"" + kv.value().as<String>() + "\"}";
             first = false;
          }
          realtimeJson += "]";
          xSemaphoreGive(jsonMutex);
          request->send(200, "application/json", realtimeJson);
      } 
      else 
      {
          request->send(200, "application/json", "[]");
      } });
  server.on("/getCurrentValue", HTTP_GET, [](AsyncWebServerRequest *request)
            {
      if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(100))) 
      {
          DynamicJsonDocument docTemp(2048); // Gunakan doc lokal agar aman
          
          // Fix: Loop dari 1 sampai jumlahInputAnalog (Bukan dari 0)
          for (int i = 1; i <= jumlahInputAnalog; i++) {
            docTemp["rawValue" + String(i)] = analogInput[i].adcValue;
            docTemp["scaledValue" + String(i)] = analogInput[i].mapValue;
          }
          
          // Tambahkan Digital Input
          for (int i = 1; i <= jumlahInputDigital; i++) {
             docTemp["diValue" + String(i)] = digitalInput[i].value;
          }

          String dataLoad;
          serializeJson(docTemp, dataLoad);
          xSemaphoreGive(jsonMutex);
          request->send(200, "application/json", dataLoad); 
      }
      else
      {
          request->send(200, "application/json", "{}");
      } });

  server.on("/homeLoad", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              // Ambil Mutex agar data konsisten
              if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(100))) 
              {
                doc = DynamicJsonDocument(4096);
                
                // --- Data Network & System ---
                doc["networkMode"] = networkSettings.networkMode;
                doc["ssid"] = networkSettings.ssid;
                doc["ipAddress"] = (networkSettings.networkMode == "WiFi") ? WiFi.localIP().toString() : networkSettings.ipAddress;
                doc["macAddress"] = (networkSettings.networkMode == "WiFi") ? WiFi.macAddress() : networkSettings.macAddress;
                doc["protocolMode"] = networkSettings.protocolMode;
                doc["endpoint"] = networkSettings.endpoint;
                doc["connStatus"] = networkSettings.connStatus;
                doc["jobNumber"] = jobNum;
                doc["sendInterval"] = networkSettings.sendInterval;
                doc["datetime"] = getTimeDateNow();

                // --- Data Analog Input ---
                JsonObject AI = doc.createNestedObject("AI");
                JsonArray AIRawValue = AI.createNestedArray("rawValue");
                JsonArray AIScaledValue = AI.createNestedArray("scaledValue");
                
                // [FIX] BARIS INI WAJIB DIAKTIFKAN (Hapus tanda //)
                JsonArray EnAI = AI.createNestedArray("enAI"); 
                // ------------------------------------------------

                for (int i = 1; i <= jumlahInputAnalog; i++) {
                  AIRawValue.add(analogInput[i].adcValue);
                  AIScaledValue.add(analogInput[i].mapValue);
                  
                  // [FIX] BARIS INI WAJIB DIAKTIFKAN (Hapus tanda //)
                  // Jika nama sensor terisi, kirim 1 (aktif). Jika kosong, kirim 0.
                  EnAI.add(analogInput[i].name != "" ? 1 : 0);
                  // ------------------------------------------------
                }

                // --- Data Digital Input ---
                JsonObject DI = doc.createNestedObject("DI");
                JsonArray DIValue = DI.createNestedArray("value");
                JsonArray DITaskMode = DI.createNestedArray("taskMode");
                JsonArray DIName = DI.createNestedArray("name");

                for (int i = 1; i <= jumlahInputDigital; i++) {
                  DIValue.add(digitalInput[i].value);
                  DITaskMode.add(digitalInput[i].taskMode);
                  DIName.add(digitalInput[i].name);
                }

                String dataLoad;
                serializeJson(doc, dataLoad);
                xSemaphoreGive(jsonMutex);
                
                request->send(200, "application/json", dataLoad);
              }
              else 
              {
                request->send(503, "application/json", "{}");
              } });
  // WiFi Status and Diagnostic Endpoint
  server.on("/wifiStatus", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    DynamicJsonDocument statusDoc(512);
    
    statusDoc["mode"] = networkSettings.networkMode;
    statusDoc["ssid"] = networkSettings.ssid;
    statusDoc["connected"] = (WiFi.status() == WL_CONNECTED);
    statusDoc["ip"] = WiFi.localIP().toString();
    statusDoc["rssi"] = WiFi.RSSI();
    statusDoc["channel"] = WiFi.channel();
    statusDoc["macAddress"] = WiFi.macAddress();
    statusDoc["apIP"] = WiFi.softAPIP().toString();
    statusDoc["apClients"] = WiFi.softAPgetStationNum();
    
    int rssi = WiFi.RSSI();
    String quality;
    if (rssi > -50) quality = "Excellent";
    else if (rssi > -60) quality = "Good";
    else if (rssi > -70) quality = "Fair";
    else if (rssi > -80) quality = "Weak";
    else quality = "Very Weak";
    statusDoc["signalQuality"] = quality;
    
    statusDoc["freeHeap"] = ESP.getFreeHeap();
    statusDoc["cpuFreq"] = getCpuFrequencyMhz();
    statusDoc["uptime"] = millis() / 1000;
    statusDoc["connectionStatus"] = networkSettings.connStatus;
    
    String response;
    serializeJson(statusDoc, response);
    request->send(200, "application/json", response); });

  // =========================================================================
  // ENDPOINT BARU: Ethernet Status and Diagnostic
  // =========================================================================
  server.on("/ethernetStatus", HTTP_GET, [](AsyncWebServerRequest *request)
            {
  DynamicJsonDocument statusDoc(512);
  
  statusDoc["mode"] = networkSettings.networkMode;
  
  if (networkSettings.networkMode == "Ethernet")
  {
    // Hardware Status
    auto hwStatus = Ethernet.hardwareStatus();
    switch (hwStatus)
    {
      case EthernetNoHardware:
        statusDoc["hardware"] = "Not Found";
        statusDoc["hardwareStatus"] = false;
        break;
      case EthernetW5100:
        statusDoc["hardware"] = "W5100";
        statusDoc["hardwareStatus"] = true;
        break;
      case EthernetW5200:
        statusDoc["hardware"] = "W5200";
        statusDoc["hardwareStatus"] = true;
        break;
      case EthernetW5500:
        statusDoc["hardware"] = "W5500";
        statusDoc["hardwareStatus"] = true;
        break;
      default:
        statusDoc["hardware"] = "Unknown";
        statusDoc["hardwareStatus"] = false;
        break;
    }
    
    // Link Status
    auto linkStatus = Ethernet.linkStatus();
    switch (linkStatus)
    {
      case Unknown:
        statusDoc["link"] = "Unknown";
        statusDoc["connected"] = false;
        break;
      case LinkON:
        statusDoc["link"] = "Connected";
        statusDoc["connected"] = true;
        break;
      case LinkOFF:
        statusDoc["link"] = "Disconnected";
        statusDoc["connected"] = false;
        break;
    }
    
    // Network Information
    statusDoc["ip"] = Ethernet.localIP().toString();
    statusDoc["subnet"] = Ethernet.subnetMask().toString();
    statusDoc["gateway"] = Ethernet.gatewayIP().toString();
    statusDoc["dns"] = Ethernet.dnsServerIP().toString();
    statusDoc["macAddress"] = networkSettings.macAddress;
    
    // DHCP Mode
    statusDoc["dhcpMode"] = networkSettings.dhcpMode;
    
    // Connection quality indicator
    if (linkStatus == LinkON)
    {
      statusDoc["quality"] = "Excellent";
      statusDoc["qualityPercent"] = 100;
    }
    else if (linkStatus == LinkOFF)
    {
      statusDoc["quality"] = "No Cable";
      statusDoc["qualityPercent"] = 0;
    }
    else
    {
      statusDoc["quality"] = "Unknown";
      statusDoc["qualityPercent"] = 50;
    }
  }
  else
  {
    statusDoc["error"] = "Not in Ethernet mode";
    statusDoc["connected"] = false;
  }
  
  // AP Status (jika AP aktif)
  statusDoc["apIP"] = WiFi.softAPIP().toString();
  statusDoc["apClients"] = WiFi.softAPgetStationNum();
  statusDoc["apSSID"] = networkSettings.apSsid;
  
  // System Information
  statusDoc["freeHeap"] = ESP.getFreeHeap();
  statusDoc["cpuFreq"] = getCpuFrequencyMhz();
  statusDoc["uptime"] = millis() / 1000;
  statusDoc["connectionStatus"] = networkSettings.connStatus;
  
  String response;
  serializeJson(statusDoc, response);
  request->send(200, "application/json", response); });
  server.on("/networkInfo", HTTP_GET, [](AsyncWebServerRequest *request)
            {
  DynamicJsonDocument infoDoc(768);
  
  infoDoc["currentMode"] = networkSettings.networkMode;
  infoDoc["timestamp"] = getTimeDateNow();
  
  // Ethernet Information
  JsonObject ethInfo = infoDoc.createNestedObject("ethernet");
  if (networkSettings.networkMode == "Ethernet")
  {
    ethInfo["active"] = true;
    ethInfo["hardware"] = "W5500";
    ethInfo["link"] = (Ethernet.linkStatus() == LinkON) ? "Connected" : "Disconnected";
    ethInfo["ip"] = Ethernet.localIP().toString();
    ethInfo["mac"] = networkSettings.macAddress;
  }
  else
  {
    ethInfo["active"] = false;
  }
  
  // WiFi Information
  JsonObject wifiInfo = infoDoc.createNestedObject("wifi");
  if (networkSettings.networkMode == "WiFi")
  {
    wifiInfo["active"] = true;
    wifiInfo["ssid"] = networkSettings.ssid;
    wifiInfo["connected"] = (WiFi.status() == WL_CONNECTED);
    wifiInfo["ip"] = WiFi.localIP().toString();
    wifiInfo["rssi"] = WiFi.RSSI();
    wifiInfo["mac"] = WiFi.macAddress();
  }
  else
  {
    wifiInfo["active"] = false;
  }
  
  // Access Point Information
  JsonObject apInfo = infoDoc.createNestedObject("accessPoint");
  apInfo["ssid"] = networkSettings.apSsid;
  apInfo["ip"] = WiFi.softAPIP().toString();
  apInfo["clients"] = WiFi.softAPgetStationNum();
  apInfo["active"] = apReady;
  
  // Protocol Information
  JsonObject protocolInfo = infoDoc.createNestedObject("protocol");
  protocolInfo["primary"] = networkSettings.protocolMode;
  protocolInfo["endpoint"] = networkSettings.endpoint;
  protocolInfo["port"] = networkSettings.port;
  protocolInfo["modbus"] = networkSettings.protocolMode2;
  protocolInfo["status"] = networkSettings.connStatus;
  
  String response;
  serializeJson(infoDoc, response);
  request->send(200, "application/json", response); });

  // =========================================================================
  // ENDPOINT BARU: Test Ethernet Connectivity
  // =========================================================================
  server.on("/testEthernet", HTTP_GET, [](AsyncWebServerRequest *request)
            {
  DynamicJsonDocument testDoc(256);
  
  if (networkSettings.networkMode != "Ethernet")
  {
    testDoc["success"] = false;
    testDoc["error"] = "Not in Ethernet mode";
    String response;
    serializeJson(testDoc, response);
    request->send(400, "application/json", response);
    return;
  }
  
  // Test hardware
  testDoc["hardwareDetected"] = (Ethernet.hardwareStatus() != EthernetNoHardware);
  
  // Test link
  testDoc["cableConnected"] = (Ethernet.linkStatus() == LinkON);
  
  // Test IP configuration
  testDoc["ipConfigured"] = (Ethernet.localIP() != IPAddress(0, 0, 0, 0));
  
  // Overall test result
  bool allTestsPassed = testDoc["hardwareDetected"].as<bool>() && 
                        testDoc["cableConnected"].as<bool>() && 
                        testDoc["ipConfigured"].as<bool>();
  
  testDoc["success"] = allTestsPassed;
  testDoc["message"] = allTestsPassed ? "All tests passed" : "Some tests failed";
  
  if (allTestsPassed)
  {
    testDoc["ip"] = Ethernet.localIP().toString();
  }
  
  String response;
  serializeJson(testDoc, response);
  request->send(allTestsPassed ? 200 : 500, "application/json", response); });

  // Check connected interface
  tcpip_adapter_ip_info_t ipInfo;
  if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo) == ESP_OK)
  {
    ESP_LOGI("Wi-Fi STA", "Wi-Fi STA IP: %s", ip4addr_ntoa(&ipInfo.ip));
  }
  if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ipInfo) == ESP_OK)
  {
    ESP_LOGI("Wi-Fi AP", "Wi-Fi AP IP: %s", ip4addr_ntoa(&ipInfo.ip));
  }
  if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_ETH, &ipInfo) == ESP_OK)
  {
    ESP_LOGI("Ethernet", "Ethernet IP: %s", ip4addr_ntoa(&ipInfo.ip));
  }
  else
  {
    ESP_LOGW("Ethernet", "Ethernet interface not initialized or no IP assigned.");
  }
  server.serveStatic("/css", SPIFFS, "/css").setCacheControl("max-age=31536000");
  server.serveStatic("/js", SPIFFS, "/js").setCacheControl("max-age=31536000");
  server.serveStatic("/bootstrap.min.css", SPIFFS, "/bootstrap.min.css").setCacheControl("max-age=31536000");
  server.serveStatic("/bootstrap.bundle.min.js", SPIFFS, "/bootstrap.bundle.min.js").setCacheControl("max-age=31536000");
  server.serveStatic("/jquery-3.5.1.min.js", SPIFFS, "/jquery-3.5.1.min.js").setCacheControl("max-age=31536000");
  server.serveStatic("/logoMeditech32.png", SPIFFS, "/logoMeditech32.png").setCacheControl("max-age=31536000");
  server.serveStatic("/logoMeditech256.png", SPIFFS, "/logoMeditech256.png").setCacheControl("max-age=31536000");
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("home.html").setCacheControl("no-cache");
  server.begin();
  if (networkSettings.networkMode == "Ethernet")
  {
    ethServer.begin();
    Serial.println("✅ Ethernet Web Server STARTED on Port 80");
  }
  ESP_LOGI("WebServer", "Web server started successfully");
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  dnsStarted = true;
  Serial.println("DNS Server started, redirecting all domains to " + WiFi.softAPIP().toString());
}

void setupInterrupts()
{
  for (byte i = 1; i <= jumlahInputDigital; i++)
    attachDigitalInputInterrupt(i);
  configureSendTriggerInterrupt(networkSettings);
}

// ============================================================================
// MODBUS SLAVE SETUP
// ============================================================================
void modbusSlaveSetup()
{
  bool useTCP = (networkSettings.protocolMode2 == "Modbus TCP/IP" || networkSettings.protocolMode2 == "Modbus RTU + TCP/IP");
  bool useRTU = (networkSettings.protocolMode2 == "Modbus RTU" || networkSettings.protocolMode2 == "Modbus RTU + TCP/IP");

  if (useTCP)
    mbIP.server(502);

  if (useRTU)
    mbRTU.begin(&SerialModbus);

  for (byte i = 0; i < 100; i++)
  {
    if (useTCP)
      mbIP.addIreg(i);
    if (useRTU)
      mbRTU.addIreg(i);
  }
}

float filterSensor(float filterVar, float filterResult_1, float fc)
{
  // Safety check: Jika fc terlalu kecil, skip filter
  if (fc < 0.01)
    return filterVar;

  float Ts = 0.1; // Sampling time (sesuai loop 100ms)
  float RC = 1.0 / (2.0 * 3.14159 * fc);
  float alpha = Ts / (RC + Ts); // Normalized (0-1)

  // Low-pass filter: Y = alpha*X + (1-alpha)*Y_prev
  float filterResult = (alpha * filterVar) + ((1.0 - alpha) * filterResult_1);

  // Safety: Jika hasil tidak masuk akal, return raw value
  if (isnan(filterResult) || isinf(filterResult) || filterResult > 30000 || filterResult < -1000)
  {
    return filterVar;
  }

  return filterResult;
}
unsigned int readModbusNonBlocking(unsigned int modbusAddress, unsigned int funCode, unsigned int regAddress, unsigned int timeoutMs, bool &success)
{
  unsigned int buffSend[8];
  unsigned int crcValue;
  byte buffModbus[64];
  int resIndex = 0;
  success = false; // Default gagal

  // 1. Bersihkan Buffer
  int flushLimit = 0;
  while (SerialModbus.available() && flushLimit < 100)
  {
    SerialModbus.read();
    flushLimit++;
    delay(1);
  }

  // 2. Siapkan Request
  buffSend[0] = modbusAddress;
  buffSend[1] = funCode;
  buffSend[2] = parseByte(regAddress, 1);
  buffSend[3] = parseByte(regAddress, 0);
  buffSend[4] = 0;
  buffSend[5] = 1; // Minta 1 Register (Ubah ke 2 jika sensor Float)

  crcValue = crcModbus(buffSend, 0, 6);
  buffSend[6] = parseByte(crcValue, 1);
  buffSend[7] = parseByte(crcValue, 0);

  // 3. Kirim
  for (byte j = 0; j < 8; j++)
  {
    SerialModbus.write(buffSend[j]);
  }
  SerialModbus.flush();

  // 4. Tunggu Respon
  unsigned long startWait = millis();
  bool dataComing = false;
  while (millis() - startWait < timeoutMs)
  {
    if (SerialModbus.available())
    {
      dataComing = true;
      break;
    }
    vTaskDelay(1);
  }

  if (!dataComing)
    return 0; // Timeout

  // 5. Baca Data
  vTaskDelay(pdMS_TO_TICKS(20)); // Tunggu buffer penuh
  while (SerialModbus.available() && resIndex < 60)
  {
    buffModbus[resIndex] = SerialModbus.read();
    resIndex++;
  }

  // 6. Validasi
  unsigned int returnValue = 0;
  if (resIndex >= 5)
  {
    if (buffModbus[0] == modbusAddress && buffModbus[1] == funCode)
    {
      if (funCode == 3 || funCode == 4)
      {
        returnValue = (buffModbus[3] << 8) | buffModbus[4];
        success = true; // Data Valid!
      }
    }
  }

  return returnValue;
}

// unsigned int readModbus(unsigned int modbusAddress, unsigned int funCode, unsigned int regAddress)
// {
//   unsigned int buffSend[8], crcValue, returnValue;
//   int resIndex = 0;
//   byte buffModbus[20];

//   buffSend[0] = modbusAddress;
//   buffSend[1] = funCode;
//   buffSend[2] = parseByte(regAddress, 1);
//   buffSend[3] = parseByte(regAddress, 0);
//   buffSend[4] = 0;
//   buffSend[5] = 1;
//   crcValue = crcModbus(buffSend, 0, 6);
//   buffSend[6] = parseByte(crcValue, 1);
//   buffSend[7] = parseByte(crcValue, 0);

//   for (byte j = 0; j < 8; j++)
//   {
//     SerialModbus.write(buffSend[j]);
//     delayMicroseconds(100);
//   }

//   while (!(SerialModbus.available()))
//   {
//     for (int timeout = 0; timeout < 500; timeout++)
//     {
//       delay(1);
//     }
//     break;
//   }

//   while (SerialModbus.available())
//   {
//     buffModbus[resIndex] = SerialModbus.read();
//     resIndex++;
//   }

//   if (funCode == 1 or funCode == 2)
//   {
//     if (buffModbus[2] == 1)
//     {
//       returnValue = buffModbus[3];
//     }
//     else if (buffModbus[2] == 2)
//     {
//       returnValue = buffModbus[3] << 8 | buffModbus[4];
//     }
//   }
//   else if (funCode == 3)
//   {
//     returnValue = (buffModbus[3] << 8) | buffModbus[4];
//   }
//   else if (funCode == 4)
//   {
//     returnValue = (buffModbus[3] << 8) | buffModbus[4];
//   }
//   else if (funCode == 6)
//   {
//     if (buffModbus[4] == buffSend[4] and buffModbus[5] == buffSend[5])
//     {
//       returnValue = 1;
//     }
//     else
//     {
//       returnValue = 0;
//     }
//   }

//   return returnValue;
// }

// unsigned int readModbus(unsigned int modbusAddress, unsigned int funCode, unsigned int regAddress)
// {
//   unsigned int buffSend[8];
//   unsigned int crcValue;
//   // [FIX 1] Inisialisasi returnValue ke 0. Jika gagal baca, hasilnya 0 (bukan sampah 10/100)
//   unsigned int returnValue = 0;

//   byte buffModbus[32]; // Perbesar buffer sedikit
//   int resIndex = 0;

//   // Bersihkan buffer sebelum dipakai
//   memset(buffModbus, 0, sizeof(buffModbus));

//   // 1. Susun Frame Request
//   buffSend[0] = modbusAddress;
//   buffSend[1] = funCode;
//   buffSend[2] = parseByte(regAddress, 1); // High Byte Address
//   buffSend[3] = parseByte(regAddress, 0); // Low Byte Address
//   buffSend[4] = 0;
//   buffSend[5] = 1; // Jumlah data yg diminta (1 Register)

//   crcValue = crcModbus(buffSend, 0, 6);
//   buffSend[6] = parseByte(crcValue, 1);
//   buffSend[7] = parseByte(crcValue, 0);

//   // 2. Bersihkan RX Buffer sebelum kirim request baru
//   while(SerialModbus.available()) SerialModbus.read();

//   // 3. Kirim Request ke RS485
//   for (byte j = 0; j < 8; j++)
//   {
//     SerialModbus.write(buffSend[j]);
//     // delayMicroseconds(100); // Opsional, tergantung hardware
//   }
//   SerialModbus.flush(); // Pastikan data terkirim semua

//   // 4. Tunggu Respon (Timeout Logic yang Benar)
//   unsigned long startTime = millis();
//   bool dataReceived = false;

//   // Tunggu sampai ada data pertama masuk (Timeout 1 detik)
//   while ((millis() - startTime) < 1000)
//   {
//     if (SerialModbus.available()) {
//       dataReceived = true;
//       break;
//     }
//     delay(10);
//   }

//   // 5. Jika Timeout (Tidak ada balasan dari TK4S)
//   if (!dataReceived) {
//     // Return 0 agar ketahuan kalau error/putus (JANGAN return sampah)
//     return 0;
//   }

//   // 6. Baca Data Balasan
//   // Beri jeda sedikit untuk memastikan seluruh byte sampai
//   delay(20);

//   while (SerialModbus.available() && resIndex < 30)
//   {
//     buffModbus[resIndex] = SerialModbus.read();
//     resIndex++;
//   }

//   // 7. Parsing Data (Validasi Function Code)
//   // Respon Input Register (FC 04) atau Holding Register (FC 03)
//   // Format: [Addr] [FC] [ByteCount] [DataHigh] [DataLow] [CRC] [CRC]
//   // Index:    0      1       2           3         4

//   if (funCode == 3 || funCode == 4)
//   {
//     // Validasi dasar: Address & Function code harus sesuai request
//     if (buffModbus[0] == modbusAddress && buffModbus[1] == funCode && buffModbus[2] == 2)
//     {
//        returnValue = (buffModbus[3] << 8) | buffModbus[4];
//     }
//   }
//   // Respon Coil/Status (FC 01 / 02)
//   else if (funCode == 1 || funCode == 2)
//   {
//     // Cek Byte Count (biasanya 1 byte data)
//     if (buffModbus[2] == 1)
//     {
//       returnValue = buffModbus[3];
//     }
//   }

//   return returnValue;
// }

// ============================================================================
// FUNGSI BACA MODBUS (DENGAN SERIAL PRINT DEBUG LENGKAP)
// ============================================================================
unsigned int readModbus(unsigned int modbusAddress, unsigned int funCode, unsigned int regAddress)
{
  unsigned int buffSend[8];
  unsigned int crcValue;
  unsigned int returnValue = 0;
  byte buffModbus[64];
  int resIndex = 0;
  unsigned long startTime;

  // [FIX 1] Bounded Loop untuk Clear Buffer (Anti-Hang jika ada noise)
  int clearCount = 0;
  while (SerialModbus.available() && clearCount < 100)
  {
    SerialModbus.read();
    clearCount++;
  }

  // 2. Siapkan Request
  buffSend[0] = modbusAddress;
  buffSend[1] = funCode;
  buffSend[2] = parseByte(regAddress, 1);
  buffSend[3] = parseByte(regAddress, 0);
  buffSend[4] = 0;
  buffSend[5] = 1;
  crcValue = crcModbus(buffSend, 0, 6);
  buffSend[6] = parseByte(crcValue, 1);
  buffSend[7] = parseByte(crcValue, 0);

  // Kirim Request
  for (byte j = 0; j < 8; j++)
  {
    SerialModbus.write(buffSend[j]);
  }
  SerialModbus.flush();

  // 3. Tunggu Respon (Timeout 200ms)
  startTime = millis();
  while (millis() - startTime < 200)
  {
    if (SerialModbus.available())
      break;
    vTaskDelay(1); // [FIX 2] Wajib Yield agar Task lain jalan
  }

  // Jika timeout
  if (!SerialModbus.available())
    return 0;

  delay(10); // Tunggu byte lengkap

  // 4. Baca Balasan dengan Batas Waktu & Jumlah
  startTime = millis();
  while (millis() - startTime < 50 && resIndex < 60) // [FIX 3] Double protection
  {
    if (SerialModbus.available())
    {
      buffModbus[resIndex] = SerialModbus.read();
      resIndex++;
    }
  }

  // 5. Validasi & Ambil Data
  if (resIndex >= 5) // Minimal 5 byte (Addr, FC, ByteCount, DataH, DataL)
  {
    if (buffModbus[0] == modbusAddress && buffModbus[1] == funCode)
    {
      if (funCode == 3 || funCode == 4)
      {
        returnValue = (buffModbus[3] << 8) | buffModbus[4];
      }
    }
  }

  return returnValue;
}

unsigned int crcModbus(unsigned int crc[], byte start, byte sizeArray)
{
  unsigned int crcReg = 0xffff;
  for (byte j = start; j < sizeArray; j++)
  {
    crcReg ^= crc[j];
    for (byte i = 0; i < 8; i++)
    {
      if (crcReg & 1)
      {
        crcReg >>= 1;
        crcReg ^= 0xa001;
        continue;
      }
      crcReg >>= 1;
    }
  }
  unsigned int temp = 0;
  temp = crcReg & 0xff;
  crcReg = ((crcReg & 0xff00) >> 8) | (temp << 8);
  return crcReg;
}

unsigned int parseByte(unsigned int bytes, bool byteOrder)
{
  unsigned int parseResult;
  if (byteOrder == 0)
    parseResult = bytes & 0xff;
  else
    parseResult = (bytes & 0xff00) >> 8;
  return parseResult;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  float mappedValue = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return roundf(mappedValue * 100.0) / 100.0;
}
float calculate_Measurement(float mA, float minRange, float maxRange)
{
  // (Max - Min) --> Ini adalah Span (Rentang pengukuran)
  float span = maxRange - minRange;

  // Min + ((mA - 4) * Span / 16) --> Rumus standar instrumentasi 4-20mA
  // 16.0 adalah rentang arus (20mA - 4mA)
  float result = minRange + ((mA - 4.0) * span / 16.0);

  return result;
}
String getTimeDateNow()
{
  DateTime now;
  if (rtc.begin())
    now = rtc.now();
  else
  {
    struct tm timeInfo;
    if (!getLocalTime(&timeInfo))
    {
      ESP_LOGW("NTP", "Failed to obtain time from NTP server");
      now = DateTime(0, 0, 0, 0, 0, 0);
    }
    else
    {
      now = DateTime(timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
    }
  }

  char timeBuffer[32];
  sprintf(timeBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  // timestamp = year + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + second;
  return String(timeBuffer);
}

String getTimeNow()
{
  String dateTime = getTimeDateNow();
  int spaceIndex = dateTime.indexOf(' ');
  if (spaceIndex != -1)
  {
    return dateTime.substring(spaceIndex + 1);
  }
  return dateTime;
}

void authenthicateUser(AsyncWebServerRequest *request)
{
  if (!request->authenticate(networkSettings.loginUsername.c_str(), networkSettings.loginPassword.c_str()))
    return request->requestAuthentication();
}

// ============================================================================
// FILE & CONFIG HANDLING FUNCTIONS
// ============================================================================

void readConfig()
{
  ESP_LOGI("SPIFFS", "Mounting file system");

  if (SPIFFS.begin())
  {
    ESP_LOGI("SPIFFS", "File system mounted");

    // Read Network Config
    if (SPIFFS.exists("/configNetwork.json"))
    {
      doc = DynamicJsonDocument(4096);
      ESP_LOGI("SPIFFS", "Reading network config file...");
      File configFile = SPIFFS.open("/configNetwork.json");

      if (configFile)
      {
        ESP_LOGI("SPIFFS", "Opened network config file");
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        auto error = deserializeJson(doc, buf.get());

        if (error)
        {
          ESP_LOGE("SPIFFS NETWORK", "Failed to parse JSON file");
          return;
        }

        const char *temp;
        temp = doc["networkMode"];
        networkSettings.networkMode = String(temp);
        temp = doc["ssid"];
        networkSettings.ssid = String(temp);
        temp = doc["password"];
        networkSettings.password = String(temp);
        temp = doc["apSsid"];
        networkSettings.apSsid = String(temp);
        temp = doc["apPassword"];
        networkSettings.apPassword = String(temp);
        temp = doc["dhcpMode"];
        networkSettings.dhcpMode = String(temp);
        temp = doc["ipAddress"];
        networkSettings.ipAddress = String(temp);
        temp = doc["subnet"];
        networkSettings.subnetMask = String(temp);
        temp = doc["ipGateway"];
        networkSettings.ipGateway = String(temp);
        temp = doc["ipDNS"];
        networkSettings.ipDNS = String(temp);
        temp = doc["protocolMode"];
        networkSettings.protocolMode = String(temp);
        temp = doc["endpoint"];
        networkSettings.endpoint = String(temp);
        temp = doc["pubTopic"];
        networkSettings.pubTopic = String(temp);
        temp = doc["subTopic"];
        networkSettings.subTopic = String(temp);
        temp = doc["mqttUsername"];
        networkSettings.mqttUsername = String(temp);
        temp = doc["mqttPass"];
        networkSettings.mqttPassword = String(temp);
        // networkSettings.sendInterval = doc["sendInterval"];
        if (doc.containsKey("sendInterval"))
          networkSettings.sendInterval = doc["sendInterval"];
        temp = doc["sendTrig"];
        networkSettings.sendTrig = String(temp);
        networkSettings.port = doc["port"];
        networkSettings.loggerMode = doc["loggerMode"].as<String>();
        const char *tempProtocol2 = doc["protocolMode2"]; // Use explicit const char*
        networkSettings.protocolMode2 = String(tempProtocol2);
        modbusParam.mode = doc["modbusMode"].as<String>();
        modbusParam.port = doc["modbusPort"];
        modbusParam.slaveID = doc["modbusSlaveID"];
        networkSettings.erpUrl = doc["erpUrl"].as<String>();
        networkSettings.erpUsername = doc["erpUsername"].as<String>();
        networkSettings.erpPassword = doc["erpPassword"].as<String>();
        // temp = doc["protocolMode2"];
        // networkSettings.protocolMode2 = String(temp);
        // modbusParam.mode = doc["modbusMode"];
        // modbusParam.port = doc["modbusPort"];
        // modbusParam.slaveID = doc["modbusSlaveID"];
        // temp = doc["erpUrl"];
        // networkSettings.erpUrl = String(temp);
        // temp = doc["erpUsername"];
        // networkSettings.erpUsername = String(temp);
        // temp = doc["erpPassword"];
        // networkSettings.erpPassword = String(temp);

        configNetwork();
        configProtocol();
      }
    }

    // Read Digital Config
    if (SPIFFS.exists("/configDigital.json"))
    {
      doc = DynamicJsonDocument(4096);
      ESP_LOGI("SPIFFS", "Reading digital input config file...");
      File configFile = SPIFFS.open("/configDigital.json");

      if (configFile)
      {
        ESP_LOGI("SPIFFS", "Opened digital input config file");
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        auto error = deserializeJson(doc, buf.get());

        if (error)
        {
          ESP_LOGE("SPIFFS DIGITAL", "Failed to parse JSON file");
          return;
        }

        const char *temp;
        for (unsigned char i = 1; i < jumlahInputDigital + 1; i++)
        {
          temp = doc["DI" + String(i)]["name"];
          digitalInput[i].name = String(temp);
          digitalInput[i].inv = doc["DI" + String(i)]["invers"];
          temp = doc["DI" + String(i)]["taskMode"];
          digitalInput[i].taskMode = String(temp);
          temp = doc["DI" + String(i)]["inputState"];
          digitalInput[i].inputState = String(temp) == "High" ? 1 : 0;
          digitalInput[i].intervalTime = doc["DI" + String(i)]["intervalTime"];
          digitalInput[i].conversionFactor = doc["DI" + String(i)]["conversionFactor"];
        }
      }
    }

    // Read Analog Config
    if (SPIFFS.exists("/configAnalog.json"))
    {
      doc = DynamicJsonDocument(4096);
      Serial.println("Reading analog input config file...");
      File configFile = SPIFFS.open("/configAnalog.json");

      if (configFile)
      {
        ESP_LOGI("SPIFFS", "Opened analog input config file");
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        auto error = deserializeJson(doc, buf.get());

        if (error)
        {
          ESP_LOGE("SPIFFS ANALOG", "Failed to parse JSON file");
          return;
        }

        const char *temp;
        for (unsigned char i = 1; i < jumlahInputAnalog + 1; i++)
        {
          temp = doc["AI" + String(i)]["name"];
          analogInput[i].name = String(temp);
          temp = doc["AI" + String(i)]["inputType"];
          analogInput[i].inputType = String(temp);
          analogInput[i].filter = doc["AI" + String(i)]["filter"];
          analogInput[i].filterPeriod = doc["AI" + String(i)]["filterPeriod"];
          analogInput[i].scaling = doc["AI" + String(i)]["scaling"];
          analogInput[i].lowLimit = doc["AI" + String(i)]["lowLimit"];
          analogInput[i].highLimit = doc["AI" + String(i)]["highLimit"];
          // analogInput[i].calibration = doc["AI" + String(i)]["calibration"];
          // analogInput[i].mValue = doc["AI" + String(i)]["mValue"];
          // analogInput[i].cValue = doc["AI" + String(i)]["cValue"];
          calculateAnalogCalibration(i);
        }
      }
    }

    // Read Modbus Config
    if (SPIFFS.exists("/modbusSetup.json"))
    {
      ESP_LOGI("SPIFFS", "Reading Modbus config file...");
      File configFile = SPIFFS.open("/modbusSetup.json");

      if (configFile)
      {
        ESP_LOGI("SPIFFS", "Opened Modbus config file");
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        auto error = deserializeJson(jsonParam, buf.get());

        if (error)
        {
          ESP_LOGE("SPIFFS MODBUS", "Failed to parse JSON file");
          return;
        }

        const char *temp;
        temp = jsonParam["parity"];
        modbusParam.parity = String(temp);
        modbusParam.baudrate = jsonParam["baudrate"];
        modbusParam.stopBit = jsonParam["stopBit"];
        modbusParam.dataBit = jsonParam["dataBit"];
        modbusParam.scanRate = jsonParam["scanRate"];

        // Configure Serial Modbus
        if (modbusParam.dataBit == 8 and modbusParam.stopBit == 1 and modbusParam.parity == "None")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_8N1, 17, 16);
        else if (modbusParam.dataBit == 8 and modbusParam.stopBit == 2 and modbusParam.parity == "None")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_8N2, 17, 16);
        else if (modbusParam.dataBit == 8 and modbusParam.stopBit == 1 and modbusParam.parity == "Odd")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_8O1, 17, 16);
        else if (modbusParam.dataBit == 8 and modbusParam.stopBit == 2 and modbusParam.parity == "Odd")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_8O2, 17, 16);
        else if (modbusParam.dataBit == 8 and modbusParam.stopBit == 1 and modbusParam.parity == "Even")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_8E1, 17, 16);
        else if (modbusParam.dataBit == 8 and modbusParam.stopBit == 2 and modbusParam.parity == "Even")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_8E2, 17, 16);
        else if (modbusParam.dataBit == 7 and modbusParam.stopBit == 1 and modbusParam.parity == "None")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_7N1, 17, 16);
        else if (modbusParam.dataBit == 7 and modbusParam.stopBit == 2 and modbusParam.parity == "None")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_7N2, 17, 16);
        else if (modbusParam.dataBit == 7 and modbusParam.stopBit == 1 and modbusParam.parity == "Odd")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_7O1, 17, 16);
        else if (modbusParam.dataBit == 7 and modbusParam.stopBit == 2 and modbusParam.parity == "Odd")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_7O2, 17, 16);
        else if (modbusParam.dataBit == 7 and modbusParam.stopBit == 1 and modbusParam.parity == "Even")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_7E1, 17, 16);
        else if (modbusParam.dataBit == 7 and modbusParam.stopBit == 2 and modbusParam.parity == "Even")
          SerialModbus.begin(modbusParam.baudrate, SERIAL_7E2, 17, 16);

        JsonArray nameData = jsonParam["nameData"];
        numOfParam = nameData.size();

        stringParam = "";
        serializeJson(jsonParam, stringParam);
      }
    }

    // Read Runtime Data
    if (SPIFFS.exists("/runtimeData.json"))
    {
      File runtimeFile = SPIFFS.open("/runtimeData.json");
      doc = DynamicJsonDocument(4096);

      if (runtimeFile)
      {
        size_t size = runtimeFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        runtimeFile.readBytes(buf.get(), size);
        auto error = deserializeJson(doc, buf.get());

        if (!error)
        {
          for (int i = 1; i < jumlahInputDigital + 1; i++)
          {
            if (digitalInput[i].taskMode == "Run Time")
            {
              digitalInput[i].value = doc[String(i)];
            }
          }
        }
      }
    }

    // Read System Settings
    if (SPIFFS.exists("/systemSettings.json"))
    {
      File runtimeFile = SPIFFS.open("/systemSettings.json");
      doc = DynamicJsonDocument(4096);

      if (runtimeFile)
      {
        size_t size = runtimeFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        runtimeFile.readBytes(buf.get(), size);
        auto error = deserializeJson(doc, buf.get());

        if (!error)
        {
          const char *temp;
          temp = doc["username"];
          networkSettings.loginUsername = temp;
          temp = doc["password"];
          networkSettings.loginPassword = temp;

          if (doc.containsKey("sdInterval"))
          {
            networkSettings.sdSaveInterval = doc["sdInterval"];
          }
        }
      }
    }
  }
  else
  {
    ESP_LOGE("SPIFFS", "Failed to mount FS");
    errorMessages.addMessage(getTimeNow() + " - Failed to mount FS");
  }
}

void updateJson(const char *dir, const char *jsonKey, const char *jsonValue)
{
  if (SPIFFS.begin())
  {
    File file = SPIFFS.open(dir);
    if (file)
    {
      size_t size = file.size();
      std::unique_ptr<char[]> buf(new char[size]);
      file.readBytes(buf.get(), size);
      file.close();
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, buf.get());
      if (!error)
      {
        doc[jsonKey] = jsonValue;
        File updatedFile = SPIFFS.open(dir, "w");
        if (updatedFile)
        {
          serializeJson(doc, updatedFile);
          updatedFile.close();
        }
      }
    }
  }
}

void updateJson(const char *dir, const char *jsonKey, int jsonValue)
{
  if (SPIFFS.begin())
  {
    File file = SPIFFS.open(dir);
    if (file)
    {
      size_t size = file.size();
      std::unique_ptr<char[]> buf(new char[size]);
      file.readBytes(buf.get(), size);
      file.close();
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, buf.get());
      if (!error)
      {
        doc[jsonKey] = jsonValue;
        File updatedFile = SPIFFS.open(dir, "w");
        if (updatedFile)
        {
          serializeJson(doc, updatedFile);
          updatedFile.close();
        }
      }
    }
  }
}

void handleFileRequest(AsyncWebServerRequest *request, const char *filePath, const char *mimeType)
{
  if (SPIFFS.exists(filePath))
  {
    File file = SPIFFS.open(filePath, "r");
    request->send(SPIFFS, filePath, mimeType);
    file.close();
  }
  else
  {
    request->send(404, "text/plain", "File Not Found");
  }
}

void handleFormSubmit(AsyncWebServerRequest *request)
{
  Serial.println("Masuk Submit Form");
  Serial.println("\n--- [DEBUG] FORM DATA RECEIVED ---");
  if (request->hasArg("loggerMode"))
  {
    Serial.print("1. Logger Mode Raw: ");
    Serial.println(request->arg("loggerMode"));
  }
  else
  {
    Serial.println("1. Logger Mode Raw: NOT FOUND / NULL");
  }

  if (request->hasArg("modbusMode"))
  {
    Serial.print("2. Modbus Mode Raw: ");
    Serial.println(request->arg("modbusMode"));
  }
  else
  {
    Serial.println("2. Modbus Mode Raw: NOT FOUND / NULL");
  }
  Serial.println("----------------------------------\n");
  // ========================================================================
  // 1. SAVE NETWORK CONFIGURATION (Mode, IP, MQTT, dll)
  // ========================================================================
  if (request->hasArg("networkMode") && request->hasArg("dhcpMode"))
  {
    networkSettings.networkMode = request->arg("networkMode");
    networkSettings.dhcpMode = request->arg("dhcpMode");
    networkSettings.apSsid = request->arg("apSsid");
    networkSettings.apPassword = request->arg("apPassword");
    networkSettings.sendTrig = request->arg("sendTrig");
    networkSettings.sendInterval = request->arg("sendInterval").toFloat();

    if (request->hasArg("ssid"))
    {
      networkSettings.ssid = request->arg("ssid");
      networkSettings.password = request->arg("password");
    }
    if (request->hasArg("ipAddress"))
    {
      networkSettings.ipAddress = request->arg("ipAddress");
      networkSettings.subnetMask = request->arg("subnet");
      networkSettings.ipGateway = request->arg("ipGateway");
      networkSettings.ipDNS = request->arg("ipDNS");
    }

    networkSettings.protocolMode = request->arg("protocolMode");
    networkSettings.endpoint = request->arg("endpoint");
    networkSettings.port = request->arg("port").toInt();

    if (request->hasArg("pubTopic"))
    {
      networkSettings.pubTopic = request->arg("pubTopic");
      networkSettings.subTopic = request->arg("subTopic");
    }
    if (request->hasArg("mqttUsername"))
      networkSettings.mqttUsername = request->arg("mqttUsername");
    if (request->hasArg("mqttPass"))
      networkSettings.mqttPassword = request->arg("mqttPass");
    if (request->hasArg("loggerMode"))
    {
      networkSettings.loggerMode = request->arg("loggerMode");
    }
    else
    {
      networkSettings.loggerMode = "Disabled";
    }
    if (request->hasArg("modbusMode"))
    {
      networkSettings.protocolMode2 = request->arg("protocolMode2");
      if (request->hasArg("modbusPort"))
        modbusParam.port = request->arg("modbusPort").toInt();
      if (request->hasArg("modbusSlaveID"))
        modbusParam.slaveID = request->arg("modbusSlaveID").toInt();
    }
    configureSendTriggerInterrupt(networkSettings);
    saveToJson("/configNetwork.json", "network");
    saveToSDConfig("/configNetwork.json", "network");
    request->send(200, "text/plain", "Network saved. Restarting...");
    delay(500);
    ESP.restart();
  }

  // ========================================================================
  // 2. SAVE ERP CONFIGURATION (URL, User, Pass)
  // ========================================================================
  if (request->hasArg("erpUrl"))
  {
    networkSettings.erpUrl = request->arg("erpUrl");
    networkSettings.erpUsername = request->arg("erpUsername");
    networkSettings.erpPassword = request->arg("erpPassword");
    Serial.print("ERP URL: ");
    Serial.println(networkSettings.erpUrl);
    Serial.print("ERP Username: ");
    Serial.println(networkSettings.erpUsername);
    Serial.print("ERP Password: ");
    Serial.println(networkSettings.erpPassword);
    request->send(200, "text/plain", "Form data received");
    saveToJson("/configNetwork.json", "network");
    saveToSDConfig("/configNetwork.json", "network");
  }

  // ========================================================================
  // 3. SAVE DIGITAL INPUT CONFIGURATION
  // ========================================================================
  // else if (request->hasArg("nameDI"))
  // {
  //   for (unsigned char i = 1; i < jumlahInputDigital + 1; i++)
  //   {
  //     if (request->arg("inputPin") == ("DI" + String(i)))
  //     {
  //       digitalInput[i].name = request->arg("nameDI");
  //       if (digitalInput[i].taskMode != request->arg("taskMode"))
  //         digitalInput[i].value = 0;
  //       digitalInput[i].taskMode = request->arg("taskMode");
  //       attachDigitalInputInterrupt(i);
  //       if (digitalInput[i].taskMode == "Run Time")
  //       {
  //         if (SPIFFS.begin())
  //         {
  //           File file = SPIFFS.open("/runtimeData.json");
  //           if (file)
  //           {
  //             size_t size = file.size();
  //             std::unique_ptr<char[]> buf(new char[size]);
  //             file.readBytes(buf.get(), size);
  //             file.close();
  //             DynamicJsonDocument runtime(128);
  //             DeserializationError error = deserializeJson(runtime, buf.get());
  //             if (!error)
  //             {
  //               digitalInput[i].value = runtime[String(i)];
  //             }
  //           }
  //         }
  //
  //       if (request->arg("inputState") == "High")
  //         digitalInput[i].inputState = 1;
  //       else
  //         digitalInput[i].inputState = 0;
  //       if (request->hasArg("inputInversion"))
  //         digitalInput[i].inv = 1;
  //       else
  //         digitalInput[i].inv = 0;

  //       if (request->hasArg("intervalTime"))
  //         digitalInput[i].intervalTime = static_cast<long>(request->arg("intervalTime").toFloat() * 1000);
  //       if (request->hasArg("conversionFactor"))
  //         digitalInput[i].conversionFactor = request->arg("conversionFactor").toFloat();
  //     }
  //   }

  //   configureSendTriggerInterrupt(networkSettings);

  //   request->send(200, "text/plain", "Form data received");

  //   // Simpan ke Internal & SD Card
  //   saveToJson("/configDigital.json", "digital");
  //   saveToSDConfig("/configDigital.json", "digital");
  // }

  else if (request->hasArg("nameDI"))
  {
    String targetPin = request->arg("inputPin"); // Misal: "DI1"

    // Ambil Mutex JSON saat mengubah variabel global config
    // Ini mencegah Data Acquisition Task membaca data "setengah matang"
    if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(200)))
    {
      for (unsigned char i = 1; i <= jumlahInputDigital; i++)
      {
        if (targetPin == ("DI" + String(i)))
        {
          digitalInput[i].name = request->arg("nameDI");

          // Reset value jika mode berubah
          if (digitalInput[i].taskMode != request->arg("taskMode"))
            digitalInput[i].value = 0;

          digitalInput[i].taskMode = request->arg("taskMode");

          // Attach ulang interrupt jika perlu
          attachDigitalInputInterrupt(i);

          // Handle Run Time Persistence
          if (digitalInput[i].taskMode == "Run Time")
          {
            // Cek file runtime (tanpa SPIFFS.begin lagi!)
            if (SPIFFS.exists("/runtimeData.json"))
            {
              File file = SPIFFS.open("/runtimeData.json", "r");
              if (file)
              {
                DynamicJsonDocument runtime(512);
                deserializeJson(runtime, file);
                digitalInput[i].value = runtime[String(i)];
                file.close();
              }
            }
          }

          // Input State logic
          digitalInput[i].inputState = (request->arg("inputState") == "High") ? 1 : 0;

          // Inversion logic
          digitalInput[i].inv = request->hasArg("inputInversion") ? 1 : 0;

          // Numeric logic
          if (request->hasArg("intervalTime"))
            digitalInput[i].intervalTime = static_cast<long>(request->arg("intervalTime").toFloat() * 1000);

          if (request->hasArg("conversionFactor"))
            digitalInput[i].conversionFactor = request->arg("conversionFactor").toFloat();
          if (digitalInput[i].name != "")
          {
            jsonSend[digitalInput[i].name] = digitalInput[i].value;
            jsonSend[digitalInput[i].name + "_mode"] = digitalInput[i].taskMode;
          }
          break;
        }
      }
      xSemaphoreGive(jsonMutex); // Lepas Mutex
    }
    configureSendTriggerInterrupt(networkSettings);
    request->send(200, "text/plain", "Digital Config Saved");
    saveToJson("/configDigital.json", "digital");
    saveToSDConfig("/configDigital.json", "digital");
  }

  // ========================================================================
  // 4. SAVE ANALOG INPUT CONFIGURATION
  // ========================================================================
  else if (request->hasArg("inputType"))
  {
    for (unsigned char i = 1; i < jumlahInputAnalog + 1; i++)
    {
      if (request->arg("inputPin") == ("AI" + String(i)))
      {
        analogInput[i].name = request->arg("name");
        analogInput[i].inputType = request->arg("inputType");
        analogInput[i].filter = request->hasArg("filter") ? 1 : 0;
        // Scaling flag is still saved, but y=mx+c is always applied now
        analogInput[i].scaling = request->hasArg("scaling") ? 1 : 0;

        analogInput[i].filterPeriod = request->arg("filterPeriod").toFloat();

        // Save User Limits
        analogInput[i].lowLimit = request->arg("lowLimit").toFloat();
        analogInput[i].highLimit = request->arg("highLimit").toFloat();

        // AUTO-CALCULATE m AND c BASED ON YOUR FORMULA
        // This ignores user input for m/c and calculates it from Low/High limits
        calculateAnalogCalibration(i);
      }
    }
    request->send(200, "text/plain", "Analog Config Saved & Calibrated");

    // Simpan ke Internal & SD Card
    saveToJson("/configAnalog.json", "analog");
    saveToSDConfig("/configAnalog.json", "analog");
  }

  // ========================================================================
  // 5. SAVE MODBUS SETUP (Baudrate, Parity, dll)
  // ========================================================================
  else if (request->hasArg("baudrate"))
  {
    modbusParam.scanRate = request->arg("scanRate").toFloat();
    modbusParam.stopBit = request->arg("stopBit").toInt();
    modbusParam.dataBit = request->arg("dataBit").toInt();
    modbusParam.baudrate = request->arg("baudrate").toInt();
    modbusParam.parity = request->arg("parity");

    request->send(200, "text/plain", "Form data received");

    // Update JSON global 'jsonParam' agar sinkron
    jsonParam["baudrate"] = modbusParam.baudrate;
    jsonParam["parity"] = modbusParam.parity;
    jsonParam["stopBit"] = modbusParam.stopBit;
    jsonParam["dataBit"] = modbusParam.dataBit;
    jsonParam["scanRate"] = modbusParam.scanRate;

    // Simpan ke Internal & SD Card
    saveToJson("/modbusSetup.json", "modbusSetup");
    saveToSDConfig("/modbusSetup.json", "modbusSetup");
  }

  // ========================================================================
  // 6. SAVE SYSTEM SETTINGS (Username, Pass, Date)
  // ========================================================================
  else if (request->hasArg("username"))
  {
    if (request->arg("datetime") != "")
    {
      String dateTimeUpdate = request->arg("datetime");
      unsigned int updatedYear = dateTimeUpdate.substring(0, 4).toInt();
      unsigned int updatedMonth = dateTimeUpdate.substring(5, 7).toInt();
      unsigned int updatedDay = dateTimeUpdate.substring(8, 10).toInt();
      unsigned int updatedHour = dateTimeUpdate.substring(11, 13).toInt();
      unsigned int updatedMinute = dateTimeUpdate.substring(14).toInt();
      rtc.adjust(DateTime(updatedYear, updatedMonth, updatedDay, updatedHour, updatedMinute, 0));
    }

    networkSettings.loginUsername = request->arg("username");
    networkSettings.loginPassword = request->arg("password");

    if (request->hasArg("sdInterval"))
    {
      networkSettings.sdSaveInterval = request->arg("sdInterval").toInt();
    }

    request->send(200, "text/plain", "Form data received");

    // Simpan ke Internal & SD Card
    saveToJson("/systemSettings.json", "systemSettings");
    saveToSDConfig("/systemSettings.json", "systemSettings");
  }
  else
  {
    request->send(400, "text/plain", "Bad Request");
  }
}

void saveToJson(const char *dir, const char *configType)
{
  // 1. Pastikan ambil Mutex dulu agar tidak bentrok dengan pembacaan sensor
  if (xSemaphoreTake(jsonMutex, pdMS_TO_TICKS(2000)))
  {
    // Gunakan buffer cukup besar (4KB) untuk menampung JSON config
    DynamicJsonDocument docSave(4096);
    String type = String(configType);

    // ========================================================
    // A. CONFIG NETWORK
    // ========================================================
    if (type == "network")
    {
      docSave["networkMode"] = networkSettings.networkMode;
      docSave["ssid"] = networkSettings.ssid;
      docSave["password"] = networkSettings.password;
      docSave["apSsid"] = networkSettings.apSsid;
      docSave["apPassword"] = networkSettings.apPassword;
      docSave["dhcpMode"] = networkSettings.dhcpMode;
      docSave["ipAddress"] = networkSettings.ipAddress;
      docSave["subnet"] = networkSettings.subnetMask;
      docSave["ipGateway"] = networkSettings.ipGateway;
      docSave["ipDNS"] = networkSettings.ipDNS;
      docSave["sendInterval"] = networkSettings.sendInterval;
      docSave["protocolMode"] = networkSettings.protocolMode;
      docSave["endpoint"] = networkSettings.endpoint;
      docSave["port"] = networkSettings.port;
      docSave["pubTopic"] = networkSettings.pubTopic;
      docSave["subTopic"] = networkSettings.subTopic;
      docSave["mqttUsername"] = networkSettings.mqttUsername;
      docSave["mqttPass"] = networkSettings.mqttPassword;
      docSave["loggerMode"] = networkSettings.loggerMode;
      docSave["protocolMode2"] = networkSettings.protocolMode2;
      docSave["modbusMode"] = modbusParam.mode;
      docSave["modbusPort"] = modbusParam.port;
      docSave["modbusSlaveID"] = modbusParam.slaveID;
      docSave["sendTrig"] = networkSettings.sendTrig;
      docSave["erpUrl"] = networkSettings.erpUrl;
      docSave["erpUsername"] = networkSettings.erpUsername;
      docSave["erpPassword"] = networkSettings.erpPassword;
    }
    // ========================================================
    // B. CONFIG DIGITAL
    // ========================================================
    else if (type == "digital")
    {
      for (unsigned char i = 1; i <= jumlahInputDigital; i++)
      {
        // Membuat object seperti "DI1": { ... }
        JsonObject diObj = docSave.createNestedObject("DI" + String(i));
        diObj["name"] = digitalInput[i].name;
        diObj["invers"] = digitalInput[i].inv;
        diObj["taskMode"] = digitalInput[i].taskMode;
        diObj["inputState"] = digitalInput[i].inputState ? "High" : "Low";
        diObj["intervalTime"] = digitalInput[i].intervalTime;
        diObj["conversionFactor"] = digitalInput[i].conversionFactor;
      }
    }
    // ========================================================
    // C. CONFIG ANALOG
    // ========================================================
    else if (type == "analog")
    {
      for (unsigned char i = 1; i <= jumlahInputAnalog; i++)
      {
        JsonObject aiObj = docSave.createNestedObject("AI" + String(i));
        aiObj["name"] = analogInput[i].name;
        aiObj["inputType"] = analogInput[i].inputType;
        aiObj["filter"] = analogInput[i].filter;
        aiObj["filterPeriod"] = analogInput[i].filterPeriod;
        aiObj["scaling"] = analogInput[i].scaling;
        aiObj["lowLimit"] = analogInput[i].lowLimit;
        aiObj["highLimit"] = analogInput[i].highLimit;
        aiObj["calibration"] = analogInput[i].calibration;
        aiObj["mValue"] = analogInput[i].mValue;
        aiObj["cValue"] = analogInput[i].cValue;
      }
    }
    // ========================================================
    // D. CONFIG MODBUS SETUP
    // ========================================================
    else if (type == "modbusSetup")
    {
      docSave = jsonParam;
    }
    // ========================================================
    // E. SYSTEM SETTINGS
    // ========================================================
    else if (type == "systemSettings")
    {
      docSave["username"] = networkSettings.loginUsername;
      docSave["password"] = networkSettings.loginPassword;
      docSave["sdInterval"] = networkSettings.sdSaveInterval;
    }

    // ========================================================
    // WRITE TO FILE (SPIFFS)
    // ========================================================
    if (!SPIFFS.begin(true))
    {
      xSemaphoreGive(jsonMutex);
      return;
    }
    File file = SPIFFS.open(dir, "w");
    if (!file)
    {
      Serial.printf("❌ Failed to open file for writing: %s\n", dir);
    }
    else
    {
      if (serializeJson(docSave, file) == 0)
      {
        Serial.println("❌ Failed to write JSON content");
      }
      else
      {
        Serial.printf("✅ Config Saved: %s\n", dir);
      }
      file.close();
    }

    // PENTING: Lepaskan Mutex setelah selesai!
    xSemaphoreGive(jsonMutex);
  }
  else
  {
    Serial.println("⚠️ Save skipped: jsonMutex busy");
  }
}

void saveToSDConfig(const char *dir, const char *configType)
{
  // Cek apakah SD Card siap (gunakan flag global jika ada, atau mutex)
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)))
  {
    Serial.printf("[SD] Saving config: %s ...\n", dir);

    // Hapus file lama agar data bersih (overwrite)
    if (SD.exists(dir))
    {
      SD.remove(dir);
    }

    DynamicJsonDocument docSD(4096);

    // 1. NETWORK CONFIG
    if (String(configType) == "network")
    {
      docSD["networkMode"] = networkSettings.networkMode;
      docSD["ssid"] = networkSettings.ssid;
      docSD["password"] = networkSettings.password;
      docSD["apSsid"] = networkSettings.apSsid;
      docSD["apPassword"] = networkSettings.apPassword;
      docSD["dhcpMode"] = networkSettings.dhcpMode;
      docSD["ipAddress"] = networkSettings.ipAddress;
      docSD["subnet"] = networkSettings.subnetMask;
      docSD["ipGateway"] = networkSettings.ipGateway;
      docSD["ipDNS"] = networkSettings.ipDNS;
      docSD["sendInterval"] = networkSettings.sendInterval;
      docSD["protocolMode"] = networkSettings.protocolMode;
      docSD["endpoint"] = networkSettings.endpoint;
      docSD["port"] = networkSettings.port;
      docSD["pubTopic"] = networkSettings.pubTopic;
      docSD["subTopic"] = networkSettings.subTopic;
      docSD["mqttUsername"] = networkSettings.mqttUsername;
      docSD["mqttPass"] = networkSettings.mqttPassword;
      docSD["loggerMode"] = networkSettings.loggerMode;
      docSD["protocolMode2"] = networkSettings.protocolMode2;
      docSD["modbusMode"] = modbusParam.mode;
      docSD["modbusPort"] = modbusParam.port;
      docSD["modbusSlaveID"] = modbusParam.slaveID;
      docSD["sendTrig"] = networkSettings.sendTrig;
      docSD["erpUrl"] = networkSettings.erpUrl;
      docSD["erpUsername"] = networkSettings.erpUsername;
      docSD["erpPassword"] = networkSettings.erpPassword;
    }
    // 2. DIGITAL INPUT CONFIG
    else if (String(configType) == "digital")
    {
      for (unsigned char i = 1; i < jumlahInputDigital + 1; i++)
      {
        docSD["DI" + String(i)]["name"] = digitalInput[i].name;
        docSD["DI" + String(i)]["invers"] = digitalInput[i].inv;
        docSD["DI" + String(i)]["taskMode"] = digitalInput[i].taskMode;
        docSD["DI" + String(i)]["inputState"] = digitalInput[i].inputState ? "High" : "Low";
        docSD["DI" + String(i)]["intervalTime"] = digitalInput[i].intervalTime;
        docSD["DI" + String(i)]["conversionFactor"] = digitalInput[i].conversionFactor;
      }
    }
    // 3. ANALOG INPUT CONFIG
    else if (String(configType) == "analog")
    {
      for (unsigned char i = 1; i < jumlahInputAnalog + 1; i++)
      {
        docSD["AI" + String(i)]["name"] = analogInput[i].name;
        docSD["AI" + String(i)]["inputType"] = analogInput[i].inputType;
        docSD["AI" + String(i)]["filter"] = analogInput[i].filter;
        docSD["AI" + String(i)]["filterPeriod"] = analogInput[i].filterPeriod;
        docSD["AI" + String(i)]["scaling"] = analogInput[i].scaling;
        docSD["AI" + String(i)]["lowLimit"] = analogInput[i].lowLimit;
        docSD["AI" + String(i)]["highLimit"] = analogInput[i].highLimit;
        docSD["AI" + String(i)]["calibration"] = analogInput[i].calibration;
        docSD["AI" + String(i)]["mValue"] = analogInput[i].mValue;
        docSD["AI" + String(i)]["cValue"] = analogInput[i].cValue;
      }
    }
    // 4. MODBUS SETUP (Menggunakan jsonParam global yang sudah diupdate)
    else if (String(configType) == "modbusSetup")
    {
      docSD = jsonParam;
    }
    // 5. SYSTEM SETTINGS
    else if (String(configType) == "systemSettings")
    {
      docSD["username"] = networkSettings.loginUsername;
      docSD["password"] = networkSettings.loginPassword;
      docSD["sdInterval"] = networkSettings.sdSaveInterval;
    }

    // Tulis ke SD Card
    File file = SD.open(dir, FILE_WRITE);
    if (file)
    {
      if (serializeJson(docSD, file) == 0)
      {
        Serial.println("  ✗ Failed to write config to SD");
      }
      else
      {
        Serial.println("  ✓ Config saved to SD Card");
      }
      file.close();
    }
    else
    {
      Serial.println("  ✗ Failed to open file on SD Card");
    }

    xSemaphoreGive(sdMutex);
  }
  else
  {
    Serial.println("  ⚠ SD Card Busy (Mutex), Config NOT saved to SD.");
  }
}

// ============================================================================
// FUNGSI DEBUG: TAMPILKAN DASHBOARD CONFIG LENGKAP
// ============================================================================
void printConfigurationDetails()
{
  Serial.println("               CURRENT SYSTEM CONFIGURATION                  ");
  Serial.println("[ NETWORK INFO ]");
  Serial.printf("  %-18s : %s\n", "Network Mode", networkSettings.networkMode.c_str());

  String currentIP = (networkSettings.networkMode == "Ethernet") ? Ethernet.localIP().toString() : networkSettings.ipAddress;
  String currentMAC = (networkSettings.networkMode == "Ethernet") ? networkSettings.macAddress : WiFi.macAddress();

  Serial.printf("  %-18s : %s\n", "IP Address", currentIP.c_str());
  Serial.printf("  %-18s : %s\n", "MAC Address", currentMAC.c_str());
  if (networkSettings.networkMode == "WiFi")
  {
    Serial.printf("  %-18s : %s\n", "SSID", networkSettings.ssid.c_str());
  }
  Serial.println();

  // 2. DATA LOGGER & PROTOCOL
  Serial.println("[ DATA LOGGER SETTINGS ]");
  String logStatus = (networkSettings.loggerMode == "Enabled") ? "ENABLED (ON)" : "DISABLED (OFF)";

  Serial.printf("  %-18s : %s\n", "Logger Mode", logStatus.c_str());
  Serial.printf("  %-18s : %s\n", "Protocol", networkSettings.protocolMode.c_str());
  Serial.printf("  %-18s : %.1f seconds\n", "Interval", networkSettings.sendInterval);

  if (networkSettings.protocolMode == "HTTP")
  {
    Serial.printf("  %-18s : %s\n", "HTTP URL", networkSettings.endpoint.c_str());
  }
  else
  {
    Serial.printf("  %-18s : %s\n", "MQTT Broker", networkSettings.endpoint.c_str());
    Serial.printf("  %-18s : %s\n", "Pub Topic", networkSettings.pubTopic.c_str());
  }

  // 2. DATA LOGGER & ERP
  Serial.println("\n[ DATA LOGGER & ERP ]");
  String modeLog = (networkSettings.loggerMode.length() > 0) ? networkSettings.loggerMode : "Disabled";
  Serial.printf("%-20s | %s\n", "Logger Mode", modeLog.c_str());
  Serial.printf("%-20s | %s\n", "Protocol", networkSettings.protocolMode.c_str());
  Serial.printf("%-20s | %.2f sec\n", "Send Interval", networkSettings.sendInterval);
  Serial.printf("%-20s | %s\n", "ERP URL", networkSettings.erpUrl.c_str()); // --------------------------------------------------------------------------
  // 2. DIGITAL INPUT CONFIG
  // --------------------------------------------------------------------------
  Serial.println("\n[ DIGITAL INPUT CONFIG ]");
  Serial.println("ID | Name                | Mode         | Inverted | Active State");
  for (int i = 1; i <= jumlahInputDigital; i++)
  {
    String name = (digitalInput[i].name.length() > 0) ? digitalInput[i].name : "-";
    Serial.printf("D%d | %-19s | %-12s | %-8s | %s\n",
                  i,
                  name.c_str(),
                  digitalInput[i].taskMode.c_str(),
                  digitalInput[i].inv ? "YES" : "NO",
                  digitalInput[i].inputState ? "HIGH" : "LOW");
  }

  // --------------------------------------------------------------------------
  // 3. ANALOG INPUT CONFIG
  // --------------------------------------------------------------------------
  Serial.println("\n[ ANALOG INPUT CONFIG ]");
  Serial.println("ID | Name                | Type      | Scaling (Low - High) | Calibration (m/c)");
  for (int i = 1; i <= jumlahInputAnalog; i++)
  {
    String name = (analogInput[i].name.length() > 0) ? analogInput[i].name : "-";
    String scaleStr = String(analogInput[i].lowLimit) + " - " + String(analogInput[i].highLimit);
    String calStr = "m:" + String(analogInput[i].mValue) + " c:" + String(analogInput[i].cValue);

    Serial.printf("A%d | %-19s | %-9s | %-20s | %s\n",
                  i,
                  name.c_str(),
                  analogInput[i].inputType.c_str(),
                  analogInput[i].scaling ? scaleStr.c_str() : "OFF",
                  analogInput[i].calibration ? calStr.c_str() : "OFF");
  }

  // --------------------------------------------------------------------------
  // 4. MODBUS CONFIG (Jika ada)
  // --------------------------------------------------------------------------
  if (stringParam.length() > 5)
  {
    Serial.println("\n[ MODBUS RTU CONFIG ]");
    Serial.printf("Baudrate : %d\n", modbusParam.baudrate);
    Serial.printf("Parity   : %s\n", modbusParam.parity.c_str());
    Serial.printf("Scan Rate: %.1f sec\n", modbusParam.scanRate);

    Serial.println("\n[ MODBUS POLLING LIST ]");
    DynamicJsonDocument docTemp(4096);
    DeserializationError error = deserializeJson(docTemp, stringParam);

    if (!error)
    {
      JsonArray nameData = docTemp["nameData"];
      int count = 0;
      for (JsonVariant v : nameData)
      {
        String name = v.as<String>();
        JsonArray params = docTemp[name];
        if (!params.isNull())
        {
          Serial.printf(" %d. %-15s -> ID:%d | FC:%d | Reg:%d | Mul:%.2f\n",
                        ++count, name.c_str(),
                        params[0].as<int>(), params[1].as<int>(),
                        params[2].as<int>(), params[3].as<float>());
        }
      }
    }
  }
}
