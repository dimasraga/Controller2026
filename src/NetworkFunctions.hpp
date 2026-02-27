#ifndef NETWORK_FUNCTIONS_HPP
#define NETWORK_FUNCTIONS_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <DNSServer.h>
#include "config.hpp"
#include <Ethernet.h>
#include "certs.h"
#include "MbedTLSHandler.hpp"
bool httpRequestInProgress = false;
unsigned long httpRequestStartTime = 0;
const unsigned long HTTP_REQUEST_TIMEOUT = 6000;

String getDomainFromUrl(String url)
{
  int index = url.indexOf("://");
  String domain = (index != -1) ? url.substring(index + 3) : url;
  int slash = domain.indexOf("/");
  if (slash != -1)
    domain = domain.substring(0, slash);
  return domain;
}

String getPathFromUrl(String url)
{
  int index = url.indexOf("://");
  String rest = (index != -1) ? url.substring(index + 3) : url;
  int slash = rest.indexOf("/");
  if (slash != -1)
    return rest.substring(slash);
  return "/";
}

class MyEthernetServer : public EthernetServer
{
public:
  MyEthernetServer(uint16_t port) : EthernetServer(port) {}
  void begin(uint16_t port = 0) override
  {
    EthernetServer::begin();
  }
};

// Forward declarations
extern PubSubClient mqtt;
extern WiFiClient esp32;
// EthernetServer ethServer(80);
MyEthernetServer ethServer(80);
extern DNSServer dnsServer;
extern const byte DNS_PORT;
extern bool dnsStarted;
extern unsigned long checkTime;
extern unsigned long sendTime;
extern bool flagGetJobNum;
extern String jobNum;
extern ErrorBlinker errorBlinker;
extern ErrorMessages errorMessages;
extern String getTimeNow();
extern String getTimeDateNow();

bool wifiConnected = false;
bool wifiConnecting = false;
bool apReady = false;
unsigned long wifiConnectStartTime = 0;
const unsigned long WIFI_CONNECT_TIMEOUT = 10000; // 10 detik timeout (sebelumnya 30000)
bool staConnectionAttemptFailed = false;          // Flag untuk berhenti mencoba koneksi STA

// Function declarations
IpAddressSplit parsingIP(String data);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void checkWiFi(int timeout);
void get_JobNum();
void startDNSServer();
void stopDNSServer();
void configNetwork();
void configProtocol();
void sendDataMQTT(String dataSend, String publishTopic, int intervalSend);
void sendDataHTTP(String data, String serverPath, String httpUsername, String httpPassword, int intervalSend);
void saveToSD(String data);
void sendBackupData();

// Implementation
IpAddressSplit parsingIP(String data)
{
  IpAddressSplit result;
  int start = 0, j = 0;
  int len = data.length();
  for (int i = 0; i <= len; i++)
  {
    if (i == len || data.charAt(i) == '.' || data.charAt(i) == ',')
    {
      if (j < 5)
      {
        result.ip[j] = data.substring(start, i).toInt();
        j++;
      }
      start = i + 1;
    }
  }
  return result;
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String command(reinterpret_cast<const char *>(payload), length);
  StaticJsonDocument<200> commandJson;
  if (deserializeJson(commandJson, command))
    return;

  for (byte i = 1; i < jumlahOutputDigital + 1; i++)
    if (commandJson.containsKey(digitalOutput[i].name))
      digitalWrite(2, commandJson[digitalOutput[i].name].as<bool>());
}

void checkWiFi(int timeout)
{
  if (networkSettings.networkMode == "WiFi" || (networkSettings.networkMode == "Ethernet" && networkSettings.ssid.length() > 1))
  {
    if (networkSettings.macAddress.length() < 10)
      networkSettings.macAddress = WiFi.macAddress();
    if (millis() - checkTime >= timeout)
    {
      wl_status_t status = WiFi.status();
      if (status != WL_CONNECTED && !wifiConnecting)
      {
        if (networkSettings.networkMode == "WiFi")
          errorBlinker.trigger(5, 500);
        Serial.println("[WiFi Check] Status: " + String(status) + " (Disconnected). Reconnecting...");
        wifiConnecting = true;
        wifiConnectStartTime = millis();
        WiFi.begin(networkSettings.ssid.c_str(), networkSettings.password.c_str());
      }

      if (wifiConnecting && (millis() - wifiConnectStartTime > WIFI_CONNECT_TIMEOUT))
      {
        Serial.println("Warning: WiFi connection timeout.");
        wifiConnecting = false;
        WiFi.disconnect(true);
        delay(500);
        staConnectionAttemptFailed = false;
      }
      if (status == WL_CONNECTED)
      {
        if (!wifiConnected || wifiConnecting)
        {
          Serial.printf("[WiFi] Connected | IP: %s | RSSI: %d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
          wifiConnected = true;
          wifiConnecting = false;
          staConnectionAttemptFailed = false;
          WiFi.setAutoReconnect(true);

          if (flagGetJobNum && networkSettings.networkMode == "WiFi")
          {
// Ambil JobNum hanya jika WiFi adalah koneksi utama (untuk menghindari bentrok request)
#ifndef DEBUG
            get_JobNum();
#endif
          }
        }
      }
      else
      {
        wifiConnected = false;
      }
      checkTime = millis();
    }
    if (apReady && millis() % 30000 < 100)
    {
      Serial.printf("[AP Status] Clients: %d | IP: %s\n", WiFi.softAPgetStationNum(), WiFi.softAPIP().toString().c_str());
    }
  }
}

void get_JobNum()
{
  bool ethReady = (networkSettings.networkMode == "Ethernet") && (Ethernet.linkStatus() == LinkON);
  bool wifiReady = (WiFi.status() == WL_CONNECTED);
  if (!ethReady && !wifiReady)
    return;

  if (ethReady)
  {
    EthernetClient ethClient;
    ethClient.setTimeout(5000);
    int result = perform_https_request_mbedtls(
        ethClient,
        getDomainFromUrl(networkSettings.erpUrl).c_str(),
        getPathFromUrl(networkSettings.erpUrl).c_str(),
        "", networkSettings.erpUsername.c_str(), networkSettings.erpPassword.c_str());
    if (result == 200)
    {
      flagGetJobNum = 0;
    }
    return;
  }

  WiFiClientSecure client;
  HTTPClient http;
  client.setInsecure();
  http.begin(client, networkSettings.erpUrl.c_str());
  http.setAuthorization(networkSettings.erpUsername.c_str(), networkSettings.erpPassword.c_str());
  http.setTimeout(5000);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0)
  {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, http.getStream());

    if (!error)
    {
      jobNum = doc["value"][0]["LaborDtl_JobNum"].as<String>();
      Serial.println("[ERP] Job Number: " + jobNum);
      flagGetJobNum = 0;
    }
    else
    {
      Serial.printf("[ERP] JSON Parse failed: %s\n", error.c_str());
    }
  }
  else
  {
    Serial.printf("[ERP] Request Failed: %d\n", httpResponseCode);
  }
  http.end();
}

void startDNSServer()
{
  if (!dnsStarted && apReady)
  {
    delay(500);
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    dnsStarted = true;
    ESP_LOGI("DNS", "DNS started: %s", WiFi.softAPIP().toString().c_str());
  }
}

void stopDNSServer()
{
  if (dnsStarted)
  {
    dnsServer.stop();
    dnsStarted = false;
    ESP_LOGI("DNS", "DNS server stopped");
  }
}

void configNetwork()
{
  if (networkSettings.networkMode == "WiFi")
  {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);
    WiFi.mode(WIFI_AP_STA);
    delay(500);
    esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, mac);
    ESP_LOGI("WiFi", "Init AP+STA | MAC set: %s", err == ESP_OK ? "OK" : "FAIL");
    delay(200);
    WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t i)
                 { wifiConnecting = false; wifiConnected = true; }, ARDUINO_EVENT_WIFI_STA_CONNECTED);
    WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t i)
                 { wifiConnected = false; }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t i)
                 { ESP_LOGI("WiFi", "IP:%s", WiFi.localIP().toString().c_str()); }, ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.softAPConfig(IPAddress(10, 22, 7, 3), IPAddress(10, 22, 7, 1), IPAddress(255, 255, 255, 0)); // Set custom AP IP
    static const uint8_t AP_CHANNEL = 6;
    bool apStarted = WiFi.softAP(
        networkSettings.apSsid.c_str(),     // SSID
        networkSettings.apPassword.c_str(), // Password
        AP_CHANNEL,                         // Channel (fixed ke 6)
        0,                                  // SSID Hidden (0=visible, 1=hidden)
        4                                   // Max connections
    );
    if (apStarted)
    {
      unsigned long apWait = millis();
      while (millis() - apWait < 1000)
      {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0))
      {
        Serial.println("  ✗ AP Failed - IP is 0.0.0.0");
        Serial.println("  → Retrying AP start...");
        WiFi.softAPdisconnect(true);
        delay(500);
        apStarted = WiFi.softAP(networkSettings.apSsid.c_str(),
                                networkSettings.apPassword.c_str(), AP_CHANNEL, 0, 4);
        delay(1000);
      }
      if (WiFi.softAPIP() != IPAddress(0, 0, 0, 0))
      {
        apReady = true;
        Serial.println("  ✓ AP Started Successfully!");
        Serial.printf("    AP IP: %s\n", WiFi.softAPIP().toString().c_str());
        Serial.printf("    AP MAC: %s\n", WiFi.softAPmacAddress().c_str());
        startDNSServer();
      }
      else
      {
        Serial.println("  ✗ AP Failed to start properly");
        apReady = false;
      }
    }
    else
    {
      Serial.println("  ✗ Failed to start AP");
      apReady = false;
    }
    // STEP 6: Configure Station (STA) IP Settings
    Serial.println("[6/7] Configuring STA IP settings...");
    if (networkSettings.dhcpMode == "DHCP")
    {
      Serial.println("  Using DHCP");
      WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    }
    else
    {
      Serial.println("  Using Static IP");
      IpAddressSplit hasilParsing;
      hasilParsing = parsingIP(networkSettings.ipAddress);
      IPAddress localIP(hasilParsing.ip[0], hasilParsing.ip[1], hasilParsing.ip[2], hasilParsing.ip[3]);
      hasilParsing = parsingIP(networkSettings.subnetMask);
      IPAddress subnetIP(hasilParsing.ip[0], hasilParsing.ip[1], hasilParsing.ip[2], hasilParsing.ip[3]);
      hasilParsing = parsingIP(networkSettings.ipGateway);
      IPAddress gatewayIP(hasilParsing.ip[0], hasilParsing.ip[1], hasilParsing.ip[2], hasilParsing.ip[3]);
      hasilParsing = parsingIP(networkSettings.ipDNS);
      IPAddress dnsIP(hasilParsing.ip[0], hasilParsing.ip[1], hasilParsing.ip[2], hasilParsing.ip[3]);
      if (WiFi.config(localIP, gatewayIP, subnetIP, dnsIP))
      {
        Serial.println("  ✓ Static IP configured");
        Serial.printf("    IP: %s\n", localIP.toString().c_str());
        Serial.printf("    Gateway: %s\n", gatewayIP.toString().c_str());
      }
      else
      {
        Serial.println("  ✗ Failed to configure static IP");
      }
    }

    // =====================================================================
    // STEP 7: Connect to WiFi Network (STA)
    // =====================================================================
    Serial.println("[7/7] Connecting to WiFi network...");
    Serial.printf("  Target SSID: %s\n", networkSettings.ssid.c_str());

    mqtt = PubSubClient(esp32);
    checkTime = millis();

    // LOGIKA BARU: Jangan auto-reconnect di awal!
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    wifiConnecting = true;
    wifiConnectStartTime = millis();
    WiFi.begin(networkSettings.ssid.c_str(), networkSettings.password.c_str());

    networkSettings.macAddress = WiFi.macAddress();
    Serial.printf("[WiFi] AP:%s(%s) ready=%d | STA->%s connecting\n",
                  networkSettings.apSsid.c_str(), WiFi.softAPIP().toString().c_str(),
                  apReady, networkSettings.ssid.c_str());
  }

  if (networkSettings.networkMode == "Ethernet")
  {
    Serial.println("CONFIGURING ETHERNET (W5500)");

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);
    WiFi.mode(WIFI_AP_STA);
    delay(500);

    // Start Access Point untuk konfigurasi
    Serial.println("[1/5] Starting Access Point...");
    WiFi.softAPConfig(IPAddress(10, 22, 7, 3), IPAddress(10, 22, 7, 1), IPAddress(255, 255, 255, 0)); // Set custom AP IP
    static const uint8_t AP_CHANNEL = 6;
    bool apStarted = WiFi.softAP(networkSettings.apSsid.c_str(),
                                 networkSettings.apPassword.c_str(), AP_CHANNEL, 0, 4);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);
    WiFi.mode(WIFI_AP_STA);
    delay(500);
    // STA selalu aktif di mode Ethernet, jika ada SSID langsung connect
    esp_wifi_set_mac(WIFI_IF_STA, mac);
    staConnectionAttemptFailed = false;
    wifiConnected = false;
    if (networkSettings.ssid.length() > 1)
    {
      Serial.printf("  ✓ Starting WiFi STA to: %s\n", networkSettings.ssid.c_str());
      WiFi.setAutoReconnect(true);
      WiFi.persistent(false);
      // Konfigurasi IP statis khusus untuk interface WiFi STA
      WiFi.config(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
      WiFi.begin(networkSettings.ssid.c_str(), networkSettings.password.c_str());
      wifiConnecting = true;
      wifiConnectStartTime = millis();
    }
    else
    {
      Serial.println("  ⚠ No SSID configured, WiFi STA idle (AP still active)");
    }
    // =====================================================================
    // STEP 2: Hardware Reset W5500
    // =====================================================================
    // Serial.println("[2/5] Resetting W5500 chip...");
    pinMode(ETH_RST, OUTPUT);
    digitalWrite(ETH_RST, LOW);
    delay(100);
    digitalWrite(ETH_RST, HIGH);
    delay(200);
    // Serial.println("  ✓ W5500 reset complete");

    Ethernet.init(ETH_CS);
    Serial.printf("  ✓ SPI initialized (CLK:%d, MISO:%d, MOSI:%d, CS:%d)\n",
                  ETH_CLK, ETH_MISO, ETH_MOSI, ETH_CS);

    // =====================================================================
    // STEP 4: Initialize Ethernet with W5500
    // =====================================================================
    Serial.println("[4/5] Initializing Ethernet...");
    Ethernet.init(ETH_CS); // Set CS pin untuk W5500
    // ip static
    // IPAddress localIP(10, 22, 7, 3);      // IP Requested
    // IPAddress gatewayIP(10, 22, 7, 1);    // Asumsi Gateway default
    // IPAddress subnetIP(255, 255, 255, 0); // Subnet standard
    // IPAddress dnsIP(8, 8, 8, 8);          // DNS Google

    // // Update variable global agar tampilan di web/serial benar
    // networkSettings.ipAddress = localIP.toString();
    // networkSettings.ipGateway = gatewayIP.toString();
    // networkSettings.subnetMask = subnetIP.toString();
    // networkSettings.ipDNS = dnsIP.toString();
    // networkSettings.dhcpMode = "Static (Forced)";

    // Serial.println("  [FORCE] Starting Ethernet with Static IP: 10.22.7.3 ...");

    // // Syntax: begin(mac, ip, dns, gateway, subnet)
    // Ethernet.begin(mac, localIP, dnsIP, gatewayIP, subnetIP);
    // delay(1000); // Beri waktu untuk inisialisasi PHY

    // Serial.println("  ✓ Ethernet configured manually");
    // ini dhcp
    // Parse IP addresses
    IpAddressSplit hasilParsing;
    hasilParsing = parsingIP(networkSettings.ipAddress);
    IPAddress localIP(hasilParsing.ip[0], hasilParsing.ip[1],
                      hasilParsing.ip[2], hasilParsing.ip[3]);

    hasilParsing = parsingIP(networkSettings.subnetMask);
    IPAddress subnetIP(hasilParsing.ip[0], hasilParsing.ip[1],
                       hasilParsing.ip[2], hasilParsing.ip[3]);

    hasilParsing = parsingIP(networkSettings.ipGateway);
    IPAddress gatewayIP(hasilParsing.ip[0], hasilParsing.ip[1],
                        hasilParsing.ip[2], hasilParsing.ip[3]);

    hasilParsing = parsingIP(networkSettings.ipDNS);
    IPAddress dnsIP(hasilParsing.ip[0], hasilParsing.ip[1],
                    hasilParsing.ip[2], hasilParsing.ip[3]);

    if (networkSettings.dhcpMode == "DHCP")
    {
      Serial.println("  Starting Ethernet with DHCP...");
      if (Ethernet.begin(mac, 10000, 4000) == 0)
      {
        ESP_LOGE("Ethernet", "  ✗ Failed to configure Ethernet using DHCP");
        errorMessages.addMessage(getTimeNow() + " - Failed to start Ethernet (DHCP)");
      }
      else
      {
        Serial.println("  ✓ Ethernet started with DHCP");
        Serial.printf("    IP: %s\n", Ethernet.localIP().toString().c_str());
      }
    }
    else
    {
      Serial.println("  Starting Ethernet with Static IP...");
      Ethernet.begin(mac, localIP, dnsIP, gatewayIP, subnetIP);
      delay(1000);
      Serial.println("  ✓ Ethernet configured with static IP");
    }
    // */ //batas dhcp dikomen saja ini
    // =====================================================================
    // STEP 5: Verify Ethernet Hardware & Link Status
    // =====================================================================
    Serial.println("[5/5] Verifying Ethernet connection...");

    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
      ESP_LOGE("Ethernet", "  ✗ W5500 chip not found!");
      errorMessages.addMessage(getTimeNow() + " - W5500 hardware not found");
      errorBlinker.trigger(5, 200);
    }
    else
    {
      Serial.println("  ✓ W5500 chip detected");

      if (Ethernet.linkStatus() == LinkOFF)
      {
        ESP_LOGW("Ethernet", "  ⚠ Ethernet cable not connected");
        errorMessages.addMessage(getTimeNow() + " - Ethernet cable disconnected");
      }
      else if (Ethernet.linkStatus() == LinkON)
      {
        Serial.println("  ✓ Ethernet cable connected");
      }
    }

    // Set MAC Address string
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    networkSettings.macAddress = macStr;
    ethServer.begin();
    Serial.printf("[ETH] Done | MAC:%s IP:%s Link:%s\n  ETH:http://%s AP:http://%s\n",
                  networkSettings.macAddress.c_str(), Ethernet.localIP().toString().c_str(),
                  Ethernet.linkStatus() == LinkON ? "UP" : "DOWN",
                  Ethernet.localIP().toString().c_str(), WiFi.softAPIP().toString().c_str());
  }
}

void configProtocol()
{
  if (networkSettings.protocolMode == "MQTT")
  {
    mqtt.setServer(networkSettings.endpoint.c_str(), networkSettings.port);
    mqtt.setCallback(mqttCallback);
  }
}

void sendDataMQTT(String dataSend, String publishTopic, int intervalSend)
{
  if (millis() - sendTime >= (intervalSend * 1000))
  {
    bool canSendMqtt = (networkSettings.networkMode == "Ethernet")
                           ? (Ethernet.linkStatus() == LinkON || WiFi.status() == WL_CONNECTED)
                           : (WiFi.status() == WL_CONNECTED);
    if (!canSendMqtt)
    {
      sendTime = millis();
      return;
    }
    if (mqtt.publish(publishTopic.c_str(), dataSend.c_str()))
    {
      Serial.println("MQTT data sent successfully");
    }
    else
    {
      Serial.println("MQTT publish failed");
    }
    sendTime = millis();
  }
}

void sendDataHTTP(String data, String serverPath, String httpUsername, String httpPassword, int intervalSend)
{
  if (intervalSend > 0 && millis() - sendTime < (unsigned long)(intervalSend * 1000))
    return;

  if (httpRequestInProgress)
  {
    if (millis() - httpRequestStartTime > HTTP_REQUEST_TIMEOUT)
    {
      Serial.println("[HTTP] Request timeout - resetting flag");
      httpRequestInProgress = false;
    }
    else
    {
      return;
    }
  }

  httpRequestInProgress = true;
  httpRequestStartTime = millis();

  // bool success = false;
  // bool isEthMode = (networkSettings.networkMode == "Ethernet");
  // bool isEthReady = isEthMode && (Ethernet.linkStatus() == LinkON);
  // bool isWifiReady = (!isEthMode) && (WiFi.status() == WL_CONNECTED);

  bool success = false;
  // Fallback to WiFi STA if Ethernet link drops by removing the !isEthMode constraint
  bool isEthReady = (networkSettings.networkMode == "Ethernet") && (Ethernet.linkStatus() == LinkON);
  bool isWifiReady = (WiFi.status() == WL_CONNECTED);

  // 1. Eksekusi Berdasarkan Mode
  if (isEthReady)
  {
    EthernetClient ethClient;
    ethClient.setTimeout(HTTP_REQUEST_TIMEOUT);

    int result = perform_https_request_mbedtls(
        ethClient,
        getDomainFromUrl(serverPath).c_str(),
        getPathFromUrl(serverPath).c_str(),
        data.c_str(),
        httpUsername.c_str(),
        httpPassword.c_str());

    if (result == 200 || result == 0)
    {
      success = true;
      Serial.printf("[HTTP] ETH Success (%lums)\n", millis() - httpRequestStartTime);
    }
    else
    {
      Serial.printf("[HTTP] ETH Failed (Code: %d)\n", result);
    }
  }
  else if (isWifiReady)
  {
    WiFiClientSecure client;
    HTTPClient https;
    client.setInsecure();

    if (https.begin(client, serverPath))
    {
      https.setAuthorization(httpUsername.c_str(), httpPassword.c_str());
      https.addHeader("Content-Type", "application/json");
      https.setTimeout(HTTP_REQUEST_TIMEOUT);

      int httpResponseCode = https.POST(data);

      if (httpResponseCode == 200 || httpResponseCode == 201)
      {
        success = true;
        Serial.printf("[HTTP] WiFi Success (%lums)\n", millis() - httpRequestStartTime);
      }
      else
      {
        Serial.printf("[HTTP] WiFi Failed: %s\n", https.errorToString(httpResponseCode).c_str());
      }
      https.end();
    }
  }
  else
  {
    Serial.println("[HTTP] No Active Connection");
  }

  // 2. Handle Hasil (Update Status & SD Card)
  if (success)
  {
    networkSettings.connStatus = "Connected";
  }
  else
  {
    networkSettings.connStatus = "Not Connected";
    Serial.println("[HTTP] Saving to SD Card...");
    saveToSD(data);
  }

  httpRequestInProgress = false;
  sendTime = millis();
  vTaskDelay(pdMS_TO_TICKS(1));
}

void saveToSD(String data)
{
  Serial.println("Saving to SD card...");
  File dataOffline = SD.open("/sensor_data.csv", FILE_APPEND);
  if (!dataOffline)
  {
    errorBlinker.trigger(3, 200);
    ESP_LOGE("SD Card", "Failed to open file!");
    errorMessages.addMessage(getTimeNow() + " - Failed to open file for writing!");
    return;
  }

  dataOffline.println(data);
  dataOffline.close();
  ESP_LOGI("SD Card", "✓ Data saved");
}
// Helper untuk mengirim chunk data backup ke server
static bool sendBackupChunk(const String &payload)
{
  if (networkSettings.networkMode == "Ethernet" && Ethernet.linkStatus() == LinkON)
  {
    EthernetClient ethClient;
    ethClient.setTimeout(5000);
    String serverPath = "https://api-logger-dev2.medionindonesia.com/api/v1/UpdateLoggingRealtime";
    int result = perform_https_request_mbedtls(ethClient, getDomainFromUrl(serverPath).c_str(), getPathFromUrl(serverPath).c_str(), payload.c_str(), networkSettings.mqttUsername.c_str(), networkSettings.mqttPassword.c_str());
    return (result == 200 || result == 0);
  }
  else if (WiFi.status() == WL_CONNECTED)
  {
    WiFiClientSecure client;
    HTTPClient https;
    client.setInsecure();
    if (https.begin(client, "https://api-logger-dev2.medionindonesia.com/api/v1/UpdateLoggingRealtime"))
    {
      https.setAuthorization(networkSettings.mqttUsername.c_str(), networkSettings.mqttPassword.c_str());
      https.addHeader("Content-Type", "application/json");
      https.setTimeout(5000);
      int httpCode = https.POST(payload);
      https.end();
      return (httpCode == 200 || httpCode == 201);
    }
  }
  return false;
}
void sendBackupData()
{
  if (!SD.exists("/sensor_data.csv"))
    return;
  File dataOffline = SD.open("/sensor_data.csv", FILE_READ);
  if (!dataOffline)
  {
    ESP_LOGE("SD", "Failed to open backup file");
    return;
  }

  size_t fileSize = dataOffline.size();
  if (fileSize < 10)
  {
    dataOffline.close();
    SD.remove("/sensor_data.csv");
    return;
  }

  int successCount = 0;
  int failCount = 0;
  int count = 0;

  // Gunakan char buffer statis untuk menghemat Heap
  char lineBuffer[512];
  String payload = "";
  payload.reserve(3000); // Pesan tempat sekali saja di awal
  payload = "[";

  while (dataOffline.available())
  {
    // Membaca langsung ke buffer C-string (Sangat cepat dan hemat RAM)
    int bytesRead = dataOffline.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer) - 1);
    lineBuffer[bytesRead] = '\0'; // Null-terminator

    // Trim in-place pada buffer C
    int start = 0, end = bytesRead - 1;
    while (start <= end && isspace((uint8_t)lineBuffer[start]))
      start++;
    while (end >= start && isspace((uint8_t)lineBuffer[end]))
      end--;
    if (end - start < 4)
      continue;
    if (lineBuffer[start] == '[')
      start++;
    if (lineBuffer[end] == ']')
      end--;
    lineBuffer[end + 1] = '\0';
    if (count > 0)
      payload += ',';
    payload += (lineBuffer + start);
    count++;

    // Kirim setiap 10 data
    if (count >= 10)
    {
      payload += "]";

      if (sendBackupChunk(payload)) // Eksekusi dengan fungsi helper
      {
        successCount++;
        networkSettings.connStatus = "Connected";
      }
      else
      {
        failCount++;
      }

      // Reset Buffer untuk chunk berikutnya
      payload = "[";
      count = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  if (count > 0)
  {
    payload += "]";

    if (sendBackupChunk(payload)) // Eksekusi sisa data dengan fungsi helper
    {
      successCount++;
      networkSettings.connStatus = "Connected";
    }
    else
    {
      failCount++;
    }
  }
  dataOffline.close();
  Serial.printf("[Backup] Summary: %d Chunks OK, %d Failed\n", successCount, failCount);

  if (failCount == 0 && successCount > 0)
  {
    SD.remove("/sensor_data.csv");
  }
}
long measureLatency(String url)
{
  String domain = getDomainFromUrl(url);
  if (domain.length() == 0)
    domain = url;

  int port = (url.indexOf("https://") >= 0) ? 443 : 80;
  unsigned long start = millis();
  bool connected = false;

  if (networkSettings.networkMode == "Ethernet")
  {
    EthernetClient client;
    client.setTimeout(2000);
    if (client.connect(domain.c_str(), port))
    {
      connected = true;
      client.stop();
    }
  }
  else
  {
    WiFiClient client;
    client.setTimeout(2000);
    if (client.connect(domain.c_str(), port))
    {
      connected = true;
      client.stop();
    }
  }
  return connected ? (millis() - start) : -1;
}
#endif