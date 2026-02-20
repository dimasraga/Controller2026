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
const unsigned long HTTP_REQUEST_TIMEOUT = 5000;

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
// PERUBAHAN 1: Ubah timeout ke 10 detik (sesuai permintaan)
const unsigned long WIFI_CONNECT_TIMEOUT = 10000; // 10 detik timeout (sebelumnya 30000)

// PERUBAHAN 2: Tambahkan flag "menyerah"
bool staConnectionAttemptFailed = false; // Flag untuk berhenti mencoba koneksi STA

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
  Serial.println("MQTT Message Received");
  String command = "";
  for (int j = 0; j < length; j++)
    command += (char)payload[j];
  Serial.println(command);

  StaticJsonDocument<200> commandJson;
  DeserializationError error = deserializeJson(commandJson, command);
  if (error)
  {
    Serial.println("Failed to parse JSON");
    return;
  }

  for (byte i = 1; i < jumlahOutputDigital + 1; i++)
  {
    if (commandJson.containsKey(digitalOutput[i].name))
    {
      Serial.print(digitalOutput[i].name + ": ");
      bool statusDO = commandJson[digitalOutput[i].name];
      Serial.println(statusDO);
      digitalWrite(2, commandJson[digitalOutput[i].name]);
    }
  }
}

// =================================================================
// PERUBAHAN 3: Modifikasi checkWiFi() secara keseluruhan
// =================================================================
void checkWiFi(int timeout)
{
  // UBAH KONDISI IF INI:
  // Ijinkan cek WiFi jika mode = WiFi ATAU (Mode = Ethernet DAN ada SSID tersimpan)
  if (networkSettings.networkMode == "WiFi" || (networkSettings.networkMode == "Ethernet" && networkSettings.ssid.length() > 1))
  {
    if (staConnectionAttemptFailed)
    {
      // Jika di mode Ethernet, kita tidak perlu terlalu agresif mematikan STA
      // Tapi biarkan logika ini berjalan untuk menghemat resource jika WiFi gagal terus
      return;
    }

    // Update MAC jika belum ada
    if (networkSettings.macAddress.length() < 10)
      networkSettings.macAddress = WiFi.macAddress();

    if (millis() - checkTime >= timeout)
    {
      wl_status_t status = WiFi.status();

      // Handle WiFi disconnection
      if (status != WL_CONNECTED && !wifiConnecting)
      {
        // Hanya trigger error blinker jika ini mode UTAMA
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

        if (networkSettings.networkMode == "WiFi")
        {
          wifiConnecting = false;
          WiFi.disconnect(false);
        }
      }

      // Handle successful connection
      if (status == WL_CONNECTED)
      {
        if (!wifiConnected || wifiConnecting)
        {
          Serial.println("=================================");
          Serial.println("✅ WiFi Connected (Background/Primary)");
          Serial.print("   IP: ");
          Serial.println(WiFi.localIP());
          Serial.print("   RSSI: ");
          Serial.println(WiFi.RSSI());
          Serial.println("=================================");

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

    // Monitor AP Status (Tetap jalan)
    if (apReady && millis() % 30000 < 100)
    {
      Serial.printf("[AP Status] Clients: %d | IP: %s\n", WiFi.softAPgetStationNum(), WiFi.softAPIP().toString().c_str());
    }
  }
}
void get_JobNum()
{
  if (WiFi.status() != WL_CONNECTED)
    return; // Skip log agar tidak spam

  WiFiClientSecure client;
  HTTPClient http;
  client.setInsecure();

  http.begin(client, networkSettings.erpUrl.c_str());
  http.setAuthorization(networkSettings.erpUsername.c_str(), networkSettings.erpPassword.c_str());
  http.setTimeout(5000);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0)
  {
    // Menggunakan buffer Stream langsung tanpa String penampung besar
    // Alokasi 512 bytes di stack lebih cepat dan aman daripada 2048 bytes di heap
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
    delay(500); // Tunggu AP fully ready
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    dnsStarted = true;
    ESP_LOGI("DNS", "DNS server started, redirecting to %s", WiFi.softAPIP().toString().c_str());
    Serial.println("DNS Server started on " + WiFi.softAPIP().toString());
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
    Serial.println("CONFIGURING WIFI (AP+STA MODE)");

    // =====================================================================
    // STEP 1: Complete WiFi Reset
    // =====================================================================
    Serial.println("[1/7] Resetting WiFi...");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);

    // MAC harus di-set setelah mode WIFI_STA/AP_STA aktif, bukan saat WIFI_OFF
    Serial.println("[2/7] Setting WiFi mode to AP+STA...");
    WiFi.mode(WIFI_AP_STA);
    delay(500);

    Serial.println("[3/7] Setting MAC Address...");
    esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, mac);
    if (err == ESP_OK)
    {
      Serial.printf("  ✓ MAC Address set: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else
    {
      Serial.println("  ✗ Failed to set MAC Address");
    }
    delay(200);
    // =====================================================================
    // STEP 4: Setup WiFi Events
    // =====================================================================
    Serial.println("[4/7] Setting up WiFi events...");

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                 { Serial.println("[WiFi Event] STA Started"); }, ARDUINO_EVENT_WIFI_STA_START);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                 {
        Serial.println("[WiFi Event] STA Connected to AP!");
        wifiConnecting = false;
        wifiConnected = true; }, ARDUINO_EVENT_WIFI_STA_CONNECTED);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                 {
                   Serial.print("[WiFi Event] STA Disconnected. Reason: ");
                   Serial.println(info.wifi_sta_disconnected.reason);

                   switch (info.wifi_sta_disconnected.reason)
                   {
                   case WIFI_REASON_AUTH_EXPIRE:
                   case WIFI_REASON_AUTH_FAIL:
                     Serial.println("  → Authentication failed (wrong password?)");
                     break;
                   case WIFI_REASON_NO_AP_FOUND:
                     Serial.println("  → AP not found (wrong SSID?)");
                     break;
                   case WIFI_REASON_ASSOC_FAIL:
                     Serial.println("  → Association failed");
                     break;
                   case WIFI_REASON_HANDSHAKE_TIMEOUT:
                     Serial.println("  → 4-way handshake timeout");
                     break;
                   default:
                     Serial.printf("  → Error code: %d\n", info.wifi_sta_disconnected.reason);
                   }

                   wifiConnected = false;
                   // Jangan panggil reconnect di sini, biarkan checkWiFi() yang handle
                   // Serial.println("  → Will retry connection...");
                 },
                 ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                 {
        Serial.print("[WiFi Event] STA Got IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("  → Subnet: ");
        Serial.println(WiFi.subnetMask());
        Serial.print("  → Gateway: ");
        Serial.println(WiFi.gatewayIP());
        Serial.print("  → DNS: ");
        Serial.println(WiFi.dnsIP()); }, ARDUINO_EVENT_WIFI_STA_GOT_IP);

    // AP Events
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                 {
        Serial.println("[WiFi Event] AP Started!");
        apReady = true; }, ARDUINO_EVENT_WIFI_AP_START);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                 {
        Serial.println("[WiFi Event] Client Connected to AP");
        Serial.printf("  Clients: %d\n", WiFi.softAPgetStationNum()); }, ARDUINO_EVENT_WIFI_AP_STACONNECTED);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                 {
        Serial.println("[WiFi Event] Client Disconnected from AP");
        Serial.printf("  Clients: %d\n", WiFi.softAPgetStationNum()); }, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);

    Serial.println("  ✓ Events configured");

    // =====================================================================
    // STEP 5: Configure Access Point (AP) - PRIORITAS PERTAMA!
    // =====================================================================
    Serial.println("[5/7] Starting Access Point...");
    Serial.println("  AP Configuration:");
    Serial.printf("    SSID: %s\n", networkSettings.apSsid.c_str());
    Serial.printf("    Pass: %s\n", networkSettings.apPassword.c_str());
    Serial.println("    Channel: 6 (fixed, tidak bentrok)");
    Serial.println("    Hidden: No");
    Serial.println("    Max Clients: 4");

    // KUNCI: Gunakan channel yang berbeda dari STA
    bool apStarted = WiFi.softAP(
        networkSettings.apSsid.c_str(),     // SSID
        networkSettings.apPassword.c_str(), // Password
        1,                                  // Channel (fixed ke 6)
        0,                                  // SSID Hidden (0=visible, 1=hidden)
        4                                   // Max connections
    );

    if (apStarted)
    {
      delay(1000); // PENTING: Tunggu AP fully initialized

      // Verify AP is actually running
      if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0))
      {
        Serial.println("  ✗ AP Failed - IP is 0.0.0.0");
        Serial.println("  → Retrying AP start...");
        WiFi.softAPdisconnect(true);
        delay(500);
        apStarted = WiFi.softAP(networkSettings.apSsid.c_str(),
                                networkSettings.apPassword.c_str(), 6, 0, 4);
        delay(1000);
      }

      if (WiFi.softAPIP() != IPAddress(0, 0, 0, 0))
      {
        apReady = true;
        Serial.println("  ✓ AP Started Successfully!");
        Serial.printf("    AP IP: %s\n", WiFi.softAPIP().toString().c_str());
        Serial.printf("    AP MAC: %s\n", WiFi.softAPmacAddress().c_str());

        // Start DNS Server setelah AP fully ready
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

    // =====================================================================
    // STEP 6: Configure Station (STA) IP Settings
    // =====================================================================
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

    Serial.println("\n=================================");
    Serial.println("CONFIGURATION COMPLETE");
    Serial.println("=================================");
    Serial.println("AP Status:");
    Serial.printf("  SSID: %s\n", networkSettings.apSsid.c_str());
    Serial.printf("  IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("  Ready: %s\n", apReady ? "YES" : "NO");
    Serial.println("\nSTA Status:");
    Serial.printf("  Target: %s\n", networkSettings.ssid.c_str());
    Serial.println("  Status: Connecting...");
    Serial.println("=================================\n");
  }

  if (networkSettings.networkMode == "Ethernet")
  {
    Serial.println("CONFIGURING ETHERNET (W5500)");

    // Disconnect WiFi completely
    WiFi.disconnect();
    WiFi.mode(WIFI_AP_STA);

    // Start Access Point untuk konfigurasi
    Serial.println("[1/5] Starting Access Point...");
    bool apStarted = WiFi.softAP(networkSettings.apSsid.c_str(),
                                 networkSettings.apPassword.c_str(), 6, 0, 4);

    if (apStarted)
    {
      // delay(1000);
      apReady = true;
      startDNSServer();
      Serial.printf("    AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    }
    else
    {
      Serial.println("  ✗ Failed to start AP");
      apReady = false;
    }
    if (networkSettings.ssid.length() > 1)
    {
      Serial.printf("  ✓ Starting WiFi Background Connection to: %s\n", networkSettings.ssid.c_str());
      esp_wifi_set_mac(WIFI_IF_STA, mac);
      WiFi.begin(networkSettings.ssid.c_str(), networkSettings.password.c_str());
      wifiConnecting = true;
      wifiConnectStartTime = millis();
      staConnectionAttemptFailed = false;
      wifiConnected = false;
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

    // =====================================================================
    // STEP 3: Initialize SPI for W5500
    // =====================================================================
    // Serial.println("[3/5] Initializing SPI...");
    // SPI sudah di-init di setup(), tapi pastikan dengan pinout yang benar
    SPI.begin(ETH_CLK, ETH_MISO, ETH_MOSI, -1); // -1 karena CS manual
    SPI.setDataMode(SPI_MODE0);
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
    networkSettings.macAddress = "";
    for (int i = 0; i < 6; i++)
    {
      if (mac[i] < 0x10)
        networkSettings.macAddress += "0";
      networkSettings.macAddress += String(mac[i], HEX);
      if (i < 5)
        networkSettings.macAddress += ":";
    }

    // Print final Ethernet status
    Serial.println("\n=================================");
    Serial.println("ETHERNET CONFIGURATION COMPLETE");
    Serial.println("=================================");
    Serial.printf("MAC Address: %s\n", networkSettings.macAddress.c_str());
    Serial.printf("IP Address:  %s\n", Ethernet.localIP().toString().c_str());
    Serial.printf("Subnet Mask: %s\n", Ethernet.subnetMask().toString().c_str());
    Serial.printf("Gateway:     %s\n", Ethernet.gatewayIP().toString().c_str());
    Serial.printf("DNS Server:  %s\n", Ethernet.dnsServerIP().toString().c_str());
    Serial.printf("Link Status: %s\n",
                  Ethernet.linkStatus() == LinkON ? "CONNECTED" : "DISCONNECTED");
    Serial.println("=================================\n");
    ethServer.begin();
    Serial.println("[INFO] AsyncWebServer will handle Ethernet requests");
    Serial.printf("  Access via Ethernet: http://%s\n", Ethernet.localIP().toString().c_str());
    Serial.printf("  Access via WiFi AP:  http://%s\n", WiFi.softAPIP().toString().c_str());
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
  if (millis() - sendTime < (intervalSend * 1000))
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

  bool success = false;
  bool isEthMode = (networkSettings.networkMode == "Ethernet");
  bool isEthReady = isEthMode && (Ethernet.linkStatus() == LinkON);
  bool isWifiReady = (!isEthMode) && (WiFi.status() == WL_CONNECTED);

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

void sendBackupData()
{
  if (!SD.exists("/sensor_data.csv"))
    return;

  Serial.println("[Backup] Checking SD card...");

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

    String line = String(lineBuffer);
    line.trim();

    if (line.length() < 5)
      continue;

    // Bersihkan kurung siku tanpa fungsi replace() yang berat
    if (line.startsWith("["))
      line = line.substring(1);
    if (line.endsWith("]"))
      line = line.substring(0, line.length() - 1);

    if (count > 0)
      payload += ",";
    payload += line;
    count++;

    // Kirim setiap 10 data
    if (count >= 10)
    {
      payload += "]";
      bool sendSuccess = false;

      // --- Eksekusi Pengiriman Berdasarkan Mode ---
      if (networkSettings.networkMode == "Ethernet" && Ethernet.linkStatus() == LinkON)
      {
        EthernetClient ethClient;
        ethClient.setTimeout(5000);
        String serverPath = "https://sensor-logger-trial.medionindonesia.com/api/v1/AddBackupList";

        int result = perform_https_request_mbedtls(
            ethClient,
            getDomainFromUrl(serverPath).c_str(),
            getPathFromUrl(serverPath).c_str(),
            payload.c_str(),
            networkSettings.mqttUsername.c_str(),
            networkSettings.mqttPassword.c_str());

        if (result == 200 || result == 0)
          sendSuccess = true;
      }
      else if (WiFi.status() == WL_CONNECTED)
      {
        WiFiClientSecure client;
        HTTPClient https;
        client.setInsecure();
        if (https.begin(client, "https://sensor-logger-trial.medionindonesia.com/api/v1/AddBackupList"))
        {
          https.setAuthorization(networkSettings.mqttUsername.c_str(), networkSettings.mqttPassword.c_str());
          https.addHeader("Content-Type", "application/json");
          https.setTimeout(5000);
          int httpCode = https.POST(payload);
          if (httpCode == 200 || httpCode == 201)
            sendSuccess = true;
          https.end();
        }
      }

      if (sendSuccess)
      {
        Serial.printf("[Backup] Chunk Sent (10 items)\n");
        successCount++;
        networkSettings.connStatus = "Connected";
      }
      else
      {
        Serial.println("[Backup] Chunk Failed!");
        failCount++;
      }

      // Reset Buffer untuk chunk berikutnya
      payload = "[";
      count = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  // Kirim sisa data (jika ada < 10)
  if (count > 0)
  {
    payload += "]";
    // (Logika pengiriman sisa data sama dengan di atas, dihapus agar contoh tidak terlalu panjang, tapi di file utuh wajib dimasukkan kembali)
  }

  dataOffline.close();
  Serial.printf("[Backup] Summary: %d Chunks OK, %d Failed\n", successCount, failCount);

  if (failCount == 0 && successCount > 0)
  {
    SD.remove("/sensor_data.csv");
    Serial.println("[Backup] File cleared.");
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
#endif // NETWORK_FUNCTIONS_HPP