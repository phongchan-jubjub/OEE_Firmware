#include <HardwareSerial.h>
#include <time.h>
#include <Ticker.h>
#include <WiFi.h>
#include "lcd_display.h"
#include "FetchSensorSetting.h"
#include <SPI.h>
#include <ModbusMaster.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Update.h> // เพิ่มเพื่อรองรับการเขียน Flash OTA

// >>> LittleFS FIFO Queue (ระบบจัดการคิวเมื่อเน็ตหลุด)
#include <FS.h>
#include <LittleFS.h>

#include "GlobalDef.h"

// =========================================================
// ********** ส่วนตั้งค่า (USER CONFIGURATION) **********
// =========================================================

// 0 = ใช้งานจริง (อ่านจาก Sensor)
// 1 = โหมดทดสอบ (จำลองค่าเอง)
#define USE_MOCK_MODBUS 0

// (ใช้เฉพาะตอน Mock) เลือกช่องที่จะสั่งให้ติด: 4, 6, หรือ 7
#define MOCK_TEST_CH 4

// =========================================================

// กำหนดขา Hardware
#define PIN_LCD_BL 23
const int RX_PIN = 7;  // ขา RX สำหรับ RS485
const int TX_PIN = 8;  // ขา TX สำหรับ RS485
const int SD_CS_PIN = 4;

String macAddress = "";
String deviceName = "ESP32Device_Final";  // ค่าเริ่มต้น (จะถูกเปลี่ยนตาม Server)
HardwareSerial modbusSerial(1);
ModbusMaster node;

// ตัวแปรเก็บค่า Sensor
uint16_t lux[8], status[8], freq[8];
uint16_t currentLux[8] = { 0 };
uint16_t prevStatus[8] = { 0xFFFF };
uint16_t lastSentStatus[8] = { 0xFFFF };

// การจับเวลา (Timing Variables)
unsigned long lastReadTime = 0;
const unsigned long readInterval = 1000;  // อ่าน Modbus ทุก 1 วินาที
unsigned long lastDisplayUpdate = 0;
const unsigned long DisplayUpdateInterval = 1000;  // อัปเดตจอทุก 1 วินาที
unsigned long lastWiFiCheckTime = 0;
const unsigned long wifiCheckInterval = 30000;  // เช็ค WiFi ทุก 30 วินาที
unsigned long lastSendTime = 0;
unsigned long totalSentCount = 0; // ตัวแปรนับจำนวน Record ที่ส่งสำเร็จจริง
volatile unsigned long lastSensorOkMs = 0;
volatile bool Flag_Sensor_Connected = false;

// ตั้งค่าเวลาเช็ค Config จาก Server (Auto-Update)
unsigned long lastConfigCheckTime = 0;
const unsigned long configCheckInterval = 10000;  // เช็คทุก 10 วินาที

// การ Retry ส่งข้อมูล
static const int POST_RETRY_MAX = 3;
static const uint16_t POST_RETRY_DELAY_MS = 300;

// ตัวแปรสถานะทั่วไป
bool isBlinking[8] = { false };
bool statusChanged[8] = { false };
bool shouldSendNow = true;
bool allDark = true;
bool displayTriggered = false;

// [ตัวแปรสำคัญ] เก็บค่า Config ล่าสุดในรูปแบบ String เพื่อเปรียบเทียบก่อนบันทึก (กัน Flash พัง)
String currentSavedConfig = "";

Ticker watchdogTicker;
volatile bool g_sensorTimeoutFired = false;

// ฟังก์ชันนับถอยหลัง Watchdog (ถ้าเซนเซอร์เงียบเกิน 60 วิ ให้รีบูต)
void IRAM_ATTR checkSensorTimeoutISR() {
  if (!USE_MOCK_MODBUS) {
    if (millis() - lastSensorOkMs > 60000) g_sensorTimeoutFired = true;
  }
}

// Config WiFi และ Server Endpoints
const char* ssid = "NPT-CEO";
const char* password = "2021020100";
const char* getDeviceInfoURL = "https://sips-test.orbray.co.th:9852/api/devices/by-mac/";
const char* postLogURL = "https://sips-test.orbray.co.th:9852/api/log-json";
const char* registerMacURL = "https://sips-test.orbray.co.th:9852/api/machines/register_mac";
const char* ackReboostBaseURL = "https://sips-test.orbray.co.th:9852/api/machines/";

// --- เพิ่มส่วน OTA Config ---
const char* currentFirmwareVersion = "1.0.3";
const char* versionUrl = "https://raw.githubusercontent.com/phongchan-jubjub/OEE_Firmware/refs/heads/main/OEE_Version.txt";
const char* firmwareBaseUrl = "https://github.com/phongchan-jubjub/OEE_Firmware/releases/download/";

// ตัวแปรเก็บการตั้งค่า Sensor (จาก API)
SensorSetting sensorSettings[8];
int sensorSettingCount = 0;

// ====================== LittleFS & Queue Constants ======================
static const char* QUEUE_DIR = "/queue";
static const char* COUNTER_PATH = "/queue/ctr.txt";
static const size_t MAX_QUEUE = 600;
static const char* CONFIG_FILE_PATH = "/last_config.json";  // ไฟล์เก็บ Config ถาวร

// ประกาศชื่อฟังก์ชันล่วงหน้า (Forward Declares)
static bool ensureQueueDir();
static bool enqueueLog(const String& json);
static void flushQueue();
bool initLittleFS();
static bool trySendSingle(const String& json);
static size_t queueCount(); 
void readModbusData();
void checkWiFiConnection();
void sendStatusToAPI(String lampColor, String statusStr);
void setLuxLimitsFromAPISettings();
String nowTimestampString();
bool isWiFiConnected();
String statusCodeToString(uint16_t code);
void fetchSensorConfig();
void registerMacAddress();
void saveConfigToLocal(String rawJson);
bool loadConfigFromLocal();
void sendReboostAck(int machineId);
bool startOTAUpdate(WiFiClient* client, int contentLength); 
String fetchLatestVersion();
void downloadAndApplyFirmware();

// ================= SETUP (เริ่มทำงานครั้งแรก) =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // ตั้งค่า Pin
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  initLCD();
  playLoadingAnimation(2);

  // เริ่มระบบไฟล์ (จำเป็นมากสำหรับ Queue และ Config Cache)
  if (!initLittleFS()) { Serial.println("[FATAL] LittleFS init failed"); }

  Serial.println("ESP32 RS485 WISE-100 Reader Start");
  checkWiFiConnection();

  String myMac = WiFi.macAddress();
  showStatusMessage("MAC: " + myMac);
  delay(1000);

  // -----------------------------------------------------------
  // [STEP 1] ลงทะเบียน MAC Address กับ Server
  // -----------------------------------------------------------
  Serial.println(">>> STEP 1: Registering Device...");
  registerMacAddress();
  delay(1000);

  // -----------------------------------------------------------
  // [STEP 2] ดึงค่า Config จาก API
  // -----------------------------------------------------------
  Serial.println(">>> STEP 2: Fetching Config from API...");
  fetchSensorConfig();

  // -----------------------------------------------------------
  // [Logic Fallback] กรณีดึง API ไม่สำเร็จ
  // -----------------------------------------------------------
  if (sensorSettingCount == 0) {
    Serial.println("[WARN] API Config Failed, Trying to load local config...");
    if (loadConfigFromLocal()) {
      Serial.println("✅ Loaded Config from Local Storage!");
    } else {
      Serial.println("⚠️ No Local Config found. Entering STANDBY MODE.");
      sensorSettingCount = 0;
    }
  }

  delay(1000);

  // เริ่มต้น Modbus
  modbusSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  modbusSerial.setTimeout(1000);
  node.begin(1, modbusSerial);

  setLuxLimitsFromAPISettings();
  readModbusData();

  // ให้บอร์ดจำสถานะล่าสุดทันทีหลังเปิดเครื่อง เพื่อไม่ให้ส่งรัวตอนเริ่มต้น
  for (int i = 0; i < 8; i++) lastSentStatus[i] = status[i];
  shouldSendNow = true;

  if (isWiFiConnected()) { flushQueue(); }

  lastSensorOkMs = millis();
  watchdogTicker.attach_ms(100, checkSensorTimeoutISR);

  Serial.print("[BOOT] Setup Complete | Mode: ");
  Serial.println(USE_MOCK_MODBUS ? "MOCK TEST" : "REAL SENSOR");
}

// ================= LOOP (วนทำงานตลอดเวลา) =================
void loop() {
  unsigned long now = millis();
  checkTouchInput();

  if (now - lastReadTime >= readInterval) {
    lastReadTime = now;
    readModbusData();
    lastSensorOkMs = millis();
  }

  if (now - lastDisplayUpdate >= DisplayUpdateInterval) {
    lastDisplayUpdate = now;
    displayTriggered = true;

    size_t freeHeap = ESP.getFreeHeap();
    size_t totalHeap = ESP.getHeapSize();
    float ramPercent = 100.0 * (1.0 - ((float)freeHeap / totalHeap));
    showSensorStatusOnlyTarget(nowTimestampString(), ramPercent, freeHeap, totalHeap, sensorSettings, sensorSettingCount, currentLux);

    Serial.println("----- Sensor Check -----");
    if (Flag_Sensor_Connected) {
      if (sensorSettingCount == 0) {
        Serial.println("   [STANDBY] No sensors configured.");
      }
      String allsum = "";
      for (int k = 0; k < sensorSettingCount; k++) {
        int sensorIdx = sensorSettings[k].index_sensor;
        int ch = sensorIdx + 1;
        Serial.printf("CH%d (%s) | Lux: %4d | StatusRaw: %d | Status: %s\n",
                      ch, sensorSettings[k].color.c_str(), currentLux[sensorIdx], status[sensorIdx], statusCodeToString(status[sensorIdx]).c_str());
        allsum += sensorSettings[k].color + "=" + statusCodeToString(status[sensorIdx]) + "|";
      }
      Serial.printf("Sum : ");
      Serial.println(allsum);
    } else {
      Serial.println("Waiting for sensor...");
    }
    Serial.println("------------------------");
    
    // [งานภูมิปรับปรุง] แสดง Queue เฉพาะเมื่อมีข้อมูลค้างหรือเน็ตหลุด
    size_t qCount = queueCount();
    if (!isWiFiConnected() || qCount > 0) {
        String statsMsg = "Queue:" + String(qCount);
        showStatusMessage(statsMsg);
    } else {
        showStatusMessage("System: Online");
    }
  }

  if (now - lastConfigCheckTime >= configCheckInterval) {
    lastConfigCheckTime = now;
    if (isWiFiConnected()) {
      Serial.println("[Auto-Update] Checking API...");
      fetchSensorConfig();
      setLuxLimitsFromAPISettings();
      lastSensorOkMs = millis();
    }
  }

  if (now - lastWiFiCheckTime >= wifiCheckInterval) {
    lastWiFiCheckTime = now;
    checkWiFiConnection();
  }

  static bool wasOnline = false;
  bool online = isWiFiConnected();
  
  if (online) {
    if (!wasOnline || queueCount() > 0) {
       flushQueue(); 
    }
  }
  wasOnline = online;

  // [งานภูมิปรับปรุง] ตรวจสอบการเปลี่ยนแปลงสถานะและส่งข้อมูลชุดเดียว
  bool anyStatusChanged = false;
  String triggerColor = "";
  String triggerStatus = "";

  for (int i = 0; i < 8; i++) {
    if (isMonitoredChannel(i)) {
      if (status[i] != lastSentStatus[i]) {
        lastSentStatus[i] = status[i]; // อัปเดตทันที
        if (statusCodeToString(status[i]) != "Dark") {
           anyStatusChanged = true;
           triggerColor = getChannelColor(i);
           triggerStatus = statusCodeToString(status[i]);
        }
      }
    }
  }

  if (anyStatusChanged) {
      sendStatusToAPI(triggerColor, triggerStatus);
      lastSendTime = millis();
      shouldSendNow = false;
  }

  if (shouldSendNow) {
    if (sensorSettingCount == 0) {
      // Standby
    } else if (allDark) {
      sendStatusToAPI("Unknown", "Unknown");
      lastSendTime = millis();
      shouldSendNow = false;
      for (int i = 0; i < 8; i++) { lastSentStatus[i] = status[i]; }
    } else {
      bool sentSomething = false;
      for (int i = 0; i < 8; i++) {
        if (isMonitoredChannel(i)) {
          bool timeOut = (millis() - lastSendTime >= 60000);
          if (timeOut) {
            if (statusCodeToString(status[i]) != "Dark") {
              sendStatusToAPI(getChannelColor(i), statusCodeToString(status[i]));
              lastSentStatus[i] = status[i];
              sentSomething = true;
              break;
            }
          }
        }
      }
      if (sentSomething) {
        lastSendTime = millis();
        shouldSendNow = false;
      }
    }
  }

  if (millis() - lastSendTime >= 60000) {
    shouldSendNow = true;
  }

  if (g_sensorTimeoutFired) {
    g_sensorTimeoutFired = false;
    Serial.println("Watchdog: Sensor Timeout -> Restart");
    ESP.restart();
  }
}

// ================= HELPER FUNCTIONS =================

void saveConfigToLocal(String rawJson) {
  if (rawJson == currentSavedConfig) {
    Serial.println("👌 Config is up-to-date (No write needed).");
    return;
  }

  File file = LittleFS.open(CONFIG_FILE_PATH, "w");
  if (!file) {
    Serial.println("❌ Failed to open config file for writing");
    return;
  }

  if (file.print(rawJson)) {
    Serial.println("💾 New Config saved to local storage.");
    currentSavedConfig = rawJson;
  } else {
    Serial.println("❌ Failed to write config to file");
  }
  file.close();
}

bool loadConfigFromLocal() {
  if (!LittleFS.exists(CONFIG_FILE_PATH)) return false;

  File file = LittleFS.open(CONFIG_FILE_PATH, "r");
  if (!file) return false;

  String payload = file.readString();
  file.close();

  currentSavedConfig = payload;

  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.println("❌ Failed to parse local config file");
    return false;
  }

  Serial.println("📂 Reading config from local storage...");

  if (doc.containsKey("device_name")) {
    deviceName = doc["device_name"].as<String>();
    Serial.println("   Device Name: " + deviceName);
  }

  String configStr = doc["device_config"].as<String>();
  DynamicJsonDocument configDoc(4096);
  deserializeJson(configDoc, configStr);
  JsonArray arr = configDoc.as<JsonArray>();

  sensorSettingCount = 0;
  for (JsonObject obj : arr) {
    if (sensorSettingCount >= 8) break;
    int api_index = obj["index_sensor"];
    sensorSettings[sensorSettingCount].index_sensor = api_index - 1;
    sensorSettings[sensorSettingCount].color = obj["color"].as<String>();
    sensorSettings[sensorSettingCount].limit_on = obj["min_lux"];
    sensorSettings[sensorSettingCount].limit_off = obj["max_lux"];
    sensorSettingCount++;
  }
  return (sensorSettingCount > 0);
}

void registerMacAddress() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Register] WiFi not connected!");
    return;
  }
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, registerMacURL);

  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);
  String jsonPayload = "{\"macAddress\":\"" + macAddress + "\"}";
  int httpCode = http.POST(jsonPayload);
  if (httpCode > 0) {
    if (httpCode == 200 || httpCode == 201 || httpCode == 409) {
      Serial.printf("✅ Register Check OK (%d)\n", httpCode);
    } else {
      Serial.printf("❌ Register Failed: %d\n", httpCode);
    }
  } else {
    Serial.printf("[Register] Network Error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

void sendReboostAck(int machineId) {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = String(ackReboostBaseURL) + machineId + "/ack_reboost";

  http.begin(client, url);
  Serial.println("[Reboost] Sending ACK to: " + url);
  http.addHeader("Content-Length", "0");
  int httpCode = http.POST("");
  if (httpCode > 0) {
    Serial.printf("[Reboost] ACK Sent! Code: %d\n", httpCode);
  } else {
    Serial.printf("[Reboost] Failed to send ACK: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

void fetchSensorConfig() {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = String(getDeviceInfoURL) + macAddress;

  http.begin(client, url);
  http.setTimeout(4000); 
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();

    // ส่วน OTA: เช็คเวอร์ชันใหม่จาก GitHub
    Serial.println("[OTA] Checking for new firmware...");
    String latestVersion = fetchLatestVersion();
    if (latestVersion != "" && latestVersion != currentFirmwareVersion) {
      Serial.println("🆕 New version found: " + latestVersion);
      showStatusMessage("Updating to V" + latestVersion);

      watchdogTicker.detach();
      // [จุดที่แก้ไข] ส่งค่าเวอร์ชันใหม่เข้าไปสร้าง URL
      downloadAndApplyFirmware(latestVersion); 
      watchdogTicker.attach_ms(100, checkSensorTimeoutISR);
    }

    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.println(F("[Config] JSON Error"));
      return;
    }

    if (doc.containsKey("status") && doc.containsKey("id")) {
      String serverStatus = doc["status"].as<String>();
      int machineId = doc["id"].as<int>();

      if (serverStatus == "RE-BOOST") {
        Serial.println("⚠️ RE-BOOST COMMAND RECEIVED!");
        sendReboostAck(machineId);
        delay(2000);
        Serial.println("Rebooting...");
        ESP.restart();
      }
    }

    if (payload == currentSavedConfig) {
      return;
    }

    Serial.println("[Config] New configuration detected!");

    if (doc.containsKey("device_name")) {
      deviceName = doc["device_name"].as<String>();
    }

    String configStr = doc["device_config"].as<String>();
    DynamicJsonDocument configDoc(4096);
    deserializeJson(configDoc, configStr);

    JsonArray arr = configDoc.as<JsonArray>();

    sensorSettingCount = 0;
    for (JsonObject obj : arr) {
      if (sensorSettingCount >= 8) break;
      int api_index = obj["index_sensor"];
      sensorSettings[sensorSettingCount].index_sensor = api_index - 1;
      sensorSettings[sensorSettingCount].color = obj["color"].as<String>();
      sensorSettings[sensorSettingCount].limit_on = obj["min_lux"];
      sensorSettings[sensorSettingCount].limit_off = obj["max_lux"];
      Serial.printf("   + Set CH%d (%s)\n", api_index, sensorSettings[sensorSettingCount].color.c_str());
      sensorSettingCount++;
    }

    Serial.printf("[Config] Updated Sensors: %d\n", sensorSettingCount);
    saveConfigToLocal(payload);

  } else {
    Serial.printf("[Config] HTTP Error: %d\n", httpCode);
  }
  http.end();
}

void readModbusData() {
  if (USE_MOCK_MODBUS) {
    allDark = true;
    for (int i = 0; i < 8; i++) {
      currentLux[i] = 0;
      status[i] = 0;
    }
    int targetIndex = MOCK_TEST_CH - 1;
    if (targetIndex >= 0 && targetIndex < 8) {
      currentLux[targetIndex] = 2000;
      status[targetIndex] = 1;
      allDark = false;
    }
    Flag_Sensor_Connected = true;
    shouldSendNow = true;
  } else {
    allDark = true;
    bool anySensorActive = false;

    uint8_t result = node.readHoldingRegisters(0, 16);
    if (result == node.ku8MBSuccess) {
      for (int i = 0; i < 8; i++) {
        lux[i] = node.getResponseBuffer(i * 2);
        currentLux[i] = lux[i];
      }
    } else {
      Flag_Sensor_Connected = false;
      Serial.printf("Read Lux Error: 0x%02X\n", result);
      return;
    }

    result = node.readHoldingRegisters(16, 8);
    if (result == node.ku8MBSuccess) {
      for (int i = 0; i < 8; i++) status[i] = node.getResponseBuffer(i);
    }
    result = node.readHoldingRegisters(24, 8);
    if (result == node.ku8MBSuccess) {
      for (int i = 0; i < 8; i++) freq[i] = node.getResponseBuffer(i);
    }

    for (int k = 0; k < sensorSettingCount; k++) {
      int idx = sensorSettings[k].index_sensor;
      uint16_t limit = sensorSettings[k].limit_on;
      if (status[idx] == 2 || status[idx] == 3) {
        continue;
      }
      if (currentLux[idx] < limit) {
        status[idx] = 0;
      } else if (status[idx] == 0) {
        status[idx] = 1;
      }
    }

    for (int i = 0; i < 8; i++) {
      if (status[i] != 0) anySensorActive = true;
    }
    Flag_Sensor_Connected = true;
    if (anySensorActive) allDark = false;
    lastSensorOkMs = millis();
  }
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(500);
      Serial.print(".");
      lastSensorOkMs = millis();
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("[WiFi] Connected! IP: ");
      Serial.println(WiFi.localIP());
      macAddress = WiFi.macAddress();
      configTime(7 * 3600, 0, "pool.ntp.org");
      flushQueue();
    }
  }
}

void sendStatusToAPI(String lampColor, String statusStr) {
  String ts_Now = nowTimestampString();
  String currentJsonStatus = "";
  for (int k = 0; k < sensorSettingCount; k++) {
    int idx = sensorSettings[k].index_sensor;
    String cColor = sensorSettings[k].color;
    String cStatus = statusCodeToString(status[idx]);
    currentJsonStatus += cColor + "=" + cStatus + "|";
  }

  String jsonData = "{";
  jsonData += "\"device_name\":\"" + deviceName + "\",";
  jsonData += "\"lampColor\":\"" + lampColor + "\",";
  jsonData += "\"Status\":\"" + statusStr + "\",";
  jsonData += "\"mac_address\":\"" + macAddress + "\",";
  jsonData += "\"timestamp\":\"" + ts_Now + "\",";
  jsonData += "\"jsonstatus\":\"" + currentJsonStatus + "\"";
  jsonData += "}";

  Serial.println(jsonData);
  showStatusMessage(deviceName + " , " + lampColor + " , " + statusStr);

  if (!isWiFiConnected()) {
    enqueueLog(jsonData);
    // แจ้งจำนวนคิวสะสมที่ Serial และหน้าจอ LCD ทันที
    Serial.printf(">>> Offline: Data saved to Queue (Total in Queue: %d)\n", queueCount());
    showStatusMessage("Queue:" + String(queueCount()));
    return;
  }
  watchdogTicker.detach();

  WiFiClientSecure client;
  client.setInsecure();

  bool ok = false;
  for (int attempt = 1; attempt <= POST_RETRY_MAX && isWiFiConnected(); ++attempt) {
    HTTPClient http;
    http.begin(client, postLogURL);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(4000); 
    int code = http.POST(jsonData);
    if (code >= 200 && code < 300) {
      ok = true;
      totalSentCount++; 
      http.end();
      break;
    } else {
      http.end();
      if (attempt < POST_RETRY_MAX) delay(POST_RETRY_DELAY_MS);
    }
  }
  if (ok) {
    flushQueue();
  } else {
    enqueueLog(jsonData);
    showStatusMessage("Queue:" + String(queueCount()));
    Serial.printf(">>> Post Failed: Saved to Queue (Total: %d)\n", queueCount());
  }
  watchdogTicker.attach_ms(100, checkSensorTimeoutISR);
}

void setLuxLimits(int channel, uint16_t lowLux, uint16_t highLux) {
  if (!USE_MOCK_MODBUS) {
    if (channel < 1 || channel > 8) return;
    uint16_t lowAddr = 0x00E6 + (channel - 1) * 2;
    uint16_t highAddr = 0x00F6 + (channel - 1) * 2;
    node.writeSingleRegister(lowAddr, lowLux);
    delay(50);
    node.writeSingleRegister(highAddr, highLux);
    delay(50);
  }
}

void setLuxLimitsFromAPISettings() {
  for (int i = 0; i < sensorSettingCount; i++) {
    int ch = sensorSettings[i].index_sensor + 1;
    setLuxLimits(ch, sensorSettings[i].limit_on, sensorSettings[i].limit_off);
  }
}

bool isMonitoredChannel(int chIndex) {
  for (int i = 0; i < sensorSettingCount; i++) {
    if (sensorSettings[i].index_sensor == chIndex) return true;
  }
  return false;
}

String getChannelColor(int index) {
  for (int i = 0; i < sensorSettingCount; i++) {
    if (sensorSettings[i].index_sensor == index) return sensorSettings[i].color;
  }
  return "-";
}

String statusCodeToString(uint16_t code) {
  switch (code) {
    case 0: return "Dark";
    case 1: return "High";
    case 2: return "SlowBlink";
    case 3: return "FastBlink";
    default: return "Unknown";
  }
}

String nowTimestampString() {
  time_t now;
  time(&now);
  if (now < 1700000000) return "1970-01-01 00:00:00";
  struct tm tmNow;
  localtime_r(&now, &tmNow);
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmNow);
  return String(buf);
}

bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

static bool ensureQueueDir() {
  if (!LittleFS.exists(QUEUE_DIR)) LittleFS.mkdir(QUEUE_DIR);
  return true;
}

static uint32_t readCounter() {
  File f = LittleFS.open(COUNTER_PATH, "r");
  if (!f) return 1;
  uint32_t v = f.parseInt();
  f.close();
  return (v == 0) ? 1 : v;
}

static void writeCounter(uint32_t v) {
  File f = LittleFS.open(COUNTER_PATH, "w");
  if (f) {
    f.printf("%u\n", v);
    f.close();
  }
}

static String idToPath(uint32_t id) {
  char name[32];
  snprintf(name, sizeof(name), "%s/%09u.json", QUEUE_DIR, id);
  return String(name);
}

static size_t queueCount() {
  size_t cnt = 0;
  File dir = LittleFS.open(QUEUE_DIR);
  if (!dir) return 0;
  File f = dir.openNextFile();
  while (f) {
    if (!f.isDirectory() && String(f.name()).endsWith(".json")) cnt++;
    f = dir.openNextFile();
  }
  return cnt;
}

static String getOldestItemPath() {
  String oldest = "";
  File dir = LittleFS.open(QUEUE_DIR);
  File f = dir.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      String full = String(f.name()).startsWith(QUEUE_DIR) ? String(f.name()) : String(QUEUE_DIR) + "/" + String(f.name());
      if (full.endsWith(".json")) {
        if (oldest == "" || full < oldest) oldest = full;
      }
    }
    f = dir.openNextFile();
  }
  return oldest;
}

static bool enqueueLog(const String& json) {
  if (!ensureQueueDir()) return false;
  while (queueCount() >= MAX_QUEUE) LittleFS.remove(getOldestItemPath());
  uint32_t id = readCounter();
  File f = LittleFS.open(idToPath(id), "w");
  if (!f) return false;
  f.print(json);
  f.close();
  writeCounter(id + 1);
  return true;
}

static bool trySendSingle(const String& json) {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, postLogURL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(4000); 
  int code = http.POST(json);
  http.end();
  if (code >= 200 && code < 300) {
    totalSentCount++;
    return true;
  }
  return false;
}

// ฟังก์ชันระบายคิวสำรองข้อมูลแบบ FIFO (เวอร์ชันปรับปรุงความไหลลื่น)
static void flushQueue() {
  if (WiFi.status() != WL_CONNECTED) return; // ข้ามฟังก์ชันถ้าไม่ได้เชื่อมต่อ WiFi
  
  // กำหนดจำนวนการส่งสูงสุดต่อรอบ เพื่อให้ระบบกลับมาทำงานหลักได้รวดเร็ว
  int sendLimit = 10; 
  int sendCount = 0;

  watchdogTicker.detach(); // ปิด Watchdog ชั่วคราวป้องกันการรีเซ็ตระหว่างจัดการไฟล์
  
  while (WiFi.status() == WL_CONNECTED && sendCount < sendLimit) {
    String p = getOldestItemPath(); // ดึงไฟล์ที่เก่าที่สุดในคิว (FIFO)
    if (p == "") break; // ถ้าไม่มีไฟล์ค้างในคิวแล้ว ให้จบการทำงาน
    
    File f = LittleFS.open(p, "r");
    if (!f) {
      LittleFS.remove(p); // ถ้าไฟล์เสียหรือเปิดไม่ได้ ให้ลบทิ้งทันทีเพื่อไม่ให้ขวางคิว
      continue;
    }
    String j = f.readString();
    f.close();
    
    // พยายามส่งข้อมูลไปยัง API
    if (trySendSingle(j)) {
      Serial.println(">>> [Queue] Send Success: Removing record.");
      LittleFS.remove(p); // ส่งสำเร็จ ลบไฟล์ทิ้งตามปกติ
    } else {
      // หากส่งไม่ผ่าน (เช่น ข้อมูลผิดรูปแบบหรือ Server Reject) ให้ลบทิ้งเพื่อเปิดทางให้ไฟล์อื่น
      Serial.println(">>> [Queue] Send Failed/Rejected: Removing to unblock queue.");
      LittleFS.remove(p); 
    }
    
    sendCount++; // นับจำนวนไฟล์ที่จัดการไปแล้วในรอบนี้
    delay(20);   // หน่วงเวลาเล็กน้อยเพื่อเสถียรภาพของระบบ
  }
  
  if (sendCount > 0) {
    Serial.printf(">>> [Queue] Batch finished: %d records handled. Returning to main loop.\n", sendCount);
  }

  lastSensorOkMs = millis(); // รีเซ็ตเวลาสำหรับ Watchdog
  watchdogTicker.attach_ms(100, checkSensorTimeoutISR); // เปิด Watchdog กลับมาทำงานตามปกติ
}

bool initLittleFS() {
  return LittleFS.begin(true) && ensureQueueDir();
}

// --- OTA FUNCTIONS ---

String fetchLatestVersion() {
  if (WiFi.status() != WL_CONNECTED) return "";
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, versionUrl);
  http.setTimeout(4000); 
  int httpCode = http.GET();
  String latestVersion = "";
  if (httpCode == HTTP_CODE_OK) {
    latestVersion = http.getString();
    latestVersion.trim();
  }
  http.end();
  return latestVersion;
}

void downloadAndApplyFirmware(String newVersion) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  // [จุดที่แก้ไข] ประกอบร่าง URL: Base + Folder Name + File Name
  // ผลลัพธ์จะเป็น: https://github.com/phongchan-jubjub/OEE_Firmware/releases/download/OEE_Firmware_V1.0.2/TowerLampMonitoring_V4.ino.bin
  String fullFirmwareUrl = String(firmwareBaseUrl) + "OEE_Firmware_V" + newVersion + "/TowerLampMonitoring_V4.ino.bin";
  
  Serial.println("[OTA] Downloading from: " + fullFirmwareUrl);
  
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.begin(client, fullFirmwareUrl);
  http.setTimeout(15000); // เพิ่มเวลาเผื่อดาวน์โหลดไฟล์ใหญ่
  
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    if (contentLength > 0) {
      WiFiClient* stream = http.getStreamPtr();
      if (startOTAUpdate(stream, contentLength)) {
        Serial.println("✅ Update Complete. Rebooting...");
        delay(2000);
        ESP.restart();
      }
    }
  } else {
    Serial.printf("❌ Download Failed, HTTP Code: %d\n", httpCode);
  }
  http.end();
}

bool startOTAUpdate(WiFiClient* client, int contentLength) {
  if (!Update.begin(contentLength)) {
    Serial.printf("OTA Error: %s\n", Update.errorString());
    return false;
  }

  size_t written = 0;
  uint8_t buffer[256];
  int lastProgress = -1;

  while (written < contentLength) {
    if (client->available()) {
      size_t len = client->read(buffer, sizeof(buffer));
      if (len > 0) {
        Update.write(buffer, len);
        written += len;
        
        int progress = (written * 100) / contentLength;
        if (progress % 10 == 0 && progress != lastProgress) {
          Serial.printf("OTA Progress: %d%%\n", progress);
          lastProgress = progress;
        }
      }
    }
    yield();
  }
  
  if (Update.end()) {
    if (Update.isFinished()) {
      Serial.println("✅ Update Success!");
      return true;
    }
  }
  Serial.printf("❌ Update Failed: %s\n", Update.errorString());
  return false;
}