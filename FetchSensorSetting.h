#ifndef _FETCH_SENSOR_SETTING_H
#define _FETCH_SENSOR_SETTING_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "GlobalDef.h"

// --- เพิ่มส่วนนี้: บอกให้ไฟล์นี้รู้จักตัวแปรจากไฟล์หลัก ---
extern String deviceName;
extern SensorSetting sensorSettings[];
extern int sensorSettingCount;
// ---------------------------------------------------

bool fetchSensorSettingFromAPI(const char* urlBase, String mac) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  String url = String(urlBase) + mac;
  
  Serial.print("[GET] Requesting: ");
  Serial.println(url);

  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    // Serial.println(payload); // Debug

    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("JSON Parse failed: ");
      Serial.println(error.c_str());
      http.end();
      return false;
    }

    // 1. ดึงชื่อ Device Name
    const char* name = doc["device_name"];
    if (name) {
       deviceName = String(name);
       Serial.println("Device Name: " + deviceName);
    }

    // 2. ดึง Setting ของ Sensor
    JsonArray sensors = doc["setting_sensor"].as<JsonArray>();
    sensorSettingCount = 0;

    for (JsonObject obj : sensors) {
      if (sensorSettingCount < 8) {
        // -1 เพราะ Index ใน DB อาจเริ่มที่ 1 แต่ Array เริ่มที่ 0
        sensorSettings[sensorSettingCount].index_sensor = obj["index_sensor"].as<int>() - 1; 
        sensorSettings[sensorSettingCount].color = obj["color"].as<String>();
        
        // ใช้ชื่อ limit_on ให้ตรงกับ GlobalDef
        sensorSettings[sensorSettingCount].limit_on = obj["limit_on"].as<int>(); 
        
        // เช็คว่ามี limit_off ไหม ถ้าไม่มีให้ใส่ 0
        if (obj.containsKey("limit_off")) {
             sensorSettings[sensorSettingCount].limit_off = obj["limit_off"].as<int>();
        } else {
             sensorSettings[sensorSettingCount].limit_off = 0;
        }

        sensorSettingCount++;
      }
    }
    
    http.end();
    return true;
  } else {
    Serial.printf("[HTTP] GET failed, code: %d\n", httpCode);
    http.end();
    return false;
  }
}

#endif