#ifndef _GLOBAL_DEF_H
#define _GLOBAL_DEF_H

#include <Arduino.h>

struct SensorSetting {
    int index_sensor;  // หมายเลข Index ของ Sensor (0-7)
    String color;      // สีที่จะแสดงบนจอ ("red", "green", etc.)
    int limit_on;      // *** จุดที่ Error: ต้องชื่อ limit_on ***
    int limit_off;     // ค่า Limit สำหรับปิด (ถ้ามี)
};

#endif