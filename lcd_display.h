#ifndef _LCD_DISPLAY_H
#define _LCD_DISPLAY_H

// ==========================================
// 1. ส่วนนำเข้าไลบรารีและตั้งค่าสี
// ==========================================
#include <Arduino_GFX_Library.h>  // ไลบรารีหลักสำหรับวาดกราฟิกบนจอ
#include <Wire.h>                 // ไลบรารี I2C สำหรับคุยกับทัชสกรีน
#include <math.h>                 // ฟังก์ชันคณิตศาสตร์ (ใช้คำนวณวงกลม/มุม)
#include "GlobalDef.h"            // ไฟล์ตั้งค่าส่วนกลาง

// ดึงตัวแปร global จากไฟล์ .ino มาใช้ในการคำนวณ Touch
extern int sensorSettingCount;
extern uint16_t status[8];

// --- นิยามสี (Color Definitions) แบบ 16-bit (RGB565) ---
#define BLACK 0x0000
#define NAVY 0x000F
#define DARKGREEN 0x03E0
#define DARKCYAN 0x03EF
#define MAROON 0x7800
#define PURPLE 0x780F
#define OLIVE 0x7BE0
#define LIGHTGREY 0xC618
#define DARKGREY 0x7BEF
#define BLUE 0x001F
#define GREEN 0x07E0
#define CYAN 0x07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define ORANGE 0xFD20
#define GREENYELLOW 0xAFE5
#define PINK 0xF81F

// ==========================================
// 2. การตั้งค่าฮาร์ดแวร์ (Hardware Config)
// ==========================================

// --- ขา Pin ที่ต่อกับจอ LCD (SPI Interface) ---
#define LCD_PIN_MISO -1  // ไม่ใช้
#define LCD_PIN_MOSI 2   // ขาส่งข้อมูลภาพ
#define LCD_PIN_SCLK 1   // ขาสัญญาณนาฬิกา
#define LCD_PIN_CS 14    // Chip Select
#define LCD_PIN_DC 15    // Data/Command
#define LCD_PIN_RST 22   // Reset จอ
#define LCD_PIN_BL 23    // Backlight (ไฟพื้นหลัง)

// --- ขา Pin ที่ต่อกับทัชสกรีน (I2C Interface) ---
#define TOUCH_SDA 18     // สาย Data I2C
#define TOUCH_SCL 19     // สาย Clock I2C
#define TOUCH_RST 20     // Reset ทัชสกรีน
#define TOUCH_INT 21     // Interrupt (แจ้งเตือนเมื่อมีการกด)
#define TOUCH_ADDR 0x63  // ที่อยู่ I2C ของชิปทัชสกรีน

// ขนาดจอภาพ
#define LCD_WIDTH 172
#define LCD_HEIGHT 320

// ==========================================
// 3. ตั้งค่าระบบความปลอดภัย (Security)
// ==========================================
const String SECRET_PIN = "1234";  // <<<< แก้รหัสผ่านตรงนี้
bool isKeypadActive = false;       // ตัวแปรเช็คว่า "ตอนนี้กำลังโชว์แป้นตัวเลขไหม"
String inputPasscode = "";         // ตัวแปรเก็บตัวเลขที่ผู้ใช้กดเข้ามา

// ==========================================
// 4. ตัวแปร Global (ใช้ร่วมกันทั้งไฟล์)
// ==========================================
int currentPage = 0;                  // หน้าปัจจุบัน (0=กราฟ, 1=ปุ่มกด)
int selectedChannel = -1;             // เก็บว่าเราจิ้มกราฟแท่งไหนอยู่ (-1 คือไม่ได้จิ้ม)
unsigned long popupStartTime = 0;     // เวลาที่เริ่มโชว์ Popup ค่าตัวเลข
bool isTouching = false;              // สถานะว่านิ้วแตะจออยู่ไหม
int startTouchX = 0, lastTouchX = 0;  // พิกัดเริ่มกด และพิกัดล่าสุด (เอาไว้คำนวณการปัด)
int startTouchY = 0, lastTouchY = 0;
unsigned long touchStartTime = 0;  // เวลาที่เริ่มกด (เอาไว้เช็คว่ากดสั้นหรือยาว)

// ตัวแปรสำหรับข้อความสถานะด้านล่างจอ
String lastStatusMsg = "";
unsigned long statusMsgStartTime = 0;
bool isStatusShowing = false;

// ประกาศชื่อฟังก์ชันล่วงหน้า (Prototype)
void checkWiFiConnection();
void readModbusData();
void performRestart();

// ==========================================
// 5. เริ่มต้นวัตถุจอภาพ (Display Object Init)
// ==========================================

Arduino_DataBus *bus = new Arduino_HWSPI(LCD_PIN_DC, LCD_PIN_CS, LCD_PIN_SCLK, LCD_PIN_MOSI);
Arduino_GFX *lcd = new Arduino_ST7789(bus, LCD_PIN_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 34, 0, 34, 0);

// --- Magic Code: ชุดคำสั่งพิเศษสำหรับตั้งค่า Driver จอ ---
void lcd_reg_init(void) {
  static const uint8_t init_operations[] = {
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x11, END_WRITE, DELAY, 120,  // Sleep Out
    BEGIN_WRITE,
    WRITE_C8_D16, 0xDF, 0x98, 0x53,
    WRITE_C8_D8, 0xB2, 0x23,
    WRITE_COMMAND_8, 0xB7, WRITE_BYTES, 4, 0x00, 0x47, 0x00, 0x6F,
    WRITE_COMMAND_8, 0xBB, WRITE_BYTES, 6, 0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0,
    WRITE_C8_D16, 0xC0, 0x44, 0xA4,
    WRITE_C8_D8, 0xC1, 0x16,
    WRITE_COMMAND_8, 0xC3, WRITE_BYTES, 8, 0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77,
    WRITE_COMMAND_8, 0xC4, WRITE_BYTES, 12, 0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82,
    WRITE_COMMAND_8, 0xC8, WRITE_BYTES, 32, 0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00, 0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,
    WRITE_COMMAND_8, 0xD0, WRITE_BYTES, 5, 0x04, 0x06, 0x6B, 0x0F, 0x00,
    WRITE_C8_D16, 0xD7, 0x00, 0x30,
    WRITE_C8_D8, 0xE6, 0x14,
    WRITE_C8_D8, 0xDE, 0x01,
    WRITE_COMMAND_8, 0xB7, WRITE_BYTES, 5, 0x03, 0x13, 0xEF, 0x35, 0x35,
    WRITE_COMMAND_8, 0xC1, WRITE_BYTES, 3, 0x14, 0x15, 0xC0,
    WRITE_C8_D16, 0xC2, 0x06, 0x3A,
    WRITE_C8_D16, 0xC4, 0x72, 0x12,
    WRITE_C8_D8, 0xBE, 0x00,
    WRITE_C8_D8, 0xDE, 0x02,
    WRITE_COMMAND_8, 0xE5, WRITE_BYTES, 3, 0x00, 0x02, 0x00,
    WRITE_COMMAND_8, 0xE5, WRITE_BYTES, 3, 0x01, 0x02, 0x00,
    WRITE_C8_D8, 0xDE, 0x00, WRITE_C8_D8, 0x35, 0x00, WRITE_C8_D8, 0x3A, 0x05,
    WRITE_COMMAND_8, 0x2A, WRITE_BYTES, 4, 0x00, 0x22, 0x00, 0xCD,
    WRITE_COMMAND_8, 0x2B, WRITE_BYTES, 4, 0x00, 0x00, 0x01, 0x3F,
    WRITE_C8_D8, 0xDE, 0x02,
    WRITE_COMMAND_8, 0xE5, WRITE_BYTES, 3, 0x00, 0x02, 0x00,
    WRITE_C8_D8, 0xDE, 0x00, WRITE_C8_D8, 0x36, 0x00,
    WRITE_COMMAND_8, 0x21, END_WRITE,  // Inversion On
    DELAY, 10,
    BEGIN_WRITE, WRITE_COMMAND_8, 0x29, END_WRITE  // Display On
  };
  bus->batchOperation(init_operations, sizeof(init_operations));
}

void turnOnLCD() {
  pinMode(LCD_PIN_BL, OUTPUT);
  digitalWrite(LCD_PIN_BL, HIGH);  // เปิดไฟหน้าจอ
}

void initLCD() {
  turnOnLCD();

  pinMode(LCD_PIN_RST, OUTPUT);
  digitalWrite(LCD_PIN_RST, HIGH);
  delay(50);
  digitalWrite(LCD_PIN_RST, LOW);
  delay(200);
  digitalWrite(LCD_PIN_RST, HIGH);
  delay(200);

  if (!lcd->begin()) { Serial.println("LCD Init Failed!"); }
  lcd_reg_init();
  lcd->setRotation(0);
  lcd->fillScreen(BLACK);

  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  Wire.setClock(400000);
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(50);
  digitalWrite(TOUCH_RST, HIGH);
  pinMode(TOUCH_INT, INPUT_PULLUP);
  delay(500);
}

// ฟังก์ชันวาดวงกลมหมุนๆ ตอนโหลด
void playLoadingAnimation(int loops) {
  int cx = 86;
  int cy = 160;
  int r = 35;
  int dots = 8;
  lcd->setTextSize(1);
  lcd->setTextColor(WHITE);
  lcd->setCursor(cx - 20, cy - 3);
  lcd->print("LOADING");
  for (int j = 0; j < loops; j++) {
    for (int i = 0; i < dots; i++) {
      float angle = (2 * PI / dots) * i;
      int x = cx + r * cos(angle);
      int y = cy + r * sin(angle);
      lcd->fillCircle(x, y, 6, CYAN);
      delay(30);
      lcd->fillCircle(x, y, 6, 0x1082);
    }
  }
  lcd->fillScreen(BLACK);
}

// ฟังก์ชันแปลงชื่อสี (String) เป็นรหัสสี (Hex)
uint16_t colorFromName(const String &name) {
  if (name.equalsIgnoreCase("red")) return RED;
  if (name.equalsIgnoreCase("green")) return GREEN;
  if (name.equalsIgnoreCase("blue")) return BLUE;
  if (name.equalsIgnoreCase("yellow")) return YELLOW;
  if (name.equalsIgnoreCase("orange")) return ORANGE;
  return LIGHTGREY;
}

// ฟังก์ชันเลือกคู่สี (สว่าง/มืด) ตามชื่อ
void getColorSet(String name, uint16_t &brightColor, uint16_t &dimColor) {
  name.toLowerCase();
  if (name == "red") {
    brightColor = RED;
    dimColor = 0x3000;
  } else if (name == "green") {
    brightColor = GREEN;
    dimColor = 0x0160;
  } else if (name == "yellow") {
    brightColor = YELLOW;
    dimColor = 0x3180;
  } else if (name == "blue") {
    brightColor = BLUE;
    dimColor = 0x0005;
  } else if (name == "orange") {
    brightColor = ORANGE;
    dimColor = 0x4100;
  } else {
    brightColor = LIGHTGREY;
    dimColor = 0x1082;
  }
}

void performRestart() {
  lcd->fillScreen(BLACK);
  lcd->setTextColor(RED);
  lcd->setTextSize(2);
  lcd->setCursor(26, 150);
  lcd->print("RESTARTING");
  delay(1000);
  ESP.restart();
}

// ==========================================
// 6. ฟังก์ชันวาดปุ่มกด (Keypad UI)
// ==========================================

void drawSingleKey(int r, int c, bool pressed) {
  int startX = 20, startY = 110;
  int w = 40, h = 40, gap = 6;
  int x = startX + c * (w + gap);
  int y = startY + r * (h + gap);

  char keys[4][3] = {
    { '1', '2', '3' },
    { '4', '5', '6' },
    { '7', '8', '9' },
    { 'D', '0', 'K' }
  };
  char key = keys[r][c];

  uint16_t bgColor = DARKGREY;
  uint16_t txtColor = WHITE;

  if (key == 'K') bgColor = GREEN;   // ปุ่ม OK สีเขียว
  if (key == 'D') bgColor = MAROON;  // ปุ่มลบ สีแดงเลือดหมู

  if (pressed) {
    bgColor = WHITE;
    txtColor = BLACK;
    if (key == 'K') bgColor = 0x07E0;
    if (key == 'D') bgColor = RED;
  }

  lcd->fillRoundRect(x, y, w, h, 5, bgColor);
  lcd->drawRoundRect(x, y, w, h, 5, WHITE);
  lcd->setTextColor(txtColor);
  lcd->setTextSize(2);

  if (key == 'K') {
    lcd->setCursor(x + 8, y + 12);
    lcd->print("OK");
  } else if (key == 'D') {
    lcd->setCursor(x + 5, y + 12);
    lcd->print("Del");
  } else {
    lcd->setCursor(x + 14, y + 12);
    lcd->print(key);
  }
}

void drawKeypad(bool forceRedraw) {
  if (!forceRedraw && !isKeypadActive) return;

  if (forceRedraw) {
    lcd->fillScreen(BLACK);
    lcd->setTextColor(WHITE);
    lcd->setTextSize(2);
    lcd->setCursor(40, 20);
    lcd->print("ENTER PIN");

    // ปุ่มปิด (X)
    lcd->fillRoundRect(140, 5, 25, 25, 4, RED);
    lcd->setCursor(147, 10);
    lcd->print("X");

    for (int r = 0; r < 4; r++) {
      for (int c = 0; c < 3; c++) {
        drawSingleKey(r, c, false);
      }
    }
  }

  // วาดช่องกรอกรหัส
  lcd->drawRoundRect(20, 50, 132, 40, 5, WHITE);
  lcd->fillRect(22, 52, 128, 36, BLACK);
  lcd->setTextColor(CYAN);
  lcd->setTextSize(3);

  String mask = "";
  for (int i = 0; i < inputPasscode.length(); i++) mask += "*";
  int txtW = mask.length() * 18;
  lcd->setCursor(20 + (132 - txtW) / 2, 60);
  lcd->print(mask);
}

// ==========================================
// 7. ฟังก์ชันรับสัมผัส (Touch Input Logic) **ปรับปรุง Hitbox + Fixed Graph**
// ==========================================
void checkTouchInput() {
  if (digitalRead(TOUCH_INT) == HIGH && !isTouching) {
    return;
  }

  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return;

  Wire.requestFrom(TOUCH_ADDR, 7);
  if (Wire.available() >= 7) {
    uint8_t buf[7];
    for (int i = 0; i < 7; i++) buf[i] = Wire.read();

    int points = buf[2];
    int rawX = ((buf[3] & 0x0F) << 8) | buf[4];
    int rawY = ((buf[5] & 0x0F) << 8) | buf[6];

    int touchX = LCD_WIDTH - rawX;  // Mirror X
    int touchY = rawY;              // Normal Y

    if (points > 0) {
      if (!isTouching) {
        isTouching = true;
        startTouchX = touchX;
        startTouchY = touchY;
        touchStartTime = millis();
      }
      lastTouchX = touchX;
      lastTouchY = touchY;
    } else {
      // นิ้วปล่อยจากจอ (Release)
      if (isTouching) {
        isTouching = false;
        int deltaX = lastTouchX - startTouchX;
        int deltaY = lastTouchY - startTouchY;
        unsigned long duration = millis() - touchStartTime;

        // --- โหมด 1: Keypad (Padding Hitbox) ---
        if (isKeypadActive) {
          if (startTouchY < 50 && startTouchX > 120) {
            isKeypadActive = false;
            currentPage = 1;
            return;
          }

          int startX = 20;
          int startY = 110;
          int w = 40, h = 40, gap = 6;
          int padding = 20;  // Expanded Hitbox

          int keypadTop = startY - padding;
          int keypadBottom = startY + (4 * (h + gap)) + padding;
          int keypadLeft = startX - padding;
          int keypadRight = startX + (3 * (w + gap)) + padding;

          if (startTouchY >= keypadTop && startTouchY <= keypadBottom && startTouchX >= keypadLeft && startTouchX <= keypadRight) {

            int gridW = w + gap;
            int gridH = h + gap;
            int relX = startTouchX - startX;
            int relY = startTouchY - startY;
            int col, row;

            if (relX < 0) col = 0;
            else col = relX / gridW;
            if (col > 2) col = 2;
            if (relY < 0) row = 0;
            else row = relY / gridH;
            if (row > 3) row = 3;

            if (col >= 0 && col < 3 && row >= 0 && row < 4) {
              drawSingleKey(row, col, true);
              delay(100);
              drawSingleKey(row, col, false);

              char key = ' ';
              if (row == 0) key = (col == 0) ? '1' : (col == 1) ? '2'
                                                                : '3';
              if (row == 1) key = (col == 0) ? '4' : (col == 1) ? '5'
                                                                : '6';
              if (row == 2) key = (col == 0) ? '7' : (col == 1) ? '8'
                                                                : '9';
              if (row == 3) key = (col == 0) ? 'D' : (col == 1) ? '0'
                                                                : 'K';

              if (key >= '0' && key <= '9') {
                if (inputPasscode.length() < 6) inputPasscode += key;
                drawKeypad(false);
              } else if (key == 'D') {
                if (inputPasscode.length() > 0) inputPasscode.remove(inputPasscode.length() - 1);
                drawKeypad(false);
              } else if (key == 'K') {
                if (inputPasscode == SECRET_PIN) {
                  performRestart();
                } else {
                  lcd->drawRoundRect(20, 50, 132, 40, 5, RED);
                  delay(200);
                  lcd->drawRoundRect(20, 50, 132, 40, 5, WHITE);
                  inputPasscode = "";
                  drawKeypad(false);
                }
              }
            }
          }
          return;
        }

        // --- โหมด 2: Swipe ---
        if (abs(deltaX) > 30 && abs(deltaY) < 50) {
          int totalPages = 2;
          if (deltaX < -30) {
            currentPage++;
            if (currentPage >= totalPages) currentPage = 0;
          } else if (deltaX > 30) {
            currentPage--;
            if (currentPage < 0) currentPage = totalPages - 1;
          }
        }
        // --- โหมด 3: Tap (จิ้ม) ---
        else if (duration < 500 && abs(deltaX) < 15 && abs(deltaY) < 15) {
          if (currentPage == 0) {  // หน้ากราฟ

            // --- [แก้ไข] พิกัดตายตัว (Fixed Top-Down) ---
            int lampWidth = 100;
            int lampHeight = 42;
            int gap = 6;
            int startY = 60;  // Fixed Start Y
            int startX = (LCD_WIDTH - lampWidth) / 2;
            // ------------------------------------------

            // ตรวจสอบว่าจิ้มโดนหลอดไหน
            if (startTouchX >= startX && startTouchX <= startX + lampWidth) {
              for (int i = 0; i < sensorSettingCount; i++) {
                int lampTop = startY + (i * (lampHeight + gap));
                int lampBottom = lampTop + lampHeight;

                if (startTouchY >= lampTop && startTouchY <= lampBottom) {
                  selectedChannel = i;
                  popupStartTime = millis();
                  break;
                }
              }
            }

          } else if (currentPage == 1) {  // หน้าปุ่ม
            // ปุ่ม RESTART กลางจอ
            if (startTouchY >= 120 && startTouchY <= 200) {
              isKeypadActive = true;
              inputPasscode = "";
            }
          }
        }
      }
    }
  }
}

// ==========================================
// 8. ฟังก์ชันวาดกราฟิกหน้าต่างๆ
// ==========================================

// หน้าที่ 1: กราฟแท่งแนวนอน (Tower) แบบ Smart Refresh ป้องกันจอ Overload
void drawPage1_Tower(int w, const SensorSetting sensorList[], int deviceCount, const uint16_t currentLux[], bool forceRedraw) {
  if (deviceCount <= 0) return;

  int lampWidth = 100;
  int lampHeight = 42;
  int gap = 6;
  int startY = 60;
  int startX = (w - lampWidth) / 2;
  int poleHeight = 200;

  // 1. วาดส่วนโครงสร้างพื้นหลัง
  if (forceRedraw) {
    lcd->fillRect(0, 40, w, 240, BLACK);
    lcd->fillRect(startX + (lampWidth / 2) - 8, startY, 16, poleHeight, 0x8410); // Pole
    int baseW = 50;
    int baseH = 12;
    int baseY = startY + poleHeight - baseH;
    lcd->fillRect(startX + (lampWidth / 2) - (baseW / 2), baseY, baseW, baseH, 0x8410); // Base
  }

  if (selectedChannel != -1 && millis() - popupStartTime > 3000) {
    selectedChannel = -1;
  }

  int currentY = startY;
  static uint16_t lastDrawColor[8] = { 0 };
  static int lastSelected = -2;

  if (forceRedraw) {
    for (int k = 0; k < 8; k++) lastDrawColor[k] = 0xFFFF;
    lastSelected = -99;
  }

  // 2. วนลูปวาดหลอดไฟแต่ละดวง
  for (int i = 0; i < deviceCount; i++) {
    uint16_t brightColor = LIGHTGREY;
    uint16_t dimColor = 0x4208;

    int chIndex = sensorList[i].index_sensor;
    uint16_t val = currentLux[chIndex];
    getColorSet(sensorList[i].color, brightColor, dimColor);

    // --- แก้ไขตรรกะ: ให้ติดค้างถ้าไม่ใช่ Dark (0) ---
    int currentStatus = status[chIndex]; 
    bool isVisible = false;

    // ตรวจสอบสถานะ: 1 (High), 2 (SlowBlink), 3 (FastBlink) จะถือว่าติดค้างทั้งหมด [cite: 150, 151]
    if (currentStatus > 0) { 
      isVisible = true; 
    } else {
      isVisible = false; // สถานะ 0 (Dark) [cite: 150]
    }

    uint16_t targetColor = isVisible ? brightColor : dimColor;

    // วาดใหม่เฉพาะเมื่อสีเปลี่ยน หรือมีการสั่ง Force หรือเปลี่ยนการเลือกช่อง
    if (targetColor != lastDrawColor[i] || forceRedraw || selectedChannel != lastSelected) {
      lcd->fillRoundRect(startX, currentY, lampWidth, lampHeight, 12, targetColor);

      if (isVisible) {
        lcd->fillRoundRect(startX + 8, currentY + 6, 25, 8, 4, WHITE);
        lcd->drawRoundRect(startX, currentY, lampWidth, lampHeight, 12, WHITE);
      } else {
        lcd->drawRoundRect(startX, currentY, lampWidth, lampHeight, 12, 0x528A);
      }

      if (selectedChannel == i) {
        int popW = 80;
        int popH = 24;
        int popX = startX + (lampWidth - popW) / 2;
        int popY = currentY + (lampHeight - popH) / 2;
        lcd->fillRoundRect(popX, popY, popW, popH, 6, BLACK);
        lcd->drawRoundRect(popX, popY, popW, popH, 6, WHITE);
        lcd->setTextColor(WHITE);
        lcd->setTextSize(2);
        String vStr = String(val);
        int vW = vStr.length() * 12;
        lcd->setCursor(popX + (popW - vW) / 2, popY + 5);
        lcd->print(vStr);
      }
      lastDrawColor[i] = targetColor;
    }
    currentY += (lampHeight + gap);
  }
  lastSelected = selectedChannel;
}

// หน้าที่ 2: ปุ่มกดขนาดใหญ่
void drawPage2_Buttons(int w, bool forceRedraw) {
  if (forceRedraw) {
    lcd->setTextColor(CYAN, BLACK);
    lcd->setTextSize(2);
    String title = "CONTROL PANEL";
    int titleW = title.length() * 12;
    int titleX = (w - titleW) / 2;
    lcd->setCursor(titleX, 50);
    lcd->print(title);
    lcd->drawFastHLine(15, 70, 142, CYAN);

    int btnX = 20;
    int btnW = 132;
    int btnH = 80;

    // --- ปุ่ม RESTART (ย้ายมาไว้ตรงกลางจอ) ---
    int rY = 120;

    lcd->fillRoundRect(btnX, rY, btnW, btnH, 15, RED);
    lcd->drawRoundRect(btnX, rY, btnW, btnH, 15, WHITE);
    lcd->setTextColor(WHITE);
    lcd->setTextSize(2);
    String txt1 = "RESTART";
    lcd->setCursor(btnX + (btnW - (txt1.length() * 12)) / 2, rY + (btnH - 14) / 2);
    lcd->print(txt1);
  }
}

// ==========================================
// 9. Main Loop Display Controller
// ==========================================
void showSensorStatusOnlyTarget(const String &timeStr, float ramPercent, size_t freeHeap, size_t totalHeap, const SensorSetting sensorList[], int deviceCount, const uint16_t currentLux[]) {
  static int lastPage = -1;
  static bool wasKeypadActive = false;

  if (isKeypadActive) {
    if (!wasKeypadActive) {
      drawKeypad(true);
      wasKeypadActive = true;
    }
    return;
  } else {
    if (wasKeypadActive) {
      wasKeypadActive = false;
      lastPage = -1;
    }
  }

  bool pageChanged = (currentPage != lastPage);

  if (pageChanged) {
    lcd->fillScreen(BLACK);
    lastPage = currentPage;
    isStatusShowing = false;
    lastStatusMsg = "";
  }

  int w = 172;
  static String lastTimeStr = "";
  if (pageChanged || timeStr != lastTimeStr) {
    lcd->setTextColor(LIGHTGREY, BLACK);
    lcd->setTextSize(1);
    lcd->setCursor((w - (10 * 6)) / 2, 5);
    lcd->print(timeStr.substring(0, 10));
    lcd->setTextColor(WHITE, BLACK);
    lcd->setTextSize(2);
    lcd->setCursor((w - (8 * 12)) / 2, 20);
    lcd->print(timeStr.substring(11, 19));
    lastTimeStr = timeStr;
  }

  if (currentPage == 0) drawPage1_Tower(w, sensorList, deviceCount, currentLux, pageChanged);
  else drawPage2_Buttons(w, pageChanged);

  int infoY = 290;
  int cpuFake = random(1, 5);
  static int lastRam = -1;
  static int lastCpu = -1;
  int currentRam = (int)ramPercent;

  if (pageChanged || currentRam != lastRam || cpuFake != lastCpu) {
    char sysBuf[40];
    sprintf(sysBuf, "RAM:%02d%% CPU:%02d%%", currentRam, cpuFake);
    int sysX = (w - (strlen(sysBuf) * 6)) / 2;
    lcd->setTextColor(LIGHTGREY, BLACK);
    lcd->setTextSize(1);
    lcd->setCursor(sysX, infoY);
    lcd->print(sysBuf);
    lastRam = currentRam;
    lastCpu = cpuFake;
  }

  int dotY = 280;
  uint16_t c1 = (currentPage == 0) ? WHITE : DARKGREY;
  uint16_t c2 = (currentPage == 1) ? WHITE : DARKGREY;
  if (pageChanged) {
    lcd->fillCircle(w / 2 - 6, dotY, 3, c1);
    lcd->fillCircle(w / 2 + 6, dotY, 3, c2);
  }

  if (isStatusShowing && (millis() - statusMsgStartTime > 3000)) {
    lcd->fillRect(0, 305, 172, 15, BLACK);
    isStatusShowing = false;
    lastStatusMsg = "";
  }
}

void showStatusMessage(String message) {
  if (isStatusShowing && message == lastStatusMsg) {
    statusMsgStartTime = millis();
    return;
  }
  int footerY = 305;
  lcd->fillRect(0, footerY, 172, 15, BLACK);
  lcd->setTextSize(1);
  lcd->setTextColor(WHITE);
  lcd->setCursor(5, footerY + 2);
  lcd->print(message);
  lastStatusMsg = message;
  statusMsgStartTime = millis();
  isStatusShowing = true;
}

#endif