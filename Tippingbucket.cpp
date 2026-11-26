
// Arduino core types and built-in Wire/SPI support (required for a .cpp sketch file)
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_SR3W.h>

// ---------------------------- Configuration ----------------------------
// Rain gauge calibration: original is 0.00854 in/tip = 0.2169 mm/tip
// (bucket ~2 mL, collector 16.605 sq.in.). Calibrate with a known water volume.
const float MM_PER_TIP = 0.2169f;

// Water level geometry: distance from the sensor face to the empty bottom (cm)
const float SENSOR_TO_BOTTOM_CM = 100.0f;
const float WARN_LEVEL_CM       = 70.0f;     // shows "HIGH" when level is at/above this
const float SENSOR_MIN_CM       = 20.0f;     // JSN-SR04T blind zone
const float SENSOR_MAX_CM       = 600.0f;

const unsigned long BUCKET_DEBOUNCE_MS = 150;
const unsigned long MENU_DEBOUNCE_MS   = 300;
const unsigned long BACKLIGHT_MS       = 30000UL;
const unsigned long LCD_REFRESH_MS     = 500;
const unsigned long LEVEL_READ_MS      = 2000;
const unsigned long STATUS_LOG_MS      = 60000UL;

const uint8_t INTENSITY_WINDOW_MIN = 10;     // rolling window for mm/h

// ------------------------------- Pins ----------------------------------
const uint8_t PIN_MENU      = 2;
const uint8_t PIN_BUCKET    = 3;
const uint8_t PIN_SD_CS     = 4;
const uint8_t PIN_BACKLIGHT = 7;
const uint8_t PIN_TRIG      = A0;
const uint8_t PIN_ECHO      = A1;

// ------------------------------ Objects --------------------------------
LiquidCrystal_SR3W lcd(6, 5, 9); // data, clock, strobe
RTC_DS1307 rtc;                  // Tiny RTC module (use RTC_DS3231 if you have that chip)
File logFile;

// --------------------------- ISR shared state --------------------------
volatile uint16_t isrPendingTips = 0;
volatile bool     isrMenuPressed = false;

// ------------------------------ State ----------------------------------
bool sdOk = false;

uint16_t minuteTips[60];         // tips per minute-of-hour (ring)
uint8_t  lastMinute = 255;
uint8_t  lastHour   = 255;
uint8_t  lastDay    = 255;

uint32_t hourTips = 0, prevHourTips = 0;
uint32_t dayTips  = 0, prevDayTips  = 0;

float distanceCm = -1.0f;        // < 0 means no valid reading
float levelCm    = -1.0f;

uint8_t page = 0;
const uint8_t PAGE_COUNT = 4;

unsigned long backlightOffAt = 0;
unsigned long tLcd = 0, tLevel = 0, tStatus = 0;

const char LOG_NAME[] = "RAINLOG.CSV";

// ------------------------------- ISRs ----------------------------------
void onBucketTip() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last > BUCKET_DEBOUNCE_MS) {
    isrPendingTips++;
    last = now;
  }
}

void onMenuPress() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last > MENU_DEBOUNCE_MS) {
    isrMenuPressed = true;
    last = now;
  }
}

// ------------------------------ Helpers --------------------------------
void formatTimestamp(const DateTime &t, char *buf, size_t n) {
  snprintf(buf, n, "%04u-%02u-%02u %02u:%02u:%02u",
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
}

void lcdLine(uint8_t row, const char *text) {
  char line[17];
  snprintf(line, sizeof(line), "%-16s", text);
  lcd.setCursor(0, row);
  lcd.print(line);
}

float tipsToMm(uint32_t tips) { return tips * MM_PER_TIP; }

float intensityMmPerHour(uint8_t currentMinute) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < INTENSITY_WINDOW_MIN; i++) {
    sum += minuteTips[(currentMinute + 60 - i) % 60];
  }
  return tipsToMm(sum) * (60.0f / INTENSITY_WINDOW_MIN);
}

const char *rainCategory(float mmh) {
  if (mmh <= 0.0f)  return "No rain";
  if (mmh < 2.5f)   return "Light";
  if (mmh < 7.6f)   return "Moderate";
  if (mmh < 50.0f)  return "Heavy";
  return "Violent";
}

// ------------------------- JSN-SR04T ultrasonic ------------------------
float readDistanceOnceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(12);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, 38000UL);   // ~6.5 m timeout
  if (us == 0) return -1.0f;
  float cm = (us * 0.0343f) / 2.0f;                      // 343 m/s at ~20 C
  if (cm < SENSOR_MIN_CM || cm > SENSOR_MAX_CM) return -1.0f;
  return cm;
}

// Median of 5 valid readings rejects spikes from rain drops and splashes
float readDistanceMedianCm() {
  float v[5];
  uint8_t count = 0;
  for (uint8_t i = 0; i < 5; i++) {
    float d = readDistanceOnceCm();
    if (d > 0) v[count++] = d;
    delay(60);
  }
  if (count < 3) return -1.0f;
  for (uint8_t i = 1; i < count; i++) {           // insertion sort
    float key = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
    v[j + 1] = key;
  }
  return v[count / 2];
}

void updateWaterLevel() {
  distanceCm = readDistanceMedianCm();
  if (distanceCm > 0) {
    levelCm = SENSOR_TO_BOTTOM_CM - distanceCm;
    if (levelCm < 0) levelCm = 0;
  } else {
    levelCm = -1.0f;
  }
}

// ------------------------------ SD logging -----------------------------
void logHeaderIfNew() {
  if (!sdOk || SD.exists(LOG_NAME)) return;
  logFile = SD.open(LOG_NAME, FILE_WRITE);
  if (logFile) {
    logFile.println(F("type,timestamp,tips,mm_per_tip,intensity_mmh,hour_mm,day_mm,level_cm"));
    logFile.close();
  }
}

void logTips(const DateTime &t, uint16_t tips, float mmh) {
  if (!sdOk) return;
  char ts[24];
  formatTimestamp(t, ts, sizeof(ts));
  logFile = SD.open(LOG_NAME, FILE_WRITE);
  if (logFile) {
    logFile.print(F("TIP,"));  logFile.print(ts);       logFile.print(',');
    logFile.print(tips);       logFile.print(',');
    logFile.print(MM_PER_TIP, 4); logFile.print(',');
    logFile.print(mmh, 2);     logFile.print(',');
    logFile.print(tipsToMm(hourTips), 2); logFile.print(',');
    logFile.print(tipsToMm(dayTips), 2);  logFile.print(',');
    logFile.println();
    logFile.close();
  }
}

void logStatus(const DateTime &t, float mmh) {
  if (!sdOk) return;
  char ts[24];
  formatTimestamp(t, ts, sizeof(ts));
  logFile = SD.open(LOG_NAME, FILE_WRITE);
  if (logFile) {
    logFile.print(F("STATUS,")); logFile.print(ts); logFile.print(F(",0,,"));
    logFile.print(mmh, 2);      logFile.print(',');
    logFile.print(tipsToMm(hourTips), 2); logFile.print(',');
    logFile.print(tipsToMm(dayTips), 2);  logFile.print(',');
    if (levelCm >= 0) logFile.print(levelCm, 1);
    logFile.println();
    logFile.close();
  }
}

// ------------------------------- Display -------------------------------
void drawPage(const DateTime &now) {
  char l0[17], l1[17], a[10], b[10];
  float mmh = intensityMmPerHour(now.minute());

  switch (page) {
    case 0:
      dtostrf(mmh, 1, 1, a);
      snprintf(l0, sizeof(l0), "Rain %s mm/h", a);
      snprintf(l1, sizeof(l1), "%s", rainCategory(mmh));
      break;
    case 1:
      dtostrf(tipsToMm(hourTips), 1, 2, a);
      dtostrf(tipsToMm(prevHourTips), 1, 2, b);
      snprintf(l0, sizeof(l0), "Hour %s mm", a);
      snprintf(l1, sizeof(l1), "Prev %s mm", b);
      break;
    case 2:
      dtostrf(tipsToMm(dayTips), 1, 2, a);
      dtostrf(tipsToMm(prevDayTips), 1, 2, b);
      snprintf(l0, sizeof(l0), "Day  %s mm", a);
      snprintf(l1, sizeof(l1), "Prev %s mm", b);
      break;
    default:
      if (levelCm >= 0) {
        dtostrf(levelCm, 1, 1, a);
        dtostrf(distanceCm, 1, 1, b);
        snprintf(l0, sizeof(l0), "Level %s cm", a);
        snprintf(l1, sizeof(l1), "%s Dist %s", (levelCm >= WARN_LEVEL_CM) ? "HIGH" : "OK", b);
      } else {
        snprintf(l0, sizeof(l0), "Water level");
        snprintf(l1, sizeof(l1), "No echo");
      }
      break;
  }
  lcdLine(0, l0);
  lcdLine(1, l1);
}

// -------------------------------- Setup --------------------------------
void setup() {
  pinMode(PIN_BACKLIGHT, OUTPUT);
  digitalWrite(PIN_BACKLIGHT, HIGH);
  backlightOffAt = millis() + BACKLIGHT_MS;

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(10, OUTPUT);                     // SPI SS must be an output for SD library

  lcd.begin(16, 2);
  lcd.home();
  lcdLine(0, "Rain Monitor");

  Wire.begin();
  if (!rtc.begin()) {
    lcdLine(1, "RTC not found");
    while (true) { delay(1000); }
  }
  // First run or dead battery: set RTC to the sketch compile time
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  sdOk = SD.begin(PIN_SD_CS);
  lcdLine(1, sdOk ? "SD ready" : "No SD card");
  logHeaderIfNew();

  pinMode(PIN_BUCKET, INPUT_PULLUP);
  pinMode(PIN_MENU, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUCKET), onBucketTip, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_MENU), onMenuPress, RISING);   // fires on release, as in original

  memset(minuteTips, 0, sizeof(minuteTips));
  DateTime now = rtc.now();
  lastMinute = now.minute();
  lastHour   = now.hour();
  lastDay    = now.day();

  updateWaterLevel();
  delay(1500);
}

// --------------------------------- Loop --------------------------------
void loop() {
  unsigned long ms = millis();
  DateTime now = rtc.now();

  // Time rollovers (minute ring, hour, day)
  if (now.minute() != lastMinute) {
    lastMinute = now.minute();
    minuteTips[lastMinute] = 0;            // slot being reused starts empty
  }
  if (now.hour() != lastHour) {
    lastHour = now.hour();
    prevHourTips = hourTips;
    hourTips = 0;
  }
  if (now.day() != lastDay) {
    lastDay = now.day();
    prevDayTips = dayTips;
    dayTips = 0;
  }

  // Bucket tips (copy-and-clear atomically)
  noInterrupts();
  uint16_t tips = isrPendingTips;
  isrPendingTips = 0;
  interrupts();

  if (tips > 0) {
    minuteTips[now.minute()] += tips;
    hourTips += tips;
    dayTips  += tips;
    logTips(now, tips, intensityMmPerHour(now.minute()));
  }

  // Menu button
  if (isrMenuPressed) {
    isrMenuPressed = false;
    if (digitalRead(PIN_BACKLIGHT) == LOW) {
      digitalWrite(PIN_BACKLIGHT, HIGH);          // first press only wakes the display
    } else {
      page = (page + 1) % PAGE_COUNT;
    }
    backlightOffAt = ms + BACKLIGHT_MS;
    tLcd = 0;                                     // refresh immediately
  }

  // Backlight timeout
  if ((long)(ms - backlightOffAt) > 0) {
    digitalWrite(PIN_BACKLIGHT, LOW);
  }

  // Water level
  if (ms - tLevel >= LEVEL_READ_MS) {
    tLevel = ms;
    updateWaterLevel();
  }

  // LCD refresh
  if (ms - tLcd >= LCD_REFRESH_MS) {
    tLcd = ms;
    drawPage(now);
  }

  // Periodic status row in the CSV
  if (ms - tStatus >= STATUS_LOG_MS) {
    tStatus = ms;
    logStatus(now, intensityMmPerHour(now.minute()));
  }
}