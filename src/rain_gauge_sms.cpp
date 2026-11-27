#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_SR.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

// ---------------------------- Configuration ----------------------------
const float MM_PER_TIP = 0.2169f;            // 0.00854 in/tip from the original build

// Rain thresholds (mm/h). Heavy rain starts at 7.6 mm/h.
const float RAIN_START_MMH = 7.6f;           // water sensor turns ON at/above this
const float RAIN_STOP_MMH  = 5.0f;           // water sensor turns OFF below this (hysteresis)

// Water level geometry and alert thresholds (cm)
const float SENSOR_TO_BOTTOM_CM = 100.0f;    // sensor face to empty bottom
const float ALERT_LEVEL_CM      = 70.0f;     // send SMS at/above this level
const float CLEAR_LEVEL_CM      = 65.0f;     // alert re-arms after dropping below this
const uint8_t CONFIRM_READINGS  = 3;         // consecutive high readings before SMS
const unsigned long SMS_COOLDOWN_MS = 15UL * 60UL * 1000UL;   // reminder interval while still high

const float SENSOR_MIN_CM = 20.0f;           // JSN-SR04T blind zone
const float SENSOR_MAX_CM = 600.0f;

const unsigned long BUCKET_DEBOUNCE_MS = 150;
const unsigned long MENU_DEBOUNCE_MS   = 300;
const unsigned long BACKLIGHT_MS       = 30000UL;
const unsigned long LCD_REFRESH_MS     = 500;
const unsigned long LEVEL_READ_MS      = 2000;
const unsigned long STATUS_LOG_MS      = 60000UL;
const uint8_t INTENSITY_WINDOW_MIN     = 10;

// ------------------------------- Pins ----------------------------------
const uint8_t PIN_MENU      = 2;
const uint8_t PIN_BUCKET    = 3;
const uint8_t PIN_SD_CS     = 4;
const uint8_t PIN_BACKLIGHT = 7;
const uint8_t PIN_TRIG      = A0;
const uint8_t PIN_ECHO      = A1;
const uint8_t PIN_GW_RX     = 8;
const uint8_t PIN_GW_TX     = A2;

// ------------------------------ Objects --------------------------------
LiquidCrystal_SR lcd(6, 5, 9);
RTC_DS1307 rtc;                  // use RTC_DS3231 if your module has that chip
SoftwareSerial gw(PIN_GW_RX, PIN_GW_TX);
File logFile;

// --------------------------- ISR shared state --------------------------
volatile uint16_t isrPendingTips = 0;
volatile bool     isrMenuPressed = false;

// ------------------------------ State ----------------------------------
bool sdOk = false;

uint16_t minuteTips[60];
uint8_t  lastMinute = 255, lastHour = 255, lastDay = 255;
uint32_t hourTips = 0, prevHourTips = 0, dayTips = 0, prevDayTips = 0;

// Gate / alert state
bool     sensorActive  = false;
float    distanceCm    = -1.0f;
float    levelCm       = -1.0f;
uint8_t  highCount     = 0;
bool     alertLatched  = false;
unsigned long lastSmsMs = 0;
uint8_t  smsState      = 0;      // 0 none, 1 sent/pending, 2 gateway OK, 3 gateway error

uint8_t page = 0;
const uint8_t PAGE_COUNT = 4;
unsigned long backlightOffAt = 0, tLcd = 0, tLevel = 0, tStatus = 0;

const char LOG_NAME[] = "RAINLOG.CSV";

// ------------------------------- ISRs ----------------------------------
void onBucketTip() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last > BUCKET_DEBOUNCE_MS) { isrPendingTips++; last = now; }
}

void onMenuPress() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last > MENU_DEBOUNCE_MS) { isrMenuPressed = true; last = now; }
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
  if (mmh <= 0.0f) return "No rain";
  if (mmh < 2.5f)  return "Light";
  if (mmh < 7.6f)  return "Moderate";
  if (mmh < 50.0f) return "Heavy";
  return "Violent";
}

// ------------------------- JSN-SR04T ultrasonic ------------------------
float readDistanceOnceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(12);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 38000UL);
  if (us == 0) return -1.0f;
  float cm = (us * 0.0343f) / 2.0f;
  if (cm < SENSOR_MIN_CM || cm > SENSOR_MAX_CM) return -1.0f;
  return cm;
}

float readDistanceMedianCm() {
  float v[5];
  uint8_t count = 0;
  for (uint8_t i = 0; i < 5; i++) {
    float d = readDistanceOnceCm();
    if (d > 0) v[count++] = d;
    delay(60);
  }
  if (count < 3) return -1.0f;
  for (uint8_t i = 1; i < count; i++) {
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

// Simple marker rows: SENSOR_ON, SENSOR_OFF, SMS_REQUESTED, SMS_OK, SMS_ERR
void logEvent(const __FlashStringHelper *tag, const DateTime &t, float mmh) {
  if (!sdOk) return;
  char ts[24];
  formatTimestamp(t, ts, sizeof(ts));
  logFile = SD.open(LOG_NAME, FILE_WRITE);
  if (logFile) {
    logFile.print(tag);  logFile.print(',');
    logFile.print(ts);   logFile.print(F(",0,,"));
    logFile.print(mmh, 2); logFile.print(',');
    logFile.print(tipsToMm(hourTips), 2); logFile.print(',');
    logFile.print(tipsToMm(dayTips), 2);  logFile.print(',');
    if (levelCm >= 0) logFile.print(levelCm, 1);
    logFile.println();
    logFile.close();
  }
}

void logTips(const DateTime &t, uint16_t tips, float mmh) {
  if (!sdOk) return;
  char ts[24];
  formatTimestamp(t, ts, sizeof(ts));
  logFile = SD.open(LOG_NAME, FILE_WRITE);
  if (logFile) {
    logFile.print(F("TIP,"));  logFile.print(ts); logFile.print(',');
    logFile.print(tips);       logFile.print(',');
    logFile.print(MM_PER_TIP, 4); logFile.print(',');
    logFile.print(mmh, 2);     logFile.print(',');
    logFile.print(tipsToMm(hourTips), 2); logFile.print(',');
    logFile.print(tipsToMm(dayTips), 2);  logFile.print(',');
    logFile.println();
    logFile.close();
  }
}

// ------------------------- Rain -> sensor gate -------------------------
void updateSensorGate(const DateTime &now, float mmh) {
  if (!sensorActive && mmh >= RAIN_START_MMH) {
    sensorActive = true;
    highCount = 0;
    tLevel = 0;                       // read immediately
    logEvent(F("SENSOR_ON"), now, mmh);
  } else if (sensorActive && mmh < RAIN_STOP_MMH) {
    sensorActive = false;
    distanceCm = -1.0f;
    levelCm = -1.0f;
    highCount = 0;
    alertLatched = false;
    logEvent(F("SENSOR_OFF"), now, mmh);
  }
}

// ------------------------- Water level -> SMS --------------------------
void sendSms(const DateTime &now, float mmh) {
  char lv[10], rn[10], msg[120];
  dtostrf(levelCm, 1, 1, lv);
  dtostrf(mmh, 1, 1, rn);
  snprintf(msg, sizeof(msg), "FLOOD ALERT: water level %s cm (limit %d). Rain %s mm/h. %02u:%02u %02u/%02u",
           lv, (int)ALERT_LEVEL_CM, rn, now.hour(), now.minute(), now.month(), now.day());
  gw.print(F("SMS:"));
  gw.println(msg);
  smsState = 1;
  logEvent(F("SMS_REQUESTED"), now, mmh);
}

void checkAlert(const DateTime &now, float mmh, unsigned long ms) {
  if (levelCm < 0) return;            // no valid reading, do nothing

  if (levelCm >= ALERT_LEVEL_CM) {
    if (highCount < 255) highCount++;
  } else {
    highCount = 0;
    if (levelCm < CLEAR_LEVEL_CM) alertLatched = false;
  }

  if (highCount >= CONFIRM_READINGS) {
    bool cooldownOver = (lastSmsMs == 0) || (ms - lastSmsMs >= SMS_COOLDOWN_MS);
    if (!alertLatched || cooldownOver) {
      sendSms(now, mmh);
      alertLatched = true;
      lastSmsMs = (ms == 0) ? 1 : ms;
    }
  }
}

// Reads "ACK:OK" / "ACK:ERR" lines coming back from the ESP8266
void pollGateway(const DateTime &now, float mmh) {
  static char buf[20];
  static uint8_t n = 0;
  while (gw.available()) {
    char c = gw.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) {
        buf[n] = 0;
        if (strcmp(buf, "ACK:OK") == 0) { smsState = 2; logEvent(F("SMS_OK"), now, mmh); }
        else if (strncmp(buf, "ACK:ERR", 7) == 0) { smsState = 3; logEvent(F("SMS_ERR"), now, mmh); }
        n = 0;
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

// ------------------------------- Display -------------------------------
void drawPage(float mmh) {
  char l0[17], l1[17], a[10], b[10];
  const char *smsLabel[] = { "--", "..", "ok", "ERR" };

  switch (page) {
    case 0:
      dtostrf(mmh, 1, 1, a);
      snprintf(l0, sizeof(l0), "Rain %s mm/h", a);
      snprintf(l1, sizeof(l1), "%-8s S:%s", rainCategory(mmh), sensorActive ? "ON" : "OFF");
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
      if (!sensorActive) {
        snprintf(l0, sizeof(l0), "Water sensor");
        snprintf(l1, sizeof(l1), "OFF - low rain");
      } else if (levelCm < 0) {
        snprintf(l0, sizeof(l0), "Water level");
        snprintf(l1, sizeof(l1), "No echo");
      } else {
        dtostrf(levelCm, 1, 1, a);
        snprintf(l0, sizeof(l0), "Level %s cm", a);
        snprintf(l1, sizeof(l1), "%-4s SMS:%s", (levelCm >= ALERT_LEVEL_CM) ? "HIGH" : "OK", smsLabel[smsState]);
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
  pinMode(10, OUTPUT);                     // SPI SS must be an output for the SD library

  gw.begin(9600);

  lcd.begin(16, 2);
  lcd.home();
  lcdLine(0, "Rain Monitor");

  Wire.begin();
  if (!rtc.begin()) {
    lcdLine(1, "RTC not found");
    while (true) { delay(1000); }
  }
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  sdOk = SD.begin(PIN_SD_CS);
  lcdLine(1, sdOk ? "SD ready" : "No SD card");
  logHeaderIfNew();

  pinMode(PIN_BUCKET, INPUT_PULLUP);
  pinMode(PIN_MENU, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUCKET), onBucketTip, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_MENU), onMenuPress, RISING);

  memset(minuteTips, 0, sizeof(minuteTips));
  DateTime now = rtc.now();
  lastMinute = now.minute();
  lastHour   = now.hour();
  lastDay    = now.day();

  // NOTE: the water sensor is intentionally NOT read at startup.
  delay(1500);
}

// --------------------------------- Loop --------------------------------
void loop() {
  unsigned long ms = millis();
  DateTime now = rtc.now();

  // Time rollovers
  if (now.minute() != lastMinute) {
    lastMinute = now.minute();
    minuteTips[lastMinute] = 0;
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

  // Bucket tips
  noInterrupts();
  uint16_t tips = isrPendingTips;
  isrPendingTips = 0;
  interrupts();

  if (tips > 0) {
    minuteTips[now.minute()] += tips;
    hourTips += tips;
    dayTips  += tips;
  }

  float mmh = intensityMmPerHour(now.minute());
  if (tips > 0) logTips(now, tips, mmh);

  // STEP 1: rain intensity decides if the water sensor is allowed to run
  updateSensorGate(now, mmh);

  // STEP 2: water sensor only runs while heavy rain is active
  if (sensorActive && (ms - tLevel >= LEVEL_READ_MS)) {
    tLevel = ms;
    updateWaterLevel();
    // STEP 3: SMS only when the level is high
    checkAlert(now, mmh, ms);
  }

  pollGateway(now, mmh);

  // Menu button
  if (isrMenuPressed) {
    isrMenuPressed = false;
    if (digitalRead(PIN_BACKLIGHT) == LOW) {
      digitalWrite(PIN_BACKLIGHT, HIGH);
    } else {
      page = (page + 1) % PAGE_COUNT;
    }
    backlightOffAt = ms + BACKLIGHT_MS;
    tLcd = 0;
  }
  if ((long)(ms - backlightOffAt) > 0) digitalWrite(PIN_BACKLIGHT, LOW);

  // LCD refresh
  if (ms - tLcd >= LCD_REFRESH_MS) {
    tLcd = ms;
    drawPage(mmh);
  }

  // Periodic status row
  if (ms - tStatus >= STATUS_LOG_MS) {
    tStatus = ms;
    logEvent(F("STATUS"), now, mmh);
  }
}
