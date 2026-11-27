
#include <Arduino.h>


const uint8_t TRIG_PIN = 12; 
const uint8_t ECHO_PIN = 11;  
const uint8_t RAIN_PIN = 2;   


const float MM_PER_TIP       = 0.2794;  
const float SENSOR_HEIGHT_CM = 300.0;   
const float SPEED_OF_SOUND   = 0.0343;  

const unsigned long WINDOW_MS   = 300000UL;  
const unsigned long REPORT_MS   = 10000UL;   
const unsigned long DEBOUNCE_MS = 100UL;     


const float MIN_CM = 25.0;
const float MAX_CM = 450.0;
const uint8_t SAMPLES = 5;                  

const uint8_t MAX_TIPS = 100;
volatile unsigned long tipTimes[MAX_TIPS];
volatile uint8_t tipHead = 0;             
volatile uint8_t tipsStored = 0;            
volatile unsigned long totalTips = 0;      
volatile unsigned long lastTipMs = 0;
volatile bool lastTipSeen = false;

unsigned long lastReportMs = 0;

void onTip() {
  unsigned long now = millis();
  if (lastTipSeen && now - lastTipMs < DEBOUNCE_MS) return;
  lastTipSeen = true;
  lastTipMs = now;

  tipTimes[tipHead] = now;
  tipHead = (tipHead + 1) % MAX_TIPS;
  if (tipsStored < MAX_TIPS) tipsStored++;
  totalTips++;
}

float rainIntensityMmH() {
  unsigned int inWindow = 0;

  noInterrupts();
  unsigned long now = millis();
  for (uint8_t i = 0; i < tipsStored; i++) {
    uint8_t idx = (tipHead + MAX_TIPS - 1 - i) % MAX_TIPS;  
    if (now - tipTimes[idx] <= WINDOW_MS) inWindow++;
    else break;
  }
  interrupts();

  return inWindow * MM_PER_TIP * (3600000.0 / WINDOW_MS);
}

float totalRainMm() {
  noInterrupts();
  unsigned long tips = totalTips;
  interrupts();
  return tips * MM_PER_TIP;
}


float readOnceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(20);                       // 20 us is safer for JSN-SR04T-2.0
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);  // timeout 30 ms
  if (duration == 0) return -1;

  float cm = duration * SPEED_OF_SOUND / 2.0;
  if (cm < MIN_CM || cm > MAX_CM) return -1;   // outside sensor range
  return cm;
}

// Median of several readings to ignore spikes from rain, wind, ripples
float readDistanceCm() {
  float values[SAMPLES];
  uint8_t n = 0;

  for (uint8_t i = 0; i < SAMPLES; i++) {
    float cm = readOnceCm();
    if (cm > 0) values[n++] = cm;
    delay(60);                                 // let echoes fade between pings
  }
  if (n == 0) return -1;

  // insertion sort
  for (uint8_t i = 1; i < n; i++) {
    float key = values[i];
    uint8_t j = i;
    while (j > 0 && values[j - 1] > key) {
      values[j] = values[j - 1];
      j--;
    }
    values[j] = key;
  }
  return values[n / 2];
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT_PULLUP);             // reed switch connects pin to GND
#if defined(ARDUINO_ARCH_AVR)
  attachInterrupt(0, onTip, FALLING);          // RAIN_PIN 2 is external interrupt 0 on AVR boards
#else
  attachInterrupt(RAIN_PIN, onTip, FALLING);
#endif
}

void loop() {
  if (millis() - lastReportMs >= REPORT_MS) {
    lastReportMs = millis();

    float intensity = rainIntensityMmH();
    float totalMm = totalRainMm();
    float distance = readDistanceCm();

    Serial.print("{\"rain_mm_h\":");
    Serial.print(intensity, 2);
    Serial.print(",\"rain_total_mm\":");
    Serial.print(totalMm, 2);
    Serial.print(",\"distance_cm\":");
    if (distance < 0) Serial.print("null");
    else Serial.print(distance, 1);
    Serial.print(",\"level_cm\":");
    if (distance < 0) Serial.print("null");
    else {
      float level = SENSOR_HEIGHT_CM - distance;
      if (level < 0) level = 0;
      Serial.print(level, 1);
    }
    Serial.println("}");
  }
}