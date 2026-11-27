

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>

// ----------------------------- Settings --------------------------------
const char *WIFI_SSID = "YOUR_WIFI_NAME";
const char *WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char *API_URL    = "https://api.semaphore.co/api/v4/messages";
const char *API_KEY    = "YOUR_SEMAPHORE_API_KEY";
const char *PHONE_NUMS = "09171234567";         // several numbers: "0917xxxxxxx,0918xxxxxxx"
const char *SENDER     = "";                    // optional approved sender name, leave "" for default

const uint8_t  SEND_RETRIES = 3;
const uint16_t LINE_MAX     = 160;

SoftwareSerial link(D5, D6);                    // RX, TX

// ------------------------------ Helpers --------------------------------
String urlEncode(const String &s) {
  String out;
  const char *hex = "0123456789ABCDEF";
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += c;
    } else {
      out += '%';
      out += hex[(c >> 4) & 0x0F];
      out += hex[c & 0x0F];
    }
  }
  return out;
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print(F("Connecting WiFi"));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL) {
    delay(500);
    Serial.print('.');
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? F(" connected") : F(" failed"));
}

bool postSms(const String &message) {
  connectWifi();
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();                          // skips certificate check; fine for a hobby project
  HTTPClient http;
  if (!http.begin(client, API_URL)) return false;
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = "apikey=" + urlEncode(API_KEY) +
                "&number=" + urlEncode(PHONE_NUMS) +
                "&message=" + urlEncode(message);
  if (strlen(SENDER) > 0) body += "&sendername=" + urlEncode(SENDER);

  int code = http.POST(body);
  Serial.printf("HTTP %d\n", code);
  if (code > 0) Serial.println(http.getString());
  http.end();
  return code >= 200 && code < 300;
}

bool sendWithRetry(const String &message) {
  for (uint8_t i = 0; i < SEND_RETRIES; i++) {
    if (postSms(message)) return true;
    delay(2000);
  }
  return false;
}

// -------------------------------- Setup --------------------------------
void setup() {
  Serial.begin(115200);
  link.begin(9600);
  connectWifi();
}

// --------------------------------- Loop --------------------------------
void loop() {
  static String line;

  while (link.available()) {
    char c = link.read();
    if (c == '\n' || c == '\r') {
      if (line.startsWith("SMS:")) {
        String msg = line.substring(4);
        msg.trim();
        Serial.println("Sending: " + msg);
        link.println(sendWithRetry(msg) ? F("ACK:OK") : F("ACK:ERR"));
      }
      line = "";
    } else if (line.length() < LINE_MAX) {
      line += c;
    }
  }

  if (WiFi.status() != WL_CONNECTED) connectWifi();
}
