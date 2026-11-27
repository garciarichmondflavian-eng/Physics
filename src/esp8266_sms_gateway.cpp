// ESP8266 SMS gateway (IPROG SMS)
// Receives "SMS:<text>" lines from the Arduino Nano over serial, sends them
// through the IPROG SMS API, and replies "ACK:OK" or "ACK:ERR".
// Flash this to the ESP8266 board (NodeMCU / Wemos D1 mini), NOT the Nano.

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>
#include "secrets.private_keys.h"
      
const char *URL_SINGLE = "https://www.iprogsms.com/api/v1/sms_messages";
const char *URL_BULK   = "https://www.iprogsms.com/api/v1/sms_messages/send_bulk";

const uint8_t  SEND_RETRIES = 3;
const uint16_t LINE_MAX     = 160;
const uint16_t HTTP_TIMEOUT = 10000;                  // ms

SoftwareSerial link(D5, D6);                          // RX, TX

// Result of one send attempt
enum SendResult { SEND_OK, SEND_RETRY, SEND_FATAL };

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

// One attempt to send the SMS through IPROG SMS.
SendResult postSms(const String &message) {
  connectWifi();
  if (WiFi.status() != WL_CONNECTED) return SEND_RETRY;

  // Several comma-separated numbers -> bulk endpoint, otherwise single endpoint
  bool bulk = strchr(PHONE_NUMS, ',') != nullptr;
  const char *url = bulk ? URL_BULK : URL_SINGLE;

  WiFiClientSecure client;
  client.setInsecure();                               // skips certificate check; fine for a prototype
  client.setBufferSizes(512, 512);                    // saves RAM on the ESP8266
  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT);
  if (!http.begin(client, url)) return SEND_RETRY;
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String body = "api_token=" + urlEncode(API_TOKEN) +
                "&phone_number=" + urlEncode(PHONE_NUMS) +
                "&message=" + urlEncode(message);

  int code = http.POST(body);
  String resp = (code > 0) ? http.getString() : String();
  http.end();

  Serial.printf("HTTP %d\n", code);
  Serial.println(resp);

  if (code <= 0 || code >= 500) return SEND_RETRY;    // network problem or server error

  // IPROG SMS answers {"status":200,"message":"...queued...","message_id":"iSms-..."}
  // Errors look like {"status":500,"message":"Invalid Token"}
  resp.replace(" ", "");
  resp.replace("\n", "");
  resp.replace("\r", "");
  if (code == 200 && resp.indexOf("\"status\":200") >= 0) return SEND_OK;

  return SEND_FATAL;                                  // rejected by the API; retrying won't help
}

bool sendWithRetry(const String &message) {
  for (uint8_t i = 0; i < SEND_RETRIES; i++) {
    SendResult r = postSms(message);
    if (r == SEND_OK) return true;
    if (r == SEND_FATAL) return false;
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
