// ESP32 sensor dashboard: MQ3 alcohol/gas sensor (D32, analog) + touch sensor (D33, digital)
// Dashboard starts locked; touching the sensor unlocks it, and it auto re-locks after a
// configurable idle time. Alcohol level (Sober/Caution/Drunk) and the auto re-lock time are
// editable from a Settings panel and persisted to flash (survive reboot/power loss).

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "index_html.h"

// ---- Wi-Fi credentials: fill these in before uploading ----
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ---- Pins ----
const int GAS_PIN = 32;    // MQ3 analog output
const int TOUCH_PIN = 33;  // Touch sensor digital output (e.g. TTP223)

// If you're instead using the ESP32's native capacitive touch pad on this pin
// (no external touch module), set this to true and touchRead()/threshold is used.
const bool USE_NATIVE_TOUCH = false;
const int NATIVE_TOUCH_THRESHOLD = 40; // lower reading = touched, tune experimentally

// ---- Defaults (used only the first time, then overwritten by saved settings) ----
const int DEFAULT_SOBER_MAX = 1200;    // gas raw < this  => Sober
const int DEFAULT_CAUTION_MAX = 2500;  // gas raw < this  => Caution, else Drunk
const int DEFAULT_RELOCK_SECONDS = 30;

WebServer server(80);
Preferences prefs;

int gasRaw = 0;
bool touchedNow = false;
bool lastTouchState = false;

bool locked = true;
unsigned long unlockedAtMs = 0;

int soberMax;
int cautionMax;
int relockSeconds;
String cameraUrl;

void loadSettings() {
  prefs.begin("sensorapp", false);
  soberMax = prefs.getInt("soberMax", DEFAULT_SOBER_MAX);
  cautionMax = prefs.getInt("cautionMax", DEFAULT_CAUTION_MAX);
  relockSeconds = prefs.getInt("relock", DEFAULT_RELOCK_SECONDS);
  cameraUrl = prefs.getString("camUrl", "");
}

void saveSettings() {
  prefs.putInt("soberMax", soberMax);
  prefs.putInt("cautionMax", cautionMax);
  prefs.putInt("relock", relockSeconds);
  prefs.putString("camUrl", cameraUrl);
}

String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\\' || c == '"') out += '\\';
    out += c;
  }
  return out;
}

String alcoholState() {
  if (gasRaw < soberMax) return "Sober";
  if (gasRaw < cautionMax) return "Caution";
  return "Drunk";
}

void readSensors() {
  gasRaw = analogRead(GAS_PIN); // 0-4095 on ESP32 (12-bit ADC)

  if (USE_NATIVE_TOUCH) {
    touchedNow = touchRead(TOUCH_PIN) < NATIVE_TOUCH_THRESHOLD;
  } else {
    touchedNow = digitalRead(TOUCH_PIN) == HIGH;
  }
}

void updateLockState() {
  // Unlock on the rising edge of a touch (only while locked). Re-lock is time based.
  if (touchedNow && !lastTouchState && locked) {
    locked = false;
    unlockedAtMs = millis();
  }
  lastTouchState = touchedNow;

  if (!locked && (millis() - unlockedAtMs > (unsigned long)relockSeconds * 1000UL)) {
    locked = true;
  }
}

void handleData() {
  readSensors();
  updateLockState();

  int gasPercent = (gasRaw * 100) / 4095;
  long remaining = 0;
  if (!locked) {
    long elapsedSec = (millis() - unlockedAtMs) / 1000;
    remaining = relockSeconds - elapsedSec;
    if (remaining < 0) remaining = 0;
  }

  String json = "{";
  json += "\"gas\":" + String(gasRaw) + ",";
  json += "\"gasPercent\":" + String(gasPercent) + ",";
  json += "\"touch\":" + String(touchedNow ? "true" : "false") + ",";
  json += "\"locked\":" + String(locked ? "true" : "false") + ",";
  json += "\"unlockRemaining\":" + String(remaining) + ",";
  json += "\"alcoholState\":\"" + alcoholState() + "\",";
  json += "\"soberMax\":" + String(soberMax) + ",";
  json += "\"cautionMax\":" + String(cautionMax) + ",";
  json += "\"relockSeconds\":" + String(relockSeconds) + ",";
  json += "\"cameraUrl\":\"" + jsonEscape(cameraUrl) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleLockNow() {
  locked = true;
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSaveSettings() {
  if (!server.hasArg("soberMax") || !server.hasArg("cautionMax") || !server.hasArg("relockSeconds")) {
    server.send(400, "application/json", "{\"error\":\"missing fields\"}");
    return;
  }
  int newSober = server.arg("soberMax").toInt();
  int newCaution = server.arg("cautionMax").toInt();
  int newRelock = server.arg("relockSeconds").toInt();
  String newCameraUrl = server.hasArg("cameraUrl") ? server.arg("cameraUrl") : "";
  newCameraUrl.trim();

  if (newSober < 0 || newCaution < 0 || newSober >= newCaution) {
    server.send(400, "application/json", "{\"error\":\"soberMax must be less than cautionMax, and both non-negative\"}");
    return;
  }
  if (newRelock < 1) {
    server.send(400, "application/json", "{\"error\":\"relockSeconds must be at least 1\"}");
    return;
  }
  if (newCameraUrl.length() > 0 && !newCameraUrl.startsWith("http://") && !newCameraUrl.startsWith("https://")) {
    server.send(400, "application/json", "{\"error\":\"camera URL must start with http:// or https://\"}");
    return;
  }

  soberMax = newSober;
  cautionMax = newCaution;
  relockSeconds = newRelock;
  cameraUrl = newCameraUrl;
  saveSettings();

  server.send(200, "application/json", "{\"ok\":true}");
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void setup() {
  Serial.begin(115200);

  pinMode(TOUCH_PIN, INPUT);
  loadSettings();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. Open the dashboard at: http://");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/lock", HTTP_POST, handleLockNow);
  server.on("/settings", HTTP_POST, handleSaveSettings);
  server.begin();
}

void loop() {
  server.handleClient();
  readSensors();
  updateLockState();
}
