/* ==========================================================================
 *  Project Helmet — Wi-Fi Telemetry Web Server
 *  Board: ESP32 DevKit V1 (30-pin, WROOM-32)
 *
 *  Joins the existing Wi-Fi network "robot" (password "12345678").
 *  The router hands out the IP, which the OLED shows. From a phone/laptop on
 *  the same network open:
 *    http://<ip>/            dashboard page
 *    http://<ip>/settings    RFID + alcohol-threshold settings
 *    http://helmet.local/    same, via mDNS (if your OS supports it)
 *    GET /data             JSON telemetry
 *  Everything is LOCKED until the registered RFID card is scanned (dashboard,
 *  data and OLED). If no card is registered yet, the OLED and the web page
 *  tell you to open /settings and register one.
 *  Camera + ignition (needs the ESP32-CAM sketch running on the same Wi-Fi):
 *    GET /api/rider-check  snapshot -> VLM agent (helmet / riders / drowsiness)
 *    GET /api/snapshot.jpg the frame that was analysed
 *    GET /api/cam-set?host=espcam        camera hostname or IP (blank = espcam)
 *  Ignition unlocks only when the rider check passed AND there is no alcohol.
 *    GET /api/lock-status  lock state (always open)
 *    GET /api/lock         lock again
 *    GET /api/speed-set?v=NN             speed limit in km/h
 *    GET /api/telegram-test              send a test message to the bot
 *    GET /api/accident-clear             acknowledge an accident alert
 *    GET /api/settings     current settings (JSON)
 *    GET /api/rfid-learn   next card scanned becomes the unlock card
 *    GET /api/rfid-clear   forget the unlock card
 *    GET /api/rfid-set?uid=AA:BB:CC:DD   set the unlock card by hand
 *    GET /api/gas-set?v=NNN              set the alcohol threshold (raw ADC)
 *    GET /api/zero-yaw     set current heading as yaw = 0
 *    GET /api/calibrate    re-measure gyro bias (hold helmet STILL)
 *    GET /api/reset-peak   clear peak-impact reading
 *    GET /api/beep         sound the buzzer
 *
 *  ATTITUDE NOTE
 *    roll + pitch  -> absolute. Gravity is the reference, so a complementary
 *                     filter keeps them stable indefinitely.
 *    yaw           -> RELATIVE and DRIFTING. The MPU6050 has no magnetometer,
 *                     so yaw is integrated Z-gyro only. Expect a few degrees
 *                     of drift per minute. Use /api/zero-yaw to re-reference.
 *                     "gps.course" is a TRUE heading, but only while moving.
 *
 *  Libraries: MFRC522, Adafruit SSD1306 + GFX, Adafruit MPU6050 +
 *             Unified Sensor, TinyGPSPlus. (WiFi/WebServer/ESPmDNS/Preferences are core.)
 * ========================================================================== */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>



// ===========================  WI-FI  =====================================
const char* WIFI_SSID = "robot";
const char* WIFI_PASS = "12345678";
const char* HOSTNAME  = "helmet";            // -> http://helmet.local
// =========================================================================

// ===========================  TELEGRAM  ==================================
// The ESP32's Wi-Fi network needs internet access for the accident alert.
const char* BOT_TOKEN = "8853140495:AAH20k2Dk7aiaqQMJiYEOWy2sfTQGAcsdyE";
const char* CHAT_ID   = "1935379929";        // start a chat with your bot first!
// =========================================================================

// ===========================  CAMERA + VLM AGENT  ========================
// The VLM agent runs on the DigitalOcean droplet and holds the Anthropic key.
// This device only knows the (revocable) device key below.
const char* VLM_URL  = "http://165.22.51.212:8000/analyze";
const char* VLM_KEY  = "af2dcfd6ae8f6cde369a06acd94f409b";
const char* CAM_DEFAULT_HOST = "espcam";     // ESP32-CAM sketch advertises espcam.local
const uint8_t  MAX_RIDERS       = 2;         // more than this many people -> ignition stays locked
const uint32_t CHECK_VALID_MS   = 10UL * 60UL * 1000UL;  // a passed check expires after 10 min
const size_t   MAX_SNAP_BYTES   = 120000;
#define IGNITION_PIN           -1            // optional relay GPIO (-1 = UI/OLED only)
#define IGNITION_ACTIVE_HIGH   true
// =========================================================================

// ---------------- Pin map (matches HelmetSelfTest) ----------------
#define I2C_SDA       21
#define I2C_SCL       22

#define RFID_SS        5
#define RFID_RST      27
#define RFID_SCK      18
#define RFID_MOSI     23
#define RFID_MISO     19

#define GPS_RX        16
#define GPS_TX        17
#define GPS_BAUD    9600

#define GAS_PIN       32
#define TOUCH_PIN     33
#define LDR_PIN       34
#define BUZZER_PIN    25

// ---------------- Config ----------------
#define OLED_W       128
#define OLED_H        64
#define MPU_ADDR    0x68

#define BUZZER_ACTIVE_LOW  false
#define LDR_DARK_IS_LOW    false     // this board's D0 goes HIGH in darkness
#define TOUCH_ACTIVE_HIGH  true

const uint16_t GAS_DEFAULT_THRESHOLD = 400;  // used until you set one in /settings
const uint16_t SPEED_DEFAULT_LIMIT   = 60;   // km/h, until you set one in /settings

// Accident detection (tune to taste)
const float    ACCIDENT_TILT_DEG = 75.0f;    // |roll| or |pitch| beyond this = fallen over...
const uint32_t ACCIDENT_TILT_MS  = 2000;     // ...for at least this long
const float    ACCIDENT_IMPACT_G = 6.0f;     // ...or one single hit this hard
const uint32_t ACCIDENT_REARM_MS = 60000;    // min time before an alert can clear itself
const float    COMP_ALPHA    = 0.98f;  // gyro trust in the complementary filter
const uint32_t IMU_MS        = 10;     // 100 Hz attitude update
const uint32_t OLED_MS       = 500;
const uint32_t RFID_POLL_MS  = 150;

// ---------------- Objects ----------------
WebServer          server(80);
MFRC522            rfid(RFID_SS, RFID_RST);
Adafruit_SSD1306   oled(OLED_W, OLED_H, &Wire, -1);
Adafruit_MPU6050   mpu;
TinyGPSPlus        gps;

// ---------------- State ----------------
enum MpuMode { MPU_NONE, MPU_ADAFRUIT, MPU_RAW };
MpuMode mpuMode   = MPU_NONE;
uint8_t mpuWhoAmI = 0xFF;

// Must stay above the first function: Arduino injects prototypes there.
struct Motion { float ax, ay, az, gx, gy, gz, tempC; bool valid; };

bool     oledOK = false, rfidOK = false;
uint8_t  oledAddr = 0, rfidVersion = 0;

float roll = 0, pitch = 0, yaw = 0;      // degrees
float yawOffset = 0;
float gBias[3] = {0, 0, 0};              // rad/s, removed before integration
float peakG = 0;                         // peak |a| in g, for impact spotting
uint32_t lastImu = 0, lastOled = 0, lastRfidPoll = 0, lastGasBeep = 0, lastWifiTry = 0;
uint32_t beepUntil = 0;
bool     lastTouch = false;
uint32_t cardCount = 0, lastCardMs = 0;
String   lastUid = "none";

// ---- Persistent settings (saved in flash via Preferences) ----
Preferences prefs;
String   rfidUid = "";                   // unlock card; empty = NOT SET
uint16_t gasThreshold = GAS_DEFAULT_THRESHOLD;
bool     rfidLearn = false;              // next scanned card becomes the unlock card
uint32_t rfidLearnUntil = 0;
const uint32_t RFID_LEARN_MS = 20000;

// ---- Lock state ----
bool     unlocked = false;
uint32_t deniedUntil = 0;                // OLED shows "ACCESS DENIED" until then

uint16_t speedLimit = SPEED_DEFAULT_LIMIT;   // km/h, saved in flash

// ---- Accident state ----
float    curG = 1.0f;                    // latest |a| in g
bool     accidentActive = false;
volatile bool accidentPending = false;   // alert not delivered yet
uint32_t accidentAt = 0, tiltSince = 0, uprightSince = 0, lastTgTry = 0;
uint8_t  tgTries = 0;
const char* accidentWhy = "tilt";
uint8_t* accSnap = NULL;                 // camera frame taken when the accident fired
size_t   accSnapLen = 0;
volatile bool accSnapTried = false, accMsgSent = false, accPhotoSent = false;

// ---- Camera / rider check / ignition ----
enum RcState : uint8_t { RC_IDLE, RC_CAPTURING, RC_ANALYZING, RC_DONE, RC_ERROR };
volatile uint8_t rcState = RC_IDLE;
bool     rcHelmet = false, rcDrowsy = false, rcServerOk = false;
int      rcRiders = 0;
char     rcNote[96] = "", rcError[64] = "";
uint32_t rcAt = 0;
String   camHost = "";                   // blank = CAM_DEFAULT_HOST; saved in flash
IPAddress camIpCache;                    // 0.0.0.0 = not resolved
uint32_t camIpAt = 0;
uint8_t* snapBuf = NULL;                 // last frame sent to the VLM (for the dashboard)
size_t   snapLen = 0;
uint32_t snapId = 0;
SemaphoreHandle_t snapMux = NULL;

// ==========================================================================
//  Small helpers
// ==========================================================================
void buzzer(bool on) { digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? !on : on); }

void beep(uint16_t ms) { buzzer(true); beepUntil = millis() + ms; }

void serviceBeep() {
  if (beepUntil && millis() > beepUntil) { buzzer(false); beepUntil = 0; }
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len, (int)true) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool ldrDark() {
  bool d = digitalRead(LDR_PIN);
  return LDR_DARK_IS_LOW ? !d : d;
}

bool touched() {
  bool t = digitalRead(TOUCH_PIN);
  return TOUCH_ACTIVE_HIGH ? t : !t;
}

// ==========================================================================
//  IMU — works with genuine MPU6050 and with MPU6500/9250 clones
// ==========================================================================
static const float ACC_LSB = 4096.0f;    // +-8 g (wide enough to see crash impacts)
static const float GYR_LSB = 131.0f;     // +-250 dps

Motion readMotionRaw() {
  Motion m; m.valid = false;
  uint8_t b[14];
  if (!i2cRead(MPU_ADDR, 0x3B, b, 14)) return m;

  int16_t rax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ray = (int16_t)((b[2] << 8) | b[3]);
  int16_t raz = (int16_t)((b[4] << 8) | b[5]);
  int16_t rt  = (int16_t)((b[6] << 8) | b[7]);
  if (rax == 0 && ray == 0 && raz == 0) return m;

  m.ax = rax / ACC_LSB * 9.80665f;
  m.ay = ray / ACC_LSB * 9.80665f;
  m.az = raz / ACC_LSB * 9.80665f;
  m.tempC = rt / 340.0f + 36.53f;
  m.gx = (int16_t)((b[8]  << 8) | b[9])  / GYR_LSB * DEG_TO_RAD;
  m.gy = (int16_t)((b[10] << 8) | b[11]) / GYR_LSB * DEG_TO_RAD;
  m.gz = (int16_t)((b[12] << 8) | b[13]) / GYR_LSB * DEG_TO_RAD;
  m.valid = true;
  return m;
}

Motion readMotion() {
  if (mpuMode == MPU_RAW) return readMotionRaw();
  Motion m; m.valid = false;
  if (mpuMode != MPU_ADAFRUIT) return m;
  sensors_event_t a, g, tmp;
  mpu.getEvent(&a, &g, &tmp);
  m.ax = a.acceleration.x; m.ay = a.acceleration.y; m.az = a.acceleration.z;
  m.gx = g.gyro.x;         m.gy = g.gyro.y;         m.gz = g.gyro.z;
  m.tempC = tmp.temperature;
  m.valid = true;
  return m;
}

void initMPU() {
  mpuMode = MPU_NONE;
  if (!i2cRead(MPU_ADDR, 0x75, &mpuWhoAmI, 1)) mpuWhoAmI = 0xFF;

  if (mpu.begin(MPU_ADDR, &Wire)) {
    mpuMode = MPU_ADAFRUIT;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  } else if (i2cPresent(MPU_ADDR)) {
    i2cWrite8(MPU_ADDR, 0x6B, 0x80); delay(100);
    i2cWrite8(MPU_ADDR, 0x6B, 0x00); delay(100);
    i2cWrite8(MPU_ADDR, 0x1A, 0x03);
    i2cWrite8(MPU_ADDR, 0x1B, 0x00);
    i2cWrite8(MPU_ADDR, 0x1C, 0x10);      // +-8 g
    delay(50);
    if (readMotionRaw().valid) mpuMode = MPU_RAW;
  }
}

// Average the gyro at rest. Your board showed ~+0.18/-0.12/+0.01 rad/s of
// bias; without removing it, yaw runs away in seconds.
void calibrateGyro() {
  if (mpuMode == MPU_NONE) return;
  Serial.print(F("Calibrating gyro — hold still"));
  double sx = 0, sy = 0, sz = 0;
  const int N = 300;
  int got = 0;
  for (int i = 0; i < N; i++) {
    Motion m = readMotion();
    if (m.valid) { sx += m.gx; sy += m.gy; sz += m.gz; got++; }
    if (i % 60 == 0) Serial.print('.');
    delay(5);
  }
  if (got) {
    gBias[0] = sx / got; gBias[1] = sy / got; gBias[2] = sz / got;
  }
  Serial.printf(" done  bias[%.4f %.4f %.4f] rad/s\n",
                gBias[0], gBias[1], gBias[2]);
  // Seed the filter from gravity so it starts level instead of sweeping in.
  Motion m = readMotion();
  if (m.valid) {
    roll  = atan2(m.ay, m.az) * RAD_TO_DEG;
    pitch = atan2(-m.ax, sqrt(m.ay * m.ay + m.az * m.az)) * RAD_TO_DEG;
  }
  yaw = 0; yawOffset = 0;
}

// Complementary filter: gyro for short-term response, accelerometer to pin
// roll/pitch to gravity long-term. Yaw gets no correction — nothing to
// correct it against.
void updateAttitude() {
  static uint32_t lastUs = 0;
  Motion m = readMotion();
  if (!m.valid) return;

  uint32_t now = micros();
  float dt = lastUs ? (now - lastUs) / 1000000.0f : 0.01f;
  lastUs = now;
  if (dt <= 0 || dt > 0.5f) return;            // skip absurd gaps

  float gx = (m.gx - gBias[0]) * RAD_TO_DEG;   // deg/s
  float gy = (m.gy - gBias[1]) * RAD_TO_DEG;
  float gz = (m.gz - gBias[2]) * RAD_TO_DEG;

  float rollAcc  = atan2(m.ay, m.az) * RAD_TO_DEG;
  float pitchAcc = atan2(-m.ax, sqrt(m.ay * m.ay + m.az * m.az)) * RAD_TO_DEG;

  roll  = COMP_ALPHA * (roll  + gx * dt) + (1 - COMP_ALPHA) * rollAcc;
  pitch = COMP_ALPHA * (pitch + gy * dt) + (1 - COMP_ALPHA) * pitchAcc;
  yaw  += gz * dt;                             // free-running, drifts

  while (yaw >= 360) yaw -= 360;
  while (yaw <    0) yaw += 360;

  float mag = sqrt(m.ax * m.ax + m.ay * m.ay + m.az * m.az) / 9.80665f;
  curG = mag;
  if (mag > peakG) peakG = mag;
}

float yawDisplay() {
  float y = yaw - yawOffset;
  while (y >= 360) y -= 360;
  while (y <    0) y += 360;
  return y;
}

// ==========================================================================
//  RFID / GPS
// ==========================================================================
void initRFID() {
  rfid.PCD_Init();
  delay(50);
  rfidVersion = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  rfidOK = (rfidVersion != 0x00 && rfidVersion != 0xFF);
}

// ---- settings persistence ----
void loadSettings() {
  prefs.begin("robot", true);
  rfidUid      = prefs.getString("rfid", "");
  gasThreshold = prefs.getUShort("gasThr", GAS_DEFAULT_THRESHOLD);
  speedLimit   = prefs.getUShort("spdLim", SPEED_DEFAULT_LIMIT);
  camHost      = prefs.getString("camHost", "");
  prefs.end();
}

void saveRfid(const String& uid) {
  rfidUid = uid;
  prefs.begin("robot", false);
  if (uid.length()) prefs.putString("rfid", uid); else prefs.remove("rfid");
  prefs.end();
}

void saveGasThreshold(uint16_t v) {
  gasThreshold = v;
  prefs.begin("robot", false);
  prefs.putUShort("gasThr", v);
  prefs.end();
}

void saveCamHost(const String& h) {
  camHost = h;
  camIpCache = IPAddress(0, 0, 0, 0);
  prefs.begin("robot", false);
  if (h.length()) prefs.putString("camHost", h); else prefs.remove("camHost");
  prefs.end();
}

void saveSpeedLimit(uint16_t v) {
  speedLimit = v;
  prefs.begin("robot", false);
  prefs.putUShort("spdLim", v);
  prefs.end();
}

bool overSpeed() {
  return gps.speed.isValid() && gps.speed.age() < 5000 && gps.speed.kmph() > speedLimit;
}

void pollRFID() {
  if (rfidLearn && millis() > rfidLearnUntil) rfidLearn = false;
  if (!rfidOK || millis() - lastRfidPoll < RFID_POLL_MS) return;
  lastRfidPoll = millis();
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  String uid;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += '0';
    uid += String(rfid.uid.uidByte[i], HEX);
    if (i + 1 < rfid.uid.size) uid += ':';
  }
  uid.toUpperCase();
  lastUid = uid;
  cardCount++;
  lastCardMs = millis();
  Serial.printf("CARD %s\n", uid.c_str());

  if (rfidLearn) {
    // Settings page asked us to learn the next card.
    saveRfid(uid);
    rfidLearn = false;
    unlocked = true;                      // they just proved they hold the card
    Serial.printf("RFID unlock card set to %s\n", uid.c_str());
    beep(300);
  } else if (rfidUid.length() == 0) {
    beep(60);                             // nothing registered yet; ignore
  } else if (uid == rfidUid) {
    unlocked = !unlocked;                 // registered card toggles lock
    Serial.println(unlocked ? F("UNLOCKED") : F("LOCKED"));
    beep(unlocked ? 120 : 60);
  } else {
    deniedUntil = millis() + 2000;
    Serial.println(F("ACCESS DENIED — wrong card"));
    beep(500);
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

void feedGPS() { while (Serial2.available()) gps.encode(Serial2.read()); }

// ==========================================================================
//  Camera (ESP32-CAM) + rider check via the VLM agent
// ==========================================================================
bool camAddress(IPAddress& out) {
  if (camIpCache != IPAddress(0, 0, 0, 0) && millis() - camIpAt < 300000UL) {
    out = camIpCache;
    return true;
  }
  String h = camHost.length() ? camHost : String(CAM_DEFAULT_HOST);
  IPAddress r;
  if (!r.fromString(h)) {                          // not a dotted IP -> mDNS name
    if (h.endsWith(".local")) h.remove(h.length() - 6);
    r = MDNS.queryHost(h.c_str(), 2500);
  }
  if (r == IPAddress(0, 0, 0, 0)) return false;
  camIpCache = r;
  camIpAt = millis();
  out = r;
  return true;
}

// Downloads one JPEG from the camera into a fresh malloc'd buffer (caller frees).
bool fetchSnapshot(uint8_t** out, size_t* outLen, bool flash, uint32_t timeoutMs) {
  *out = NULL; *outLen = 0;
  if (WiFi.status() != WL_CONNECTED) return false;
  IPAddress ip;
  if (!camAddress(ip)) { Serial.println(F("Camera: cannot resolve address")); return false; }

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(timeoutMs);
  String url = "http://" + ip.toString() + "/capture" + (flash ? "?flash=1" : "");
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  int size = http.getSize();
  if (code != 200 || size <= 0 || (size_t)size > MAX_SNAP_BYTES) {
    Serial.printf("Camera: HTTP %d, size %d\n", code, size);
    http.end();
    camIpCache = IPAddress(0, 0, 0, 0);            // re-resolve next time
    return false;
  }
  uint8_t* buf = (uint8_t*)malloc(size);
  if (!buf) { http.end(); Serial.println(F("Camera: out of memory")); return false; }

  WiFiClient* st = http.getStreamPtr();
  size_t got = 0;
  uint32_t t0 = millis();
  while (got < (size_t)size && millis() - t0 < timeoutMs + 4000 &&
         (st->connected() || st->available())) {
    got += st->readBytes(buf + got, size - got);
  }
  http.end();
  if (got != (size_t)size || buf[0] != 0xFF || buf[1] != 0xD8) {
    Serial.printf("Camera: bad frame (%u of %d bytes)\n", (unsigned)got, size);
    free(buf);
    return false;
  }
  *out = buf; *outLen = got;
  return true;
}

// ---- tiny flat-JSON readers for the VLM reply: {"ok":true,"riders":1,...} ----
int jsonFind(const String& j, const char* key) {
  String k = String("\"") + key + "\":";
  int i = j.indexOf(k);
  return i < 0 ? -1 : i + (int)k.length();
}
bool jsonBool(const String& j, const char* key, bool def = false) {
  int i = jsonFind(j, key);
  return i < 0 ? def : j.startsWith("true", i);
}
int jsonInt(const String& j, const char* key, int def = 0) {
  int i = jsonFind(j, key);
  return i < 0 ? def : j.substring(i, i + 6).toInt();
}
String jsonStr(const String& j, const char* key) {
  int i = jsonFind(j, key);
  if (i < 0 || j[i] != '"') return "";
  int e = j.indexOf('"', i + 1);
  return e < 0 ? String("") : j.substring(i + 1, e);
}

void rcFail(const char* msg) {
  strlcpy(rcError, msg, sizeof(rcError));
  Serial.printf("Rider check failed: %s\n", msg);
  rcState = RC_ERROR;
}

void riderCheckTask(void*) {
  rcState = RC_CAPTURING;
  uint8_t* buf = NULL;
  size_t len = 0;
  if (!fetchSnapshot(&buf, &len, ldrDark(), 8000)) {
    rcFail("Camera not reachable - is the ESP32-CAM on?");
    vTaskDelete(NULL);
    return;
  }
  // Keep this frame for the dashboard, replacing the previous one.
  xSemaphoreTake(snapMux, portMAX_DELAY);
  free(snapBuf);
  snapBuf = buf; snapLen = len; snapId++;
  xSemaphoreGive(snapMux);

  rcState = RC_ANALYZING;
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(45000);                          // the model call takes a few seconds
  if (!http.begin(client, VLM_URL)) { rcFail("Bad VLM URL"); vTaskDelete(NULL); return; }
  http.addHeader("X-API-Key", VLM_KEY);
  http.addHeader("Content-Type", "image/jpeg");
  int code = http.POST(buf, len);
  String body = code == 200 ? http.getString() : String("");
  http.end();
  Serial.printf("VLM -> HTTP %d  %s\n", code, body.c_str());

  if (code != 200) {
    char m[64];
    snprintf(m, sizeof(m), code > 0 ? "VLM agent error (HTTP %d)" : "VLM agent unreachable (%d)", code);
    rcFail(m);
    vTaskDelete(NULL);
    return;
  }
  rcServerOk = jsonBool(body, "ok");
  rcHelmet   = jsonBool(body, "helmet");
  rcDrowsy   = jsonBool(body, "drowsy");
  rcRiders   = jsonInt(body, "riders");
  strlcpy(rcNote, jsonStr(body, "note").c_str(), sizeof(rcNote));
  rcAt = millis();
  rcState = unlocked ? RC_DONE : RC_IDLE;          // dashboard got locked meanwhile: discard
  vTaskDelete(NULL);
}

bool rcBusy() { return rcState == RC_CAPTURING || rcState == RC_ANALYZING; }

bool startRiderCheck() {
  if (rcBusy() || WiFi.status() != WL_CONNECTED) return false;
  rcState = RC_CAPTURING;
  if (xTaskCreate(riderCheckTask, "riderchk", 10240, NULL, 1, NULL) != pdPASS) {
    rcState = RC_IDLE;
    return false;
  }
  return true;
}

// ---- Ignition: locked until the rider check passes AND there is no alcohol ----
bool rcValid() { return rcState == RC_DONE && millis() - rcAt < CHECK_VALID_MS; }
bool rcPass()  { return rcValid() && rcServerOk && rcHelmet && !rcDrowsy &&
                        rcRiders >= 1 && rcRiders <= MAX_RIDERS; }
bool alcoholNow() { return analogRead(GAS_PIN) > gasThreshold; }

// ==========================================================================
//  Accident detection + Telegram alert
// ==========================================================================
String jsonEsc(const String& in) {
  String o;
  o.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n')        { o += "\\n"; }
    else if ((uint8_t)c >= 0x20) o += c;
  }
  return o;
}

int tgPost(const char* method, const String& body) {
  WiFiClientSecure client;
  client.setInsecure();                   // no CA bundle; fine for a hobby alert
  HTTPClient http;
  String url = String("https://api.telegram.org/bot") + BOT_TOKEN + "/" + method;
  http.setTimeout(8000);
  if (!http.begin(client, url)) return -1;
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  Serial.printf("Telegram %s -> HTTP %d\n", method, code);
  if (code != 200) Serial.println(http.getString());
  http.end();
  return code;
}

// The HTTPS call takes 1-3 s (more on failure), so it runs in its own task and
// never stalls the web server, IMU or OLED.
struct TgJob { String text; bool hasLoc; double lat, lng; bool accident; };
volatile bool tgBusy = false;
volatile int  tgLastCode = 0;             // 0 = nothing sent yet, 200 = OK, else failure

// Telegram sendPhoto: multipart/form-data built in one buffer (JPEG is ~30 KB).
int tgSendPhoto(const uint8_t* jpg, size_t len, const char* caption) {
  const char* B = "----helmetboundary7d3f";
  String head = String("--") + B + "\r\nContent-Disposition: form-data; name=\"chat_id\"\r\n\r\n" + CHAT_ID +
                "\r\n--" + B + "\r\nContent-Disposition: form-data; name=\"caption\"\r\n\r\n" + caption +
                "\r\n--" + B + "\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"accident.jpg\"\r\n"
                "Content-Type: image/jpeg\r\n\r\n";
  String tail = String("\r\n--") + B + "--\r\n";
  size_t total = head.length() + len + tail.length();
  uint8_t* body = (uint8_t*)malloc(total);
  if (!body) { Serial.println(F("Telegram photo: out of memory")); return -2; }
  memcpy(body, head.c_str(), head.length());
  memcpy(body + head.length(), jpg, len);
  memcpy(body + head.length() + len, tail.c_str(), tail.length());

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(15000);
  String url = String("https://api.telegram.org/bot") + BOT_TOKEN + "/sendPhoto";
  int code = -1;
  if (http.begin(client, url)) {
    http.addHeader("Content-Type", String("multipart/form-data; boundary=") + B);
    code = http.POST(body, total);
    Serial.printf("Telegram sendPhoto -> HTTP %d\n", code);
    if (code != 200) Serial.println(http.getString());
    http.end();
  }
  free(body);
  return code;
}

// Accident job: (1) grab a camera frame right away, (2) text + location,
// (3) the photo. Each step remembers if it succeeded so retries don't repeat it.
void telegramTask(void* p) {
  TgJob* j = (TgJob*)p;
  if (j->accident && !accSnapTried) {
    accSnapTried = true;
    free(accSnap);
    accSnap = NULL; accSnapLen = 0;
    fetchSnapshot(&accSnap, &accSnapLen, ldrDark(), 4000);
  }

  int code = 200;
  if (!j->accident || !accMsgSent) {
    String body = String("{\"chat_id\":\"") + CHAT_ID + "\",\"text\":\"" + jsonEsc(j->text) + "\"}";
    code = tgPost("sendMessage", body);
    if (code == 200) {
      if (j->accident) accMsgSent = true;
      if (j->hasLoc) {                    // also drop a map pin in the chat
        char loc[140];
        snprintf(loc, sizeof(loc),
                 "{\"chat_id\":\"%s\",\"latitude\":%.6f,\"longitude\":%.6f}",
                 CHAT_ID, j->lat, j->lng);
        tgPost("sendLocation", String(loc));
      }
    }
  }
  if (j->accident && accMsgSent && !accPhotoSent) {
    if (accSnap) {
      code = tgSendPhoto(accSnap, accSnapLen, "Camera snapshot at the time of the accident");
      if (code == 200) accPhotoSent = true;
    } else {
      accPhotoSent = true;                // no frame (camera offline): nothing to send
    }
  }
  tgLastCode = code;
  if (j->accident && accMsgSent && accPhotoSent) {
    accidentPending = false;
    free(accSnap);
    accSnap = NULL; accSnapLen = 0;
  }
  delete j;
  tgBusy = false;
  vTaskDelete(NULL);
}

bool startTelegram(const String& text, bool hasLoc, double lat, double lng, bool accident) {
  if (tgBusy || WiFi.status() != WL_CONNECTED) return false;
  TgJob* j = new TgJob{text, hasLoc, lat, lng, accident};
  tgBusy = true;
  if (xTaskCreate(telegramTask, "telegram", 16384, j, 1, NULL) != pdPASS) {
    delete j;
    tgBusy = false;
    return false;
  }
  return true;
}

void triggerAccident(const char* why) {
  accidentActive  = true;
  accidentPending = true;
  accidentAt = millis();
  lastTgTry = 0;
  tgTries = 0;
  accidentWhy = why;
  accSnapTried = accMsgSent = accPhotoSent = false;
  Serial.printf("ACCIDENT detected (%s)\n", why);
  beep(800);
}

String accidentText() {
  String t = "Accident occurred!\n";
  t += strcmp(accidentWhy, "impact") == 0 ? "Cause: hard impact detected.\n"
                                          : "Cause: helmet has fallen over.\n";
  if (gps.location.isValid()) {
    char b[100];
    snprintf(b, sizeof(b), "Location: https://maps.google.com/?q=%.6f,%.6f",
             gps.location.lat(), gps.location.lng());
    t += b;
  } else {
    t += "GPS location not available (no fix yet).";
  }
  return t;
}

// Retries every 10 s until Telegram accepts the message (gives up after ~2 min).
void serviceTelegram() {
  if (!accidentPending || tgBusy) return;
  if (tgTries >= 12) {
    accidentPending = false;              // give up
    free(accSnap);
    accSnap = NULL; accSnapLen = 0;
    return;
  }
  if (lastTgTry && millis() - lastTgTry < 10000) return;
  lastTgTry = millis();
  bool fix = gps.location.isValid();
  if (startTelegram(accidentText(), fix, fix ? gps.location.lat() : 0.0,
                    fix ? gps.location.lng() : 0.0, true)) tgTries++;
}

// Called at IMU rate. Accident = helmet lying on its side/upside down for
// ACCIDENT_TILT_MS, or a single impact above ACCIDENT_IMPACT_G.
void checkAccident() {
  uint32_t now = millis();
  if (now < 5000) return;                 // ignore start-up / calibration

  bool tilted = fabsf(roll) > ACCIDENT_TILT_DEG || fabsf(pitch) > ACCIDENT_TILT_DEG;
  if (tilted) { if (!tiltSince) tiltSince = now; } else tiltSince = 0;

  if (!accidentActive) {
    bool fell = tiltSince && now - tiltSince >= ACCIDENT_TILT_MS;
    bool hit  = curG >= ACCIDENT_IMPACT_G;
    if (fell || hit) triggerAccident(hit ? "impact" : "tilt");
  } else {
    // Clears itself once the helmet has been upright for 5 s (and alert is out).
    if (!tilted) { if (!uprightSince) uprightSince = now; } else uprightSince = 0;
    if (!accidentPending && now - accidentAt > ACCIDENT_REARM_MS &&
        uprightSince && now - uprightSince > 5000) accidentActive = false;
  }
}

// ==========================================================================
//  Dashboard page
// ==========================================================================
const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Project Helmet</title>
<style>
:root{--bg:#0e1116;--card:#171c24;--line:#242c38;--tx:#e6edf3;--dim:#8b96a5;
      --ok:#3fb950;--warn:#d29922;--bad:#f85149;--acc:#58a6ff}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--tx);
     font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}
h1{font-size:18px;margin:0 0 2px}
header{display:flex;justify-content:space-between;align-items:center;
       flex-wrap:wrap;gap:8px;margin-bottom:14px}
.sub{color:var(--dim);font-size:12px}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;
     background:var(--bad);margin-right:6px}
.dot.on{background:var(--ok)}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(250px,1fr))}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px}
.card h2{font-size:11px;letter-spacing:.09em;text-transform:uppercase;
         color:var(--dim);margin:0 0 10px;font-weight:600}
.row{display:flex;justify-content:space-between;padding:3px 0}
.row span:first-child{color:var(--dim)}
.mono{font-variant-numeric:tabular-nums;font-family:ui-monospace,Consolas,monospace}
.big{font-size:26px;font-weight:600;line-height:1.1}
.trip{display:flex;gap:10px;text-align:center;margin-bottom:10px}
.trip>div{flex:1;background:#0e1319;border:1px solid var(--line);
          border-radius:8px;padding:8px 4px}
.trip small{display:block;color:var(--dim);font-size:10px;
            letter-spacing:.08em;text-transform:uppercase}
.viz{display:flex;gap:12px;align-items:center;justify-content:center;margin:6px 0 12px}
.horizon{position:relative;width:132px;height:132px;border-radius:50%;
         overflow:hidden;border:2px solid var(--line);flex:none;background:#000}
.sky{position:absolute;left:-60%;top:-60%;width:220%;height:220%;
     background:linear-gradient(#2d7dd2 0 50%,#7a5c3e 50% 100%);
     transform-origin:50% 50%}
.cross{position:absolute;inset:0;pointer-events:none}
.cross:before,.cross:after{content:"";position:absolute;background:#ffcc33}
.cross:before{left:26%;right:26%;top:50%;height:2px;margin-top:-1px}
.cross:after{left:50%;top:44%;height:12px;width:2px;margin-left:-1px}
.compass{position:relative;width:132px;height:132px;border-radius:50%;
         border:2px solid var(--line);flex:none;background:#0e1319}
.needle{position:absolute;left:50%;top:50%;width:3px;height:52px;
        background:linear-gradient(var(--bad) 0 50%,#5b6672 50% 100%);
        transform-origin:50% 100%;margin:-52px 0 0 -1.5px;border-radius:2px}
.compass b{position:absolute;left:50%;transform:translateX(-50%);top:5px;
           font-size:10px;color:var(--dim)}
.pill{font-size:11px;padding:2px 8px;border-radius:99px;border:1px solid var(--line)}
.pill.ok{color:var(--ok);border-color:#1d4429}.pill.bad{color:var(--bad);border-color:#5c2023}
.pill.warn{color:var(--warn);border-color:#5a4415}
.note{color:var(--warn);font-size:11px;margin-top:8px;line-height:1.4}
button{background:#21262d;color:var(--tx);border:1px solid var(--line);
       border-radius:7px;padding:7px 12px;font-size:12px;cursor:pointer}
button:hover{border-color:var(--acc);color:var(--acc)}
.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
a{color:var(--acc)}
.alert{border-radius:8px;padding:10px 14px;margin-bottom:12px;font-weight:600;
       align-items:center;justify-content:space-between;gap:10px}
.alert.bad{background:#3a1416;border:1px solid #5c2023;color:var(--bad)}
.lampwrap{display:flex;align-items:center;gap:12px;margin-bottom:8px}
.lamp{width:76px;height:76px;border-radius:50%;background:#fff;flex:none;opacity:.12;
      transition:opacity .6s,box-shadow .6s}
.lamp.on{opacity:1;box-shadow:0 0 44px 16px rgba(255,255,255,.8)}
.lampwrap{gap:16px;margin:6px 0 14px}
.ignpill{font-size:14px;padding:5px 14px;font-weight:700}
.ignhead{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:8px}
/* --- live map --- */
#map{height:380px;border-radius:8px;border:1px solid var(--line);background:#0e1319}
#mapmsg{height:380px;display:flex;align-items:center;justify-content:center;
        text-align:center;padding:20px;color:var(--dim);font-size:13px;
        border:1px dashed var(--line);border-radius:8px}
.leaflet-container{background:#0e1319}
/* Dark-ish tiles without a second tile provider. */
.leaflet-tile{filter:brightness(.72) contrast(1.05) saturate(.85)}
.leaflet-control-attribution{background:rgba(14,17,22,.8)!important;color:#8b96a5!important}
.leaflet-control-attribution a{color:#58a6ff!important}
.hmark{width:0;height:0}
.hmark i{position:absolute;left:-9px;top:-9px;width:18px;height:18px;
         border-radius:50%;background:var(--acc);border:2px solid #fff;
         box-shadow:0 0 0 4px rgba(88,166,255,.25)}
.hmark b{position:absolute;left:-7px;top:-24px;width:14px;height:14px;
         border-left:7px solid transparent;border-right:7px solid transparent;
         border-bottom:14px solid var(--acc);display:none}
.hmark.moving b{display:block}
.mapbar{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:10px}
.mapbar label{font-size:12px;color:var(--dim);display:flex;align-items:center;gap:5px}
</style></head><body>

<header>
  <div><h1>Project Helmet</h1>
  <div class="sub" id="sub">connecting…</div></div>
  <div><a href="/settings"><button>&#9881; Settings</button></a>
  <button onclick="lockNow()">&#128274; Lock</button>
  &nbsp;<span class="dot" id="dot"></span><span class="sub" id="live">offline</span></div>
</header>

<div id="accbar" class="alert bad" style="display:none">
  <span id="acctxt">ACCIDENT DETECTED</span>
  <button onclick="api('accident-clear')">Clear alert</button></div>
<div id="spdbar" class="alert bad" style="display:none"></div>

<div class="grid">

  <div class="card" style="grid-column:1/-1">
    <h2>Ignition</h2>
    <div class="ignhead"><span id="ignpill" class="pill bad ignpill">LOCKED</span>
      <span class="sub" id="ignwhy">Run the rider check to unlock ignition.</span></div>
    <div class="row"><span>Helmet</span><span id="c_helmet" class="pill">--</span></div>
    <div class="row"><span>Riders</span><span id="c_riders" class="pill">--</span></div>
    <div class="row"><span>Drowsiness</span><span id="c_drowsy" class="pill">--</span></div>
    <div class="row"><span>Alcohol</span><span id="c_alc" class="pill">--</span></div>
    <div class="sub" id="ignnote" style="margin-top:6px"></div>
    <div class="btns">
      <button id="rcbtn" onclick="riderCheck()">&#128247; Check rider (camera)</button>
      <button id="vbtn" onclick="toggleVoice()">&#128266; Voice: ON</button>
      <button onclick="repeatMsg()">&#128483; Repeat message</button>
    </div>
    <img id="snap" alt="analysed frame" style="display:none;max-width:100%;margin-top:10px;
         border-radius:8px;border:1px solid var(--line)">
  </div>

  <div class="card" style="grid-column:1/-1">
    <h2>Attitude</h2>
    <div class="trip">
      <div><small>Roll</small><div class="big mono" id="roll">--</div></div>
      <div><small>Pitch</small><div class="big mono" id="pitch">--</div></div>
      <div><small>Yaw</small><div class="big mono" id="yaw">--</div></div>
    </div>
    <div class="viz">
      <div class="horizon"><div class="sky" id="sky"></div><div class="cross"></div></div>
      <div class="compass"><b>N</b><div class="needle" id="needle"></div></div>
    </div>
    <div class="row"><span>GPS course (true heading)</span><span class="mono" id="course">--</span></div>
    <div class="row"><span>Peak impact</span><span class="mono" id="peak">--</span></div>
    <div class="note">Roll and pitch are absolute (referenced to gravity).
      Yaw is integrated gyro only — no magnetometer, so it drifts a few
      degrees per minute. Zero it when you need a fresh reference; GPS
      course is the only true heading, and only while moving.</div>
    <div class="btns">
      <button onclick="api('zero-yaw')">Zero yaw</button>
      <button onclick="api('calibrate')">Calibrate gyro (hold still)</button>
      <button onclick="api('reset-peak')">Reset peak</button>
      <button onclick="api('beep')">Beep</button>
    </div>
  </div>

  <div class="card"><h2>Motion raw</h2>
    <div class="row"><span>accel X</span><span class="mono" id="ax">--</span></div>
    <div class="row"><span>accel Y</span><span class="mono" id="ay">--</span></div>
    <div class="row"><span>accel Z</span><span class="mono" id="az">--</span></div>
    <div class="row"><span>|a|</span><span class="mono" id="mag">--</span></div>
    <div class="row"><span>gyro X/Y/Z</span><span class="mono" id="gyr">--</span></div>
    <div class="row"><span>die temp</span><span class="mono" id="temp">--</span></div>
  </div>

  <div class="card"><h2>Environment</h2>
    <div class="lampwrap"><div class="lamp" id="lamp"></div>
      <div class="sub" id="lamptxt">--</div></div>
    <div class="row"><span>Gas</span><span class="mono" id="gas">--</span></div>
    <div class="row"><span>Alcohol threshold</span><span class="mono" id="gasthr">--</span></div>
    <div class="row"><span>Gas status</span><span id="gasp" class="pill">--</span></div>
    <div class="row"><span>Light</span><span id="ldr" class="pill">--</span></div>
    <div class="row"><span>Touch</span><span id="touch" class="pill">--</span></div>
  </div>

  <div class="card" style="grid-column:1/-1">
    <h2>Live map</h2>
    <div class="mapbar">
      <button onclick="recenter()">Recenter</button>
      <button onclick="clearTrail()">Clear trail</button>
      <label><input type="checkbox" id="follow" checked> Follow helmet</label>
      <span class="sub" id="mapinfo" style="margin-left:auto">waiting for fix…</span>
    </div>
    <div id="map"></div>
    <div id="mapmsg" style="display:none"></div>
  </div>

  <div class="card"><h2>Position</h2>
    <div class="row"><span>Fix</span><span id="fix" class="pill">--</span></div>
    <div class="row"><span>Latitude</span><span class="mono" id="lat">--</span></div>
    <div class="row"><span>Longitude</span><span class="mono" id="lng">--</span></div>
    <div class="row"><span>Satellites</span><span class="mono" id="sats">--</span></div>
    <div class="row"><span>Speed</span><span class="mono" id="kph">--</span></div>
    <div class="row"><span>Speed limit</span><span class="mono" id="spdlim">--</span></div>
    <div class="row"><span>Altitude</span><span class="mono" id="alt">--</span></div>
    <div class="row"><span>UTC</span><span class="mono" id="utc">--</span></div>
    <div class="row"><span>Map</span><span id="maplink">--</span></div>
  </div>

  <div class="card"><h2>Rider tag</h2>
    <div class="row"><span>Unlock card</span><span id="rset" class="pill">--</span></div>
    <div class="row"><span>Lock</span><span id="lock" class="pill">--</span></div>
    <div class="row"><span>Last UID</span><span class="mono" id="uid">--</span></div>
    <div class="row"><span>Scans</span><span class="mono" id="cards">--</span></div>
    <div class="row"><span>Last seen</span><span class="mono" id="ago">--</span></div>
  </div>

  <div class="card"><h2>System</h2>
    <div class="row"><span>IP</span><span class="mono" id="ip">--</span></div>
    <div class="row"><span>Wi-Fi RSSI</span><span class="mono" id="rssi">--</span></div>
    <div class="row"><span>Uptime</span><span class="mono" id="up">--</span></div>
    <div class="row"><span>Free heap</span><span class="mono" id="heap">--</span></div>
    <div class="row"><span>IMU mode</span><span class="mono" id="imumode">--</span></div>
  </div>

</div>

<script>
const $=id=>document.getElementById(id);
let fails=0;

/* ---------------- Live map ----------------
   Leaflet and the OpenStreetMap tiles are fetched by YOUR BROWSER, not by the
   ESP32. The ESP32 only serves this page and the JSON. So the phone or laptop
   viewing this needs internet access; the ESP32 itself does not.            */
let map=null, marker=null, trail=null, accCircle=null;
// Leaflet is loaded in the background so a missing internet connection can
// never block the page (the map tiles come from the internet, not the ESP32).
let leaflet='loading';
(function(){
  const c=document.createElement('link');
  c.rel='stylesheet'; c.href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css';
  document.head.appendChild(c);
  const j=document.createElement('script');
  j.src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js';
  j.onload=()=>{leaflet='ok';}; j.onerror=()=>{leaflet='failed';};
  document.head.appendChild(j);
})();
let trailPts=[], lastPt=null, lastRender=null, totalM=0;
const MAX_TRAIL=600;                   // ~600 points is plenty, keeps it snappy

function mapUnavailable(msg){
  $('map').style.display='none';
  const m=$('mapmsg');
  m.style.display='flex';
  m.innerHTML=msg;
}

function initMap(lat,lng){
  if(leaflet==='loading') return false;          // try again on the next tick
  if(leaflet==='failed' || typeof L==='undefined'){
    mapUnavailable('Map library could not load.<br>'+
      'Leaflet and the tiles come from the internet via your browser — '+
      'this device needs a working internet connection. '+
      'Coordinates below still work offline.');
    return false;
  }
  map=L.map('map',{zoomControl:true,attributionControl:true}).setView([lat,lng],17);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{
    maxZoom:19, attribution:'&copy; OpenStreetMap contributors'
  }).addTo(map);
  trail=L.polyline([],{color:'#58a6ff',weight:3,opacity:.75}).addTo(map);
  accCircle=L.circle([lat,lng],{radius:10,color:'#58a6ff',weight:1,
                                fillOpacity:.08}).addTo(map);
  marker=L.marker([lat,lng],{icon:L.divIcon({className:'hmark',
          html:'<b></b><i></i>',iconSize:[0,0]})}).addTo(map);
  // Dragging the map turns off follow so it doesn't fight you.
  map.on('dragstart',()=>{ $('follow').checked=false; });
  return true;
}

function metres(a,b){                   // haversine
  const R=6371000, t=Math.PI/180;
  const dLa=(b[0]-a[0])*t, dLo=(b[1]-a[1])*t;
  const h=Math.sin(dLa/2)**2 +
          Math.cos(a[0]*t)*Math.cos(b[0]*t)*Math.sin(dLo/2)**2;
  return 2*R*Math.asin(Math.sqrt(h));
}

function updateMap(g){
  if(!g.fix) return;
  const pt=[g.lat,g.lng];
  if(!map && !initMap(g.lat,g.lng)) return;

  // The page polls at 4 Hz but the GPS only reports at 1 Hz. Skip ticks where
  // the fix hasn't moved, so we aren't re-panning the map three times for
  // nothing.
  if(lastRender && lastRender[0]===pt[0] && lastRender[1]===pt[1]) return;
  lastRender=pt;

  marker.setLatLng(pt);
  // HDOP is a rough dilution figure, not metres. ~5 m per unit is a usable
  // visual approximation of uncertainty, nothing more.
  accCircle.setLatLng(pt).setRadius(Math.max(3,(g.hdop||1)*5));

  const el=marker.getElement();
  if(el){
    const moving=g.kph>2;
    el.classList.toggle('moving',moving);
    if(moving) el.style.transform+=' rotate('+g.course+'deg)';
  }

  // Only extend the trail on real movement, so GPS jitter at rest doesn't
  // scribble a ball of noise.
  if(!lastPt || metres(lastPt,pt)>2.5){
    if(lastPt) totalM+=metres(lastPt,pt);
    trailPts.push(pt);
    if(trailPts.length>MAX_TRAIL) trailPts.shift();
    trail.setLatLngs(trailPts);
    lastPt=pt;
  }
  if($('follow').checked) map.panTo(pt,{animate:true,duration:.4});

  $('mapinfo').textContent=g.sats+' sats · HDOP '+
    (g.hdop||0).toFixed(1)+' · trail '+
    (totalM<1000?Math.round(totalM)+' m':(totalM/1000).toFixed(2)+' km');
}

function recenter(){
  if(map && lastPt){ $('follow').checked=true; map.setView(lastPt,17); }
}
function clearTrail(){
  trailPts=[]; totalM=0; lastPt=null; lastRender=null;
  if(trail) trail.setLatLngs([]);
}

function pill(el,text,cls){el.textContent=text;el.className='pill '+(cls||'');}

/* ---------------- Voice alerts ----------------
   Uses the browser's built-in speech synthesis, so it speaks out of whichever
   device is showing the page (laptop or phone). Browsers only allow sound
   after the page has been tapped/clicked once.                                */
let voiceOn=true, voiceReady=false, lastSpoken='', prevState=null, prevReady=null;
try{ if(localStorage.getItem('voice')==='0') voiceOn=false; }catch(e){}
const canSpeak='speechSynthesis' in window;

function speak(t){
  lastSpoken=t;
  if(!voiceOn||!canSpeak) return;
  try{
    speechSynthesis.cancel();
    const u=new SpeechSynthesisUtterance(t);
    u.lang='en-US'; u.rate=.95;
    speechSynthesis.speak(u);
  }catch(e){}
}
function drawVoice(){
  $('vbtn').innerHTML=canSpeak?('&#128266; Voice: '+(voiceOn?'ON':'OFF')):'Voice not supported';
}
function toggleVoice(){
  voiceOn=!voiceOn;
  try{ localStorage.setItem('voice',voiceOn?'1':'0'); }catch(e){}
  drawVoice();
  if(voiceOn) speak('Voice alerts on.'); else if(canSpeak) speechSynthesis.cancel();
}
function repeatMsg(){
  if(!voiceOn){ voiceOn=true; drawVoice(); }
  speak(lastSpoken||'No message yet. Run the rider check.');
}
document.addEventListener('click',()=>{            // "unlock" audio on the first tap
  if(voiceReady||!canSpeak) return;
  voiceReady=true;
  try{ const u=new SpeechSynthesisUtterance(' '); u.volume=0; speechSynthesis.speak(u); }catch(e){}
});

function reasons(g,w){
  const r=[];
  if(!g.helmet) r.push('no helmet detected');
  if(g.riders<1) r.push('no rider detected');
  else if(g.riders>g.maxRiders) r.push('too many riders, '+g.riders+' detected, the maximum is '+g.maxRiders);
  if(g.drowsy) r.push('the rider looks drowsy');
  if(g.alcohol) r.push('alcohol detected');
  if(w.accident) r.push('an accident alert is active');
  if(g.state===3 && !g.valid) r.push('the rider check has expired, please run it again');
  return r;
}
function sentence(rs){ return rs.map(x=>x[0].toUpperCase()+x.slice(1)).join('. ')+'.'; }
function verdictText(g,rs){
  if(g.ready)
    return 'Safe to ignite. Helmet detected, '+(g.riders===1?'one rider':g.riders+' riders')+
           ', the rider is alert, and no alcohol detected.';
  return 'Not safe to ignite. '+(rs.length?sentence(rs):'The photo was unclear, please try again.');
}
function announce(g,w){
  const rs=reasons(g,w);
  if(prevState===null){ prevState=g.state; prevReady=g.ready; return; }   // first load: stay quiet
  const busy=(g.state===1||g.state===2);
  if(g.state!==prevState){
    if(g.state===3 && (prevState===1||prevState===2)){ speak(verdictText(g,rs)); prevReady=g.ready; }
    else if(g.state===4 && prevState!==4){ speak('Rider check failed. '+g.err); prevReady=g.ready; }
    prevState=g.state;
  }else if(!busy && g.ready!==prevReady){               // e.g. alcohol appears after a pass
    speak(g.ready?'Safe to ignite.':'Ignition locked. '+(rs.length?sentence(rs):''));
    prevReady=g.ready;
  }
}

let lastSnap=0;
function riderCheck(){
  $('rcbtn').disabled=true;
  fetch('/api/rider-check').then(r=>r.text()).then(t=>{ $('ignwhy').textContent=t; });
}

function lockNow(){ fetch('/api/lock').then(()=>location.reload()); }

function api(p){
  fetch('/api/'+p).then(r=>r.text()).then(t=>{ $('sub').textContent=t; });
}

function fmt(v,d){return (v===null||v===undefined)?'--':Number(v).toFixed(d);}

async function tick(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    const d=await r.json();
    if(d.locked){ location.reload(); return; }   // card was scanned again -> lock page
    fails=0; $('dot').classList.add('on'); $('live').textContent='live';

    $('roll').textContent  = fmt(d.att.roll,1)+'°';
    $('pitch').textContent = fmt(d.att.pitch,1)+'°';
    $('yaw').textContent   = fmt(d.att.yaw,1)+'°';
    $('sky').style.transform =
      'rotate('+(-d.att.roll)+'deg) translateY('+(d.att.pitch*1.6)+'px)';
    $('needle').style.transform = 'rotate('+d.att.yaw+'deg)';
    $('peak').textContent = fmt(d.imu.peak,2)+' g';

    $('ax').textContent=fmt(d.imu.ax,2)+' m/s²';
    $('ay').textContent=fmt(d.imu.ay,2)+' m/s²';
    $('az').textContent=fmt(d.imu.az,2)+' m/s²';
    $('mag').textContent=fmt(d.imu.mag,2)+' g';
    $('gyr').textContent=fmt(d.imu.gx,1)+' / '+fmt(d.imu.gy,1)+' / '+fmt(d.imu.gz,1)+' °/s';
    $('temp').textContent=fmt(d.imu.temp,1)+' °C';
    $('imumode').textContent=d.imu.mode;

    $('gas').textContent=d.env.gasRaw+' raw / '+d.env.gasMv+' mV';
    $('gasthr').textContent=d.env.gasThr+' raw';
    pill($('gasp'),d.env.gasAlarm?'ALARM':'normal',d.env.gasAlarm?'bad':'ok');
    pill($('ldr'),d.env.dark?'dark':'light',d.env.dark?'warn':'ok');
    $('lamp').classList.toggle('on',d.env.dark);
    $('lamptxt').textContent=d.env.dark?'Low light - lamp ON':'Bright - lamp dimmed';
    pill($('touch'),d.env.touch?'touched':'idle',d.env.touch?'ok':'');

    pill($('fix'),d.gps.fix?'locked':'searching',d.gps.fix?'ok':'warn');
    $('lat').textContent=d.gps.fix?fmt(d.gps.lat,6):'--';
    $('lng').textContent=d.gps.fix?fmt(d.gps.lng,6):'--';
    $('sats').textContent=d.gps.sats;
    $('kph').textContent=fmt(d.gps.kph,1)+' km/h';
    $('alt').textContent=fmt(d.gps.altM,1)+' m';
    $('utc').textContent=d.gps.utc;
    $('course').textContent=(d.gps.fix&&d.gps.kph>2)
        ? fmt(d.gps.course,1)+'°' : 'needs motion';
    const g=d.ign, done=g.state===3;
    $('ignpill').textContent=g.ready?'UNLOCKED - OK to ignite':'LOCKED';
    $('ignpill').className='pill ignpill '+(g.ready?'ok':'bad');
    pill($('c_helmet'),done?(g.helmet?'\u2713 worn':'\u2717 not worn'):'--',done?(g.helmet?'ok':'bad'):'');
    const rOk=g.riders>=1&&g.riders<=g.maxRiders;
    pill($('c_riders'),done?(g.riders+' (max '+g.maxRiders+')'):'--',done?(rOk?'ok':'bad'):'');
    pill($('c_drowsy'),done?(g.drowsy?'\u2717 drowsy':'\u2713 alert'):'--',done?(g.drowsy?'bad':'ok'):'');
    pill($('c_alc'),g.alcohol?'\u2717 alcohol detected':'\u2713 clear',g.alcohol?'bad':'ok');
    let why;
    if(g.state===1) why='Taking photo...';
    else if(g.state===2) why='Analyzing with the VLM agent...';
    else if(g.state===4) why=g.err;
    else if(g.ready) why='All checks passed.';
    else if(done && !g.valid) why='Check expired - run it again.';
    else if(done){
      const r=[]; if(!g.helmet) r.push('no helmet'); if(!rOk) r.push('rider count'); if(g.drowsy) r.push('drowsy');
      if(g.alcohol) r.push('alcohol'); if(d.warn.accident) r.push('accident alert');
      why='Locked: '+(r.join(', ')||'photo unclear - try again');
    } else why='Run the rider check to unlock ignition.';
    $('ignwhy').textContent=why;
    announce(g,d.warn);
    $('ignnote').textContent=(done&&g.note)?('VLM: '+g.note):'';
    $('rcbtn').disabled=(g.state===1||g.state===2);
    if(g.snapId && g.snapId!==lastSnap){
      lastSnap=g.snapId; $('snap').src='/api/snapshot.jpg?id='+g.snapId; $('snap').style.display='block';
    }
    const w=d.warn;
    $('spdlim').textContent=w.speedLimit+' km/h';
    $('spdbar').style.display=w.overSpeed?'block':'none';
    if(w.overSpeed) $('spdbar').textContent='SPEED WARNING: '+fmt(d.gps.kph,0)+
      ' km/h (limit '+w.speedLimit+' km/h)';
    $('accbar').style.display=w.accident?'flex':'none';
    if(w.accident) $('acctxt').textContent='ACCIDENT DETECTED - Telegram alert '+
      (w.tg===200?'sent':(w.tgBusy?'sending...':'not delivered yet ('+w.tg+')'));
    updateMap(d.gps);
    $('maplink').innerHTML = d.gps.fix
      ? '<a target="_blank" rel="noopener" href="https://www.openstreetmap.org/?mlat='
        +d.gps.lat+'&mlon='+d.gps.lng+'#map=18/'+d.gps.lat+'/'+d.gps.lng+'">open</a>'
      : '--';

    pill($('rset'),d.rfid.set?'set':'NOT SET',d.rfid.set?'ok':'bad');
    pill($('lock'),d.rfid.unlocked?'unlocked':'locked',d.rfid.unlocked?'ok':'warn');
    $('uid').textContent=d.rfid.uid;
    $('cards').textContent=d.rfid.count;
    $('ago').textContent=d.rfid.count?Math.round(d.rfid.agoMs/1000)+' s ago':'never';

    $('ip').textContent=d.sys.ip;
    $('rssi').textContent=d.sys.rssi+' dBm';
    $('heap').textContent=Math.round(d.sys.heap/1024)+' KB';
    const s=Math.floor(d.sys.upMs/1000);
    $('up').textContent=Math.floor(s/3600)+'h '+Math.floor(s%3600/60)+'m '+(s%60)+'s';
    $('sub').textContent='updated '+new Date().toLocaleTimeString();
  }catch(e){
    if(++fails>2){$('dot').classList.remove('on');$('live').textContent='offline';}
  }
}
drawVoice();
setInterval(tick,250); tick();
</script></body></html>)HTML";



// ==========================================================================
//  Lock page (shown instead of the dashboard until the RFID card is scanned)
// ==========================================================================
const char LOCK_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Helmet Locked</title>
<style>
:root{--bg:#0e1116;--card:#171c24;--line:#242c38;--tx:#e6edf3;--dim:#8b96a5;
      --bad:#f85149;--warn:#d29922;--acc:#58a6ff}
*{box-sizing:border-box}
body{margin:0;min-height:100vh;display:flex;align-items:center;justify-content:center;
     padding:16px;background:var(--bg);color:var(--tx);
     font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}
.box{background:var(--card);border:1px solid var(--line);border-radius:12px;
     padding:28px 24px;max-width:380px;text-align:center}
.ico{font-size:44px;line-height:1}
h1{font-size:20px;margin:10px 0 6px}
p{color:var(--dim);margin:6px 0}
.bad{color:var(--bad);font-weight:600;min-height:20px}
button{background:#1f6feb;color:#fff;border:0;border-radius:7px;padding:9px 16px;
       font-size:14px;cursor:pointer;margin-top:12px}
</style></head><body>
<div class="box">
  <div class="ico">&#128274;</div>
  <h1 id="h">Locked</h1>
  <p id="p">Scan your registered RFID card on the helmet to unlock.</p>
  <div class="bad" id="d"></div>
  <div id="a"></div>
</div>
<script>
const $=id=>document.getElementById(id);
async function poll(){
  try{
    const d=await (await fetch('/api/lock-status',{cache:'no-store'})).json();
    if(d.rfidSet && d.unlocked){ location.reload(); return; }
    if(!d.rfidSet){
      $('h').textContent='RFID not set';
      $('p').textContent='No RFID card is registered yet. Open the settings page and set your RFID card.';
      $('a').innerHTML='<a href="/settings"><button>Open settings</button></a>';
      $('d').textContent='';
    }else{
      $('h').textContent='Locked';
      $('p').textContent='Scan your registered RFID card on the helmet to unlock.';
      $('a').innerHTML='';
      $('d').textContent=d.denied?'Wrong card - access denied':'';
    }
  }catch(e){}
}
setInterval(poll,700); poll();
</script></body></html>)HTML";

// ==========================================================================
//  Settings page
// ==========================================================================
const char SETTINGS_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Helmet Settings</title>
<style>
:root{--bg:#0e1116;--card:#171c24;--line:#242c38;--tx:#e6edf3;--dim:#8b96a5;
      --ok:#3fb950;--warn:#d29922;--bad:#f85149;--acc:#58a6ff}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--tx);max-width:560px;
     margin-inline:auto;font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}
h1{font-size:18px;margin:0 0 2px}
.sub{color:var(--dim);font-size:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;
      padding:14px;margin-top:12px}
.card h2{font-size:11px;letter-spacing:.09em;text-transform:uppercase;
         color:var(--dim);margin:0 0 10px;font-weight:600}
.row{display:flex;justify-content:space-between;align-items:center;padding:3px 0}
.row span:first-child{color:var(--dim)}
.mono{font-variant-numeric:tabular-nums;font-family:ui-monospace,Consolas,monospace}
.pill{font-size:11px;padding:2px 8px;border-radius:99px;border:1px solid var(--line)}
.pill.ok{color:var(--ok);border-color:#1d4429}.pill.bad{color:var(--bad);border-color:#5c2023}
.pill.warn{color:var(--warn);border-color:#5a4415}
input{background:#0e1319;color:var(--tx);border:1px solid var(--line);
      border-radius:7px;padding:8px 10px;font-size:14px;width:100%}
button{background:#21262d;color:var(--tx);border:1px solid var(--line);
       border-radius:7px;padding:8px 12px;font-size:13px;cursor:pointer}
button:hover{border-color:var(--acc);color:var(--acc)}
button.pri{background:#1f6feb;border-color:#1f6feb;color:#fff}
.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
.inl{display:flex;gap:8px;margin-top:10px}.inl button{flex:none}
#msg{margin-top:12px;min-height:20px;color:var(--acc);font-size:13px}
a{color:var(--acc)}
.warnbox{background:#2a1d0a;border:1px solid #5a4415;color:var(--warn);
         border-radius:8px;padding:10px;margin-top:12px;font-size:13px}
</style></head><body>
<h1>Helmet Settings</h1>
<div class="sub"><a href="/">&larr; Dashboard</a></div>

<div id="warn" class="warnbox" style="display:none">
  RFID is not set. Scan a card below (or type its UID) to enable unlocking.</div>

<div class="card">
  <h2>RFID unlock card</h2>
  <div class="row"><span>Status</span><span id="rstat" class="pill">--</span></div>
  <div class="row"><span>Card UID</span><span class="mono" id="ruid">--</span></div>
  <div class="btns">
    <button class="pri" id="learnbtn" onclick="learn()">Scan card to set</button>
    <button onclick="clearRfid()">Clear card</button>
  </div>
  <div class="sub" id="learnmsg" style="margin-top:8px"></div>
  <div class="inl">
    <input id="uidin" placeholder="or type UID, e.g. 04:A3:1B:2C" autocapitalize="characters">
    <button onclick="setUid()">Save</button>
  </div>
</div>

<div class="card">
  <h2>Alcohol sensor threshold</h2>
  <div class="row"><span>Live reading</span><span class="mono" id="graw">--</span></div>
  <div class="row"><span>Current threshold</span><span class="mono" id="gthr">--</span></div>
  <div class="row"><span>State</span><span id="gstat" class="pill">--</span></div>
  <div class="inl">
    <input id="gasin" type="number" min="0" max="4095" step="1" placeholder="raw value 0 - 4095">
    <button class="pri" onclick="setGas()">Save</button>
  </div>
  <div class="btns"><button onclick="useLive()">Use live reading</button></div>
  <div class="sub" style="margin-top:8px">Alarm triggers when the live reading goes
    above the threshold. Note the value in clean air, then blow near the
    sensor and pick a number in between.</div>
</div>

<div class="card">
  <h2>Speed limit</h2>
  <div class="row"><span>Current limit</span><span class="mono" id="slim">--</span></div>
  <div class="inl">
    <input id="spdin" type="number" min="1" max="300" step="1" placeholder="km/h">
    <button class="pri" onclick="setSpd()">Save</button>
  </div>
  <div class="sub" style="margin-top:8px">A warning shows on the dashboard and the
    OLED when the GPS speed goes above this limit.</div>
</div>

<div class="card">
  <h2>Accident alert (Telegram)</h2>
  <div class="row"><span>Last message</span><span id="tgst" class="pill">--</span></div>
  <div class="btns"><button onclick="call('/api/telegram-test')">Send test message</button></div>
  <div class="sub" style="margin-top:8px">If the helmet falls over or takes a hard
    hit, "Accident occurred" plus the GPS location is sent to the Telegram bot.
    The ESP32's Wi-Fi network must have internet access.</div>
</div>

<div class="card">
  <h2>Camera (ESP32-CAM)</h2>
  <div class="row"><span>Camera address</span><span class="mono" id="camh">--</span></div>
  <div class="inl">
    <input id="camin" placeholder="blank = espcam.local, or type an IP">
    <button class="pri" onclick="setCam()">Save</button>
  </div>
  <div class="sub" style="margin-top:8px">Used for the rider check and the accident snapshot.
    If <b>espcam.local</b> doesn't resolve on your network, type the camera's IP
    (shown in its Serial Monitor).</div>
</div>

<div id="msg"></div>

<script>
const $=id=>document.getElementById(id);
let lastRaw=0, gasEdited=false, spdEdited=false, camEdited=false;
function pill(el,t,c){el.textContent=t;el.className='pill '+(c||'');}
function say(t){$('msg').textContent=t;}
async function call(u){
  try{const r=await fetch(u,{cache:'no-store'});
      if(r.status===403){location.href='/';return;}      // locked again
      const t=await r.text();say(t);poll();}
  catch(e){say('Request failed - is the ESP32 still on the Wi-Fi?');}
}
function learn(){call('/api/rfid-learn');}
function clearRfid(){ if(confirm('Forget the unlock card?')) call('/api/rfid-clear'); }
function setUid(){ call('/api/rfid-set?uid='+encodeURIComponent($('uidin').value)); }
function setGas(){ gasEdited=false; call('/api/gas-set?v='+encodeURIComponent($('gasin').value)); }
function setSpd(){ spdEdited=false; call('/api/speed-set?v='+encodeURIComponent($('spdin').value)); }
function setCam(){ camEdited=false; call('/api/cam-set?host='+encodeURIComponent($('camin').value)); }
function useLive(){ $('gasin').value=lastRaw; gasEdited=true; }
$('gasin').addEventListener('input',()=>{gasEdited=true;});
$('spdin').addEventListener('input',()=>{spdEdited=true;});
$('camin').addEventListener('input',()=>{camEdited=true;});

async function poll(){
  try{
    const r=await fetch('/api/settings',{cache:'no-store'});
    if(r.status===403){ location.href='/'; return; }
    const d=await r.json();
    lastRaw=d.gasRaw;
    $('slim').textContent=d.speedLimit+' km/h';
    $('camh').textContent=d.camHost||'espcam.local (default)';
    if(!camEdited && document.activeElement!==$('camin')) $('camin').value=d.camHost;
    if(!spdEdited && document.activeElement!==$('spdin')) $('spdin').value=d.speedLimit;
    if(d.tgBusy) pill($('tgst'),'sending...','warn');
    else if(d.tg===0) pill($('tgst'),'nothing sent yet','');
    else if(d.tg===200) pill($('tgst'),'sent OK','ok');
    else pill($('tgst'),'failed ('+d.tg+')','bad');
    pill($('rstat'),d.rfidSet?'set':'NOT SET',d.rfidSet?'ok':'bad');
    $('ruid').textContent=d.rfidSet?d.rfid:'--';
    $('warn').style.display=d.rfidSet?'none':'block';
    $('learnmsg').textContent=d.learn
      ?'Hold the card on the reader now... ('+Math.ceil(d.learnLeftMs/1000)+' s)':'';
    $('learnbtn').textContent=d.learn?'Waiting for card...':'Scan card to set';
    $('graw').textContent=d.gasRaw;
    $('gthr').textContent=d.gasThr;
    pill($('gstat'),d.gasRaw>d.gasThr?'ALARM':'normal',d.gasRaw>d.gasThr?'bad':'ok');
    if(!gasEdited && document.activeElement!==$('gasin')) $('gasin').value=d.gasThr;
  }catch(e){}
}
setInterval(poll,700); poll();
</script></body></html>)HTML";

// ==========================================================================
//  HTTP handlers
// ==========================================================================
// ---- Access control ----
// Dashboard/data/actions: only when a card is registered AND has been scanned.
// Settings: also open while NO card is registered (first-run setup).
bool isSet()      { return rfidUid.length() > 0; }
bool dashOK()     { return isSet() && unlocked; }
bool settingsOK() { return !isSet() || unlocked; }

// Ignition: needs the dashboard unlocked, a passed rider check, no alcohol now,
// and no active accident alert.
bool ignitionReady() { return dashOK() && rcPass() && !alcoholNow() && !accidentActive; }

void applyIgnition() {
#if IGNITION_PIN >= 0
  bool on = ignitionReady();
  digitalWrite(IGNITION_PIN, (on == IGNITION_ACTIVE_HIGH) ? HIGH : LOW);
#endif
}

bool guard(bool ok) {
  if (ok) return true;
  server.sendHeader("Cache-Control", "no-store");
  server.send(403, "application/json", "{\"locked\":true}");
  return false;
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", dashOK() ? INDEX_HTML : LOCK_HTML);
}

void handleData() {
  if (!guard(dashOK())) return;
  Motion m = readMotion();
  float mag = m.valid
      ? sqrt(m.ax * m.ax + m.ay * m.ay + m.az * m.az) / 9.80665f : 0;

  char utc[24] = "--";
  if (gps.date.isValid() && gps.time.isValid()) {
    snprintf(utc, sizeof(utc), "%04u-%02u-%02u %02u:%02u:%02u",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  const char* mode = mpuMode == MPU_ADAFRUIT ? "library"
                   : mpuMode == MPU_RAW      ? "direct" : "none";

  char b[2800];
  size_t n = 0;
  n += snprintf(b + n, sizeof(b) - n,
      "{\"att\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},",
      roll, pitch, yawDisplay());
  n += snprintf(b + n, sizeof(b) - n,
      "\"imu\":{\"ok\":%s,\"mode\":\"%s\",\"whoami\":%u,"
      "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
      "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
      "\"mag\":%.3f,\"peak\":%.2f,\"temp\":%.1f},",
      m.valid ? "true" : "false", mode, mpuWhoAmI,
      m.ax, m.ay, m.az,
      (m.gx - gBias[0]) * RAD_TO_DEG,
      (m.gy - gBias[1]) * RAD_TO_DEG,
      (m.gz - gBias[2]) * RAD_TO_DEG,
      mag, peakG, m.tempC);
  int gasRaw = analogRead(GAS_PIN);
  n += snprintf(b + n, sizeof(b) - n,
      "\"env\":{\"gasRaw\":%d,\"gasMv\":%lu,\"gasThr\":%u,\"gasAlarm\":%s,"
      "\"dark\":%s,\"touch\":%s},",
      gasRaw, (unsigned long)analogReadMilliVolts(GAS_PIN),
      (unsigned)gasThreshold,
      gasRaw > gasThreshold ? "true" : "false",
      ldrDark() ? "true" : "false",
      touched()  ? "true" : "false");
  n += snprintf(b + n, sizeof(b) - n,
      "\"gps\":{\"fix\":%s,\"lat\":%.6f,\"lng\":%.6f,\"sats\":%lu,"
      "\"hdop\":%.1f,\"altM\":%.1f,\"kph\":%.1f,\"course\":%.1f,"
      "\"chars\":%lu,\"utc\":\"%s\"},",
      gps.location.isValid() ? "true" : "false",
      gps.location.isValid() ? gps.location.lat() : 0.0,
      gps.location.isValid() ? gps.location.lng() : 0.0,
      (unsigned long)gps.satellites.value(),
      gps.hdop.isValid() ? gps.hdop.hdop() : 0.0,
      gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
      gps.speed.isValid() ? gps.speed.kmph() : 0.0,
      gps.course.isValid() ? gps.course.deg() : 0.0,
      (unsigned long)gps.charsProcessed(), utc);
  n += snprintf(b + n, sizeof(b) - n,
      "\"ign\":{\"ready\":%s,\"state\":%u,\"valid\":%s,\"passed\":%s,\"helmet\":%s,"
      "\"riders\":%d,\"maxRiders\":%u,\"drowsy\":%s,\"alcohol\":%s,"
      "\"note\":\"%s\",\"err\":\"%s\",\"snapId\":%lu},",
      ignitionReady() ? "true" : "false", (unsigned)rcState,
      rcValid() ? "true" : "false", rcPass() ? "true" : "false",
      rcHelmet ? "true" : "false", rcRiders, (unsigned)MAX_RIDERS,
      rcDrowsy ? "true" : "false", alcoholNow() ? "true" : "false",
      rcNote, rcError, (unsigned long)snapId);
  n += snprintf(b + n, sizeof(b) - n,
      "\"warn\":{\"speedLimit\":%u,\"overSpeed\":%s,\"accident\":%s,"
      "\"tg\":%d,\"tgBusy\":%s},",
      (unsigned)speedLimit, overSpeed() ? "true" : "false",
      accidentActive ? "true" : "false", (int)tgLastCode, tgBusy ? "true" : "false");
  n += snprintf(b + n, sizeof(b) - n,
      "\"rfid\":{\"ok\":%s,\"set\":%s,\"unlocked\":%s,\"uid\":\"%s\",\"count\":%lu,\"agoMs\":%lu},",
      rfidOK ? "true" : "false", rfidUid.length() ? "true" : "false",
      unlocked ? "true" : "false", lastUid.c_str(),
      (unsigned long)cardCount,
      (unsigned long)(lastCardMs ? millis() - lastCardMs : 0));
  n += snprintf(b + n, sizeof(b) - n,
      "\"sys\":{\"ip\":\"%s\",\"rssi\":%d,\"upMs\":%lu,\"heap\":%lu,"
      "\"oled\":%s}}",
      WiFi.localIP().toString().c_str(), WiFi.RSSI(),
      (unsigned long)millis(), (unsigned long)ESP.getFreeHeap(),
      oledOK ? "true" : "false");

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

void handleSettingsJson() {
  if (!guard(settingsOK())) return;
  int raw = analogRead(GAS_PIN);
  char b[520];
  snprintf(b, sizeof(b),
      "{\"camHost\":\"%s\",\"rfidSet\":%s,\"rfid\":\"%s\",\"learn\":%s,\"learnLeftMs\":%lu,"
      "\"gasThr\":%u,\"gasRaw\":%d,\"speedLimit\":%u,"
      "\"tg\":%d,\"tgBusy\":%s,\"unlocked\":%s}",
      camHost.c_str(),
      rfidUid.length() ? "true" : "false", rfidUid.c_str(),
      rfidLearn ? "true" : "false",
      (unsigned long)(rfidLearn && rfidLearnUntil > millis() ? rfidLearnUntil - millis() : 0),
      (unsigned)gasThreshold, raw, (unsigned)speedLimit,
      (int)tgLastCode, tgBusy ? "true" : "false", unlocked ? "true" : "false");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

// Parse a plain non-negative integer argument; -1 if missing/garbage.
long intArg(const char* name, size_t maxDigits) {
  String v = server.arg(name);
  if (v.length() == 0 || v.length() > maxDigits) return -1;
  for (size_t i = 0; i < v.length(); i++) if (!isdigit((unsigned char)v[i])) return -1;
  return v.toInt();
}

void setupRoutes() {
  server.on("/", handleRoot);
  server.on("/data", handleData);

  // ---- always open: the lock page polls this ----
  server.on("/api/lock-status", []() {
    char b[100];
    snprintf(b, sizeof(b), "{\"rfidSet\":%s,\"unlocked\":%s,\"denied\":%s}",
             isSet() ? "true" : "false", unlocked ? "true" : "false",
             millis() < deniedUntil ? "true" : "false");
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", b);
  });

  // ---- settings (first-run setup, or after unlocking) ----
  server.on("/settings", []() {
    if (!settingsOK()) {                      // locked: go to the lock page
      server.sendHeader("Location", "/");
      server.send(302, "text/plain", "locked");
      return;
    }
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html", SETTINGS_HTML);
  });
  server.on("/api/settings", handleSettingsJson);

  server.on("/api/rfid-learn", []() {
    if (!guard(settingsOK())) return;
    rfidLearn = true;
    rfidLearnUntil = millis() + RFID_LEARN_MS;
    server.send(200, "text/plain", rfidOK
        ? "Hold the new card on the reader now (20 s)..."
        : "RFID reader not detected - check wiring, or type the UID by hand.");
  });
  server.on("/api/rfid-clear", []() {
    if (!guard(settingsOK())) return;
    saveRfid("");
    rfidLearn = false;
    unlocked = false;
    server.send(200, "text/plain", "RFID card cleared. Set a new one to lock the helmet again.");
  });
  server.on("/api/rfid-set", []() {
    if (!guard(settingsOK())) return;
    // Accept "04A31B2C", "04:a3:1b:2c", "04 A3 1B 2C" ...
    String raw = server.arg("uid"), hex;
    for (size_t i = 0; i < raw.length(); i++)
      if (isxdigit((unsigned char)raw[i])) hex += (char)toupper(raw[i]);
    size_t n = hex.length();
    if (n != 8 && n != 14 && n != 20) {
      server.send(400, "text/plain",
                  "Invalid UID - need 4, 7 or 10 bytes in hex (e.g. 04:A3:1B:2C).");
      return;
    }
    String uid;
    for (size_t i = 0; i < n; i += 2) {
      if (i) uid += ':';
      uid += hex.substring(i, i + 2);
    }
    saveRfid(uid);
    unlocked = true;                          // keep the settings page usable
    beep(200);
    server.send(200, "text/plain", "RFID card saved: " + uid);
  });
  server.on("/api/gas-set", []() {
    if (!guard(settingsOK())) return;
    long n = intArg("v", 4);
    if (n < 0 || n > 4095) {
      server.send(400, "text/plain", "Invalid threshold - enter a number from 0 to 4095.");
      return;
    }
    saveGasThreshold((uint16_t)n);
    beep(120);
    server.send(200, "text/plain", "Alcohol threshold saved: " + String(n));
  });
  server.on("/api/speed-set", []() {
    if (!guard(settingsOK())) return;
    long n = intArg("v", 3);
    if (n < 1 || n > 300) {
      server.send(400, "text/plain", "Invalid speed limit - enter 1 to 300 km/h.");
      return;
    }
    saveSpeedLimit((uint16_t)n);
    beep(120);
    server.send(200, "text/plain", "Speed limit saved: " + String(n) + " km/h");
  });
  server.on("/api/telegram-test", []() {
    if (!guard(settingsOK())) return;
    if (tgBusy) { server.send(409, "text/plain", "Still sending, try again in a moment."); return; }
    bool ok = startTelegram("Test message from Project Helmet - Telegram is working.",
                            false, 0, 0, false);
    server.send(ok ? 200 : 503, "text/plain",
                ok ? "Sending test message... watch the status above."
                   : "Cannot send - the ESP32 is not connected to Wi-Fi.");
  });

  // ---- dashboard actions (unlocked only) ----
  server.on("/api/cam-set", []() {
    if (!guard(settingsOK())) return;
    String h = server.arg("host");
    h.trim();
    bool okChars = h.length() <= 40;
    for (size_t i = 0; i < h.length(); i++)
      if (!isalnum((unsigned char)h[i]) && h[i] != '.' && h[i] != '-') okChars = false;
    if (!okChars) {
      server.send(400, "text/plain", "Invalid address - use a hostname or IP (letters, digits, . and -).");
      return;
    }
    saveCamHost(h);
    server.send(200, "text/plain", h.length() ? "Camera address saved: " + h
                                              : String("Camera address reset to espcam.local"));
  });

  // ---- rider check + ignition (unlocked only) ----
  server.on("/api/rider-check", []() {
    if (!guard(dashOK())) return;
    if (rcBusy()) { server.send(409, "text/plain", "A check is already running."); return; }
    if (!startRiderCheck()) {
      server.send(503, "text/plain", "Cannot start - the ESP32 is not connected to Wi-Fi.");
      return;
    }
    server.send(200, "text/plain", "Checking rider: photo, then AI analysis (about 10 s)...");
  });
  server.on("/api/snapshot.jpg", []() {
    if (!guard(dashOK())) return;
    if (xSemaphoreTake(snapMux, pdMS_TO_TICKS(1000)) != pdTRUE) {
      server.send(503, "text/plain", "busy");
      return;
    }
    if (!snapBuf) {
      xSemaphoreGive(snapMux);
      server.send(404, "text/plain", "no snapshot yet");
      return;
    }
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "image/jpeg", (const char*)snapBuf, snapLen);
    xSemaphoreGive(snapMux);
  });

  server.on("/api/lock", []() {
    if (!guard(dashOK())) return;
    unlocked = false;
    server.send(200, "text/plain", "locked");
  });
  server.on("/api/accident-clear", []() {
    if (!guard(dashOK())) return;
    accidentActive = false;
    accidentPending = false;
    tiltSince = 0;
    server.send(200, "text/plain", "accident alert cleared");
  });
  server.on("/api/zero-yaw", []() {
    if (!guard(dashOK())) return;
    yawOffset = yaw;
    beep(60);
    server.send(200, "text/plain", "yaw zeroed");
  });
  server.on("/api/calibrate", []() {
    if (!guard(dashOK())) return;
    // Blocks ~1.5 s. The helmet must be still or the bias is wrong.
    calibrateGyro();
    beep(150);
    server.send(200, "text/plain", "gyro calibrated");
  });
  server.on("/api/reset-peak", []() {
    if (!guard(dashOK())) return;
    peakG = 0;
    server.send(200, "text/plain", "peak cleared");
  });
  server.on("/api/beep", []() {
    if (!guard(dashOK())) return;
    beep(200);
    server.send(200, "text/plain", "beep");
  });
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
}

// ==========================================================================
//  OLED
// ==========================================================================
void initOLED() {
  oledAddr = 0;
  if      (i2cPresent(0x3C)) oledAddr = 0x3C;
  else if (i2cPresent(0x3D)) oledAddr = 0x3D;
  if (!oledAddr) { oledOK = false; return; }
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, oledAddr);
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.display();
}

void oledMessage(const char* l1, const char* l2) {
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(l1);
  if (l2) oled.println(l2);
  oled.display();
}

// Settings URL for the OLED (no "http://" — 21 chars per line is all we get).
String urlIp()   { return WiFi.localIP().toString(); }
String urlFull() { return urlIp() + "/settings"; }

// One-line URL; falls back to just the IP if "IP/settings" would wrap.
String urlLine() {
  if (WiFi.status() != WL_CONNECTED) return "WiFi offline";
  String full = urlFull();
  return full.length() <= 21 ? full : urlIp();
}

void drawOLED() {
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setCursor(0, 0);

  // ---- accident: overrides everything, even while locked ----
  if (accidentActive) {
    oled.println(F("!!! ACCIDENT !!!"));
    oled.println();
    oled.print(F("Telegram: "));
    oled.println(accidentPending ? (tgBusy ? F("sending...") : F("retrying..."))
                                 : (tgLastCode == 200 ? F("SENT") : F("FAILED")));
    oled.println();
    if (gps.location.isValid()) {
      oled.printf("%.5f\n%.5f\n", gps.location.lat(), gps.location.lng());
    } else {
      oled.println(F("GPS: no fix"));
    }
    oled.display();
    return;
  }

  // ---- learning a card ----
  if (rfidLearn) {
    oled.println(F("== SET RFID CARD =="));
    oled.println();
    oled.println(F("Hold the new card"));
    oled.println(F("on the reader now..."));
    oled.println();
    oled.printf("%lu s left\n",
                (unsigned long)((rfidLearnUntil > millis() ? rfidLearnUntil - millis() : 0) / 1000));
    oled.display();
    return;
  }

  // ---- first run: no card registered ----
  if (!isSet()) {
    oled.println(F("** RFID NOT SET **"));
    if (WiFi.status() != WL_CONNECTED) {
      oled.println();
      oled.println(F("WiFi connecting..."));
      oled.printf("Network: %s\n", WIFI_SSID);
    } else {
      oled.println(F("Open this URL in a"));
      oled.println(F("browser (same WiFi)"));
      oled.println(F("and set your RFID:"));
      oled.println();
      String full = urlFull();
      if (full.length() <= 21) {
        oled.println(full);
      } else {                                       // long IP: split over two lines
        oled.println(urlIp());
        oled.println(F("/settings"));
      }
    }
    oled.display();
    return;
  }

  // ---- locked: waiting for the registered card ----
  if (!unlocked) {
    oled.println(F("===== LOCKED ====="));
    oled.println(millis() < deniedUntil ? F("ACCESS DENIED") : F(""));
    oled.println(F("Scan your RFID card"));
    oled.println(F("to unlock."));
    oled.println();
    oled.println(F("Dashboard URL:"));
    oled.println(urlLine());
    oled.display();
    return;
  }

  // ---- unlocked: normal status screen ----
  int   gas   = analogRead(GAS_PIN);
  bool  alarm = gas > gasThreshold;
  float kph   = (gps.speed.isValid() && gps.speed.age() < 5000) ? gps.speed.kmph() : 0.0f;

  oled.println(urlLine());                           // URL always visible
  oled.printf("R%6.1f P%6.1f\n", roll, pitch);
  oled.printf("Y%6.1f  %ddBm\n", yawDisplay(), WiFi.RSSI());
  oled.printf("gas %4d/%-4u %s\n", gas, (unsigned)gasThreshold, ldrDark() ? "DARK" : "LIT");
  oled.println(alarm ? F("!! ALCOHOL !!") : F("alcohol: ok"));
  if (overSpeed()) oled.printf("!!SPEED %.0f/%u!!\n", kph, (unsigned)speedLimit);
  else             oled.printf("spd %.0f/%u km/h\n", kph, (unsigned)speedLimit);
  if (gps.location.isValid()) {
    oled.printf("sat %lu  %.4f\n", (unsigned long)gps.satellites.value(),
                gps.location.lat());
  } else {
    oled.printf("gps searching %lu\n", (unsigned long)gps.satellites.value());
  }
  oled.printf("ign: %s", rcBusy() ? "checking..." : ignitionReady() ? "READY" : "LOCKED");
  oled.display();
}

// ==========================================================================
//  Wi-Fi (station mode: joins the existing "robot" network)
// ==========================================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);                 // keeps the dashboard responsive
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("Joining \"%s\"", WIFI_SSID);
  oledMessage("WiFi connecting", WIFI_SSID);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected. IP %s  RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    Serial.printf("Dashboard: http://%s/   Settings: http://%s/settings\n",
                  WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());
    if (MDNS.begin(HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("Also at http://%s.local/\n", HOSTNAME);
    }
    oledMessage("WiFi OK", WiFi.localIP().toString().c_str());
  } else {
    Serial.println(F("Wi-Fi FAILED — check SSID/password. Retrying in loop."));
    Serial.println(F("Note: ESP32 is 2.4 GHz only; a 5 GHz-only SSID cannot work."));
    oledMessage("WiFi FAILED", "check credentials");
  }
}

// ==========================================================================
//  Setup / loop
// ==========================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n\n=== Project Helmet — Wi-Fi telemetry ==="));

  pinMode(BUZZER_PIN, OUTPUT);
  buzzer(false);
  snapMux = xSemaphoreCreateMutex();
#if IGNITION_PIN >= 0
  pinMode(IGNITION_PIN, OUTPUT);
  digitalWrite(IGNITION_PIN, IGNITION_ACTIVE_HIGH ? LOW : HIGH);   // start locked
#endif
  pinMode(LDR_PIN,   INPUT);
  pinMode(TOUCH_PIN, INPUT);

  analogReadResolution(12);
  // GPIO32 is ADC1, which is the whole reason this works with Wi-Fi running.
  // ADC2 pins stop reading the moment the radio comes up.
  analogSetPinAttenuation(GAS_PIN, ADC_11db);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(150);

  initOLED();
  Serial.printf("OLED  %s (0x%02X)\n", oledOK ? "ok" : "FAIL", oledAddr);
  initMPU();
  Serial.printf("MPU   %s  WHO_AM_I 0x%02X  mode %s\n",
                mpuMode != MPU_NONE ? "ok" : "FAIL", mpuWhoAmI,
                mpuMode == MPU_ADAFRUIT ? "library" :
                mpuMode == MPU_RAW      ? "direct"  : "none");
  initRFID();
  Serial.printf("RC522 %s (0x%02X)\n", rfidOK ? "ok" : "FAIL", rfidVersion);

  oledMessage("Calibrating gyro", "hold still...");
  calibrateGyro();

  loadSettings();
  Serial.printf("RFID card: %s   gas threshold: %u   speed limit: %u km/h\n",
                rfidUid.length() ? rfidUid.c_str() : "NOT SET",
                (unsigned)gasThreshold, (unsigned)speedLimit);

  connectWiFi();
  setupRoutes();
  server.begin();
  Serial.println(F("HTTP server up on port 80"));
  drawOLED();
  beep(120);

  lastImu = lastOled = millis();
}

void loop() {
  feedGPS();                    // constantly, or NMEA bytes are dropped
  server.handleClient();
  serviceBeep();
  pollRFID();

  // Touch edge -> short beep, same behaviour as the self-test sketch.
  bool t = touched();
  if (t && !lastTouch) beep(60);
  lastTouch = t;

  if (millis() - lastImu >= IMU_MS) {
    lastImu = millis();
    updateAttitude();
    checkAccident();
  }
  serviceTelegram();

  // Locking the dashboard also throws away a passed rider check.
  static bool prevUnlocked = false;
  if (prevUnlocked && !unlocked && !rcBusy()) rcState = RC_IDLE;
  prevUnlocked = unlocked;
  applyIgnition();

  if (millis() - lastOled >= OLED_MS) {
    lastOled = millis();
    drawOLED();
  }

  // Non-blocking reconnect if the AP drops.
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > 10000) {
    lastWifiTry = millis();
    Serial.println(F("Wi-Fi lost — reconnecting"));
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  // Alcohol alarm: beep once a second while the reading is over the threshold.
  if (millis() - lastGasBeep >= 1000) {
    lastGasBeep = millis();
    if (analogRead(GAS_PIN) > gasThreshold) beep(300);
  }
}
