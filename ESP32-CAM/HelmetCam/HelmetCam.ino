/* ==========================================================================
 *  Project Helmet — Rider Camera (ESP32-CAM, AI-Thinker)
 *
 *  Takes a photo of the rider, sends it to a vision-language model (VLM)
 *  agent hosted on DigitalOcean, and reports:
 *      helmet worn?   drowsy?   eyes open / half / closed
 *
 *  Endpoints (port 80)
 *    GET /                         small status page with a live snapshot
 *    GET /capture?flash=0|1        fresh JPEG
 *    GET /last.jpg                 the photo used in the most recent check
 *    GET /check?token=..&flash=0|1 capture -> VLM -> JSON result
 *        -> {"ok":true,"face":true,"helmet":true,"drowsy":false,
 *            "eyes":"open","confidence":0.87,"notes":"...","ms":6120}
 *        -> {"ok":false,"error":"..."}
 *
 *  The main helmet ESP32 calls /check when "Check rider" is pressed on the
 *  dashboard. The dashboard shows /last.jpg straight from this camera.
 *
 *  VLM API: OpenAI-compatible chat completions with an image_url data URI.
 *  DigitalOcean GenAI agents and most self-hosted servers (vLLM, Ollama's
 *  OpenAI endpoint, LiteLLM) accept this format. The model must support
 *  image input. If your agent expects something else, only
 *  buildVlmBody() and parseVlmReply() need changing.
 *
 *  Arduino IDE settings
 *    Board: "AI Thinker ESP32-CAM"   PSRAM: Enabled
 *    Partition scheme: "Huge APP (3MB No OTA)"
 *  Libraries: ArduinoJson (v7). Camera, WiFi, WebServer, HTTPClient,
 *             WiFiClientSecure are part of the ESP32 core.
 *
 *  POWER: give the ESP32-CAM a solid 5 V / 1 A+. Wi-Fi TX + camera + flash
 *  LED on a weak supply causes brownout resets mid-upload.
 * ========================================================================== */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "mbedtls/base64.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ==========================  FILL THESE IN  ==============================
const char* WIFI_SSID = "Room_401A-2.4G";          // same network as helmet
const char* WIFI_PASS = "HLiNaKRbCsFr@gr01";
const char* HOSTNAME  = "helmetcam";

// Shared secret so random devices on the LAN can't burn your VLM credits.
// Must match CAM_TOKEN in HelmetDashboard.ino.
const char* CAM_TOKEN = "helmet-cam-7f3a9c";

// VLM agent on DigitalOcean (DUMMY values — replace with your agent's
// endpoint and access key from the DigitalOcean control panel).
const char* VLM_URL   = "https://a1b2c3d4e5f6g7h8i9j0.agents.do-ai.run/api/v1/chat/completions";
const char* VLM_KEY   = "do-agent-key-DUMMY-3f9a1c7e5b2d8f4a6c0e9b1d7f3a5c8e";
const char* VLM_MODEL = "helmet-rider-vlm";         // ignored by many agents

// Optional fixed IP so the helmet always finds the camera.
// Leave false to use DHCP (then copy the printed IP into CAM_BASE_URL).
const bool      USE_STATIC_IP = false;
IPAddress       STATIC_IP (192, 168, 1, 60);
IPAddress       GATEWAY   (192, 168, 1, 1);
IPAddress       SUBNET    (255, 255, 255, 0);
IPAddress       DNS1      (8, 8, 8, 8);
// =========================================================================

// ---------------- AI-Thinker ESP32-CAM pin map ----------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4

// ---------------- VLM prompt ----------------
const char* VLM_SYSTEM =
  "You are a road-safety vision checker for a motorcycle rider-monitoring "
  "system. You receive one photo from a camera facing the rider. Reply with "
  "ONLY one JSON object: no markdown, no code fences, no extra text. Use "
  "exactly these keys: "
  "face_visible (boolean: a person's face or head is visible), "
  "helmet (boolean: the person is wearing a motorcycle or bicycle helmet ON "
  "their head; a helmet held in the hand or lying nearby is false), "
  "drowsy (boolean: signs of drowsiness such as closed or half-closed eyes, "
  "a drooping or nodding head, or yawning), "
  "eyes (string: one of open, half, closed, not_visible), "
  "confidence (number from 0 to 1 for your overall judgement), "
  "notes (string, under 20 words, what you based the decision on). "
  "If no person is visible set face_visible false, helmet false, drowsy "
  "false, eyes not_visible, confidence 0.";

const char* VLM_USER =
  "Check this rider: is a helmet being worn, and does the rider look drowsy?";

// ---------------- State ----------------
WebServer server(80);
bool      camOK = false;
uint8_t*  lastJpg = nullptr;     // copy of last analysed photo (PSRAM)
size_t    lastLen = 0, lastCap = 0;
uint32_t  lastWifiTry = 0, checks = 0;

// ==========================================================================
//  Camera
// ==========================================================================
bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk  = XCLK_GPIO_NUM;
  c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href  = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    // VGA is plenty for a face and keeps the upload ~30-60 KB.
    c.frame_size   = FRAMESIZE_VGA;
    c.jpeg_quality = 12;                 // lower = better quality, bigger
    c.fb_count     = 2;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
    c.grab_mode    = CAMERA_GRAB_LATEST; // never hand out a stale frame
  } else {
    Serial.println(F("WARNING: no PSRAM — enable it in Tools menu. Using QVGA."));
    c.frame_size   = FRAMESIZE_QVGA;
    c.jpeg_quality = 14;
    c.fb_count     = 1;
    c.fb_location  = CAMERA_FB_IN_DRAM;
    c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s, 1);
  s->set_saturation(s, 0);
  s->set_whitebal(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_gain_ctrl(s, 1);
  // Mounted upside down or mirrored? Uncomment as needed:
  // s->set_vflip(s, 1);
  // s->set_hmirror(s, 1);
  return true;
}

// Capture a fresh frame and keep a copy in lastJpg.
bool captureToLast(bool flash, String& err) {
  if (!camOK) { err = "camera not initialised"; return false; }

  if (flash) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(120);
    // Let auto-exposure adapt to the flash: throw away two frames.
    for (int i = 0; i < 2; i++) {
      camera_fb_t* f = esp_camera_fb_get();
      if (f) esp_camera_fb_return(f);
    }
  }
  camera_fb_t* fb = esp_camera_fb_get();
  if (flash) digitalWrite(FLASH_LED_PIN, LOW);
  if (!fb) { err = "capture failed"; return false; }

  if (fb->len > lastCap) {
    uint8_t* nb = (uint8_t*)(psramFound() ? ps_malloc(fb->len) : malloc(fb->len));
    if (!nb) { esp_camera_fb_return(fb); err = "out of memory for photo"; return false; }
    free(lastJpg);
    lastJpg = nb;
    lastCap = fb->len;
  }
  memcpy(lastJpg, fb->buf, fb->len);
  lastLen = fb->len;
  esp_camera_fb_return(fb);
  return true;
}

// ==========================================================================
//  VLM call
// ==========================================================================
// Builds the whole request body in one PSRAM buffer:
//   {... "image_url":{"url":"data:image/jpeg;base64,<BASE64>"} ...}
// ArduinoJson writes everything except the image; the base64 is spliced in
// at a placeholder so we never hold two copies of the photo as Strings.
uint8_t* buildVlmBody(const uint8_t* jpg, size_t len, size_t& outLen) {
  JsonDocument doc;
  doc["model"]       = VLM_MODEL;
  doc["temperature"] = 0;
  doc["max_tokens"]  = 200;
  JsonArray msgs = doc["messages"].to<JsonArray>();

  JsonObject sys = msgs.add<JsonObject>();
  sys["role"]    = "system";
  sys["content"] = VLM_SYSTEM;

  JsonObject usr = msgs.add<JsonObject>();
  usr["role"] = "user";
  JsonArray parts = usr["content"].to<JsonArray>();
  JsonObject txt = parts.add<JsonObject>();
  txt["type"] = "text";
  txt["text"] = VLM_USER;
  JsonObject img = parts.add<JsonObject>();
  img["type"] = "image_url";
  img["image_url"]["url"] = "data:image/jpeg;base64,@@IMG@@";

  String json;
  serializeJson(doc, json);
  int at = json.indexOf("@@IMG@@");
  if (at < 0) return nullptr;
  String prefix = json.substring(0, at);
  String suffix = json.substring(at + 7);

  size_t b64Need = 0;
  mbedtls_base64_encode(nullptr, 0, &b64Need, jpg, len);   // includes NUL

  size_t cap = prefix.length() + b64Need + suffix.length() + 1;
  uint8_t* buf = (uint8_t*)(psramFound() ? ps_malloc(cap) : malloc(cap));
  if (!buf) return nullptr;

  memcpy(buf, prefix.c_str(), prefix.length());
  size_t written = 0;
  if (mbedtls_base64_encode(buf + prefix.length(), b64Need, &written, jpg, len) != 0) {
    free(buf);
    return nullptr;
  }
  memcpy(buf + prefix.length() + written, suffix.c_str(), suffix.length());
  outLen = prefix.length() + written + suffix.length();
  buf[outLen] = 0;
  return buf;
}

// Pull choices[0].message.content, then the JSON object inside it (models
// sometimes wrap it in ```json fences despite being told not to).
bool parseVlmReply(const String& resp, JsonDocument& result, String& err) {
  JsonDocument outer;
  if (deserializeJson(outer, resp)) { err = "VLM sent invalid JSON"; return false; }

  const char* content = outer["choices"][0]["message"]["content"] | (const char*)nullptr;
  if (!content) {
    const char* apiErr = outer["error"]["message"] | (const char*)nullptr;
    err = apiErr ? String("VLM error: ") + apiErr : String("VLM reply had no content");
    return false;
  }
  String c = content;
  int a = c.indexOf('{'), b = c.lastIndexOf('}');
  if (a < 0 || b <= a) { err = "VLM did not answer in JSON"; return false; }
  if (deserializeJson(result, c.substring(a, b + 1))) {
    err = "VLM answer was not valid JSON";
    return false;
  }
  return true;
}

bool askVLM(const uint8_t* jpg, size_t len, JsonDocument& result, String& err) {
  size_t bodyLen = 0;
  uint8_t* body = buildVlmBody(jpg, len, bodyLen);
  if (!body) { err = "out of memory building request"; return false; }

  WiFiClientSecure client;
  client.setInsecure();              // skips certificate check; hobby build
  HTTPClient http;
  http.setConnectTimeout(10000);
  http.setTimeout(40000);

  if (!http.begin(client, VLM_URL)) {
    free(body);
    err = "bad VLM_URL";
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + VLM_KEY);

  Serial.printf("VLM: uploading %u bytes (photo %u bytes)...\n",
                (unsigned)bodyLen, (unsigned)len);
  int code = http.POST(body, bodyLen);
  free(body);

  String resp = code > 0 ? http.getString() : "";
  http.end();

  if (code != 200) {
    if (code == 401 || code == 403) err = "VLM rejected the API key (HTTP " + String(code) + ")";
    else if (code < 0)              err = "cannot reach VLM (" + HTTPClient::errorToString(code) + ")";
    else                            err = "VLM HTTP " + String(code);
    Serial.println(resp.substring(0, 300));
    return false;
  }
  return parseVlmReply(resp, result, err);
}

// ==========================================================================
//  HTTP handlers
// ==========================================================================
void cors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
}

void sendJpeg(const uint8_t* buf, size_t len) {
  cors();
  server.setContentLength(len);
  server.send(200, "image/jpeg", "");
  server.client().write(buf, len);
}

void sendError(const String& msg) {
  JsonDocument d;
  d["ok"] = false;
  d["error"] = msg;
  String out;
  serializeJson(d, out);
  cors();
  server.send(200, "application/json", out);   // 200 so the helmet reads the message
  Serial.printf("CHECK failed: %s\n", msg.c_str());
}

void handleCapture() {
  String err;
  if (!captureToLast(server.arg("flash") == "1", err)) {
    cors();
    server.send(500, "text/plain", err);
    return;
  }
  sendJpeg(lastJpg, lastLen);
}

void handleLast() {
  if (!lastLen) { cors(); server.send(404, "text/plain", "no photo yet"); return; }
  sendJpeg(lastJpg, lastLen);
}

void handleCheck() {
  if (server.arg("token") != CAM_TOKEN) {
    cors();
    server.send(403, "application/json", "{\"ok\":false,\"error\":\"bad token\"}");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) { sendError("camera has no Wi-Fi"); return; }

  uint32_t t0 = millis();
  String err;
  if (!captureToLast(server.arg("flash") == "1", err)) { sendError(err); return; }

  JsonDocument v;
  if (!askVLM(lastJpg, lastLen, v, err)) { sendError(err); return; }

  checks++;
  JsonDocument out;
  out["ok"]         = true;
  out["face"]       = v["face_visible"] | true;
  out["helmet"]     = v["helmet"]       | false;
  out["drowsy"]     = v["drowsy"]       | false;
  out["eyes"]       = v["eyes"]         | "unknown";
  out["confidence"] = v["confidence"]   | 0.0f;
  out["notes"]      = v["notes"]        | "";
  out["ms"]         = millis() - t0;
  out["bytes"]      = lastLen;

  String s;
  serializeJson(out, s);
  cors();
  server.send(200, "application/json", s);
  Serial.printf("CHECK ok: %s\n", s.c_str());
}

const char STATUS_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Helmet camera</title>
<style>
body{margin:0;padding:16px;background:#0e1116;color:#e6edf3;
     font:14px/1.5 system-ui,sans-serif;max-width:680px}
img{width:100%;border-radius:8px;border:1px solid #242c38;background:#000;min-height:120px}
button{background:#21262d;color:#e6edf3;border:1px solid #242c38;border-radius:7px;
       padding:8px 14px;cursor:pointer;margin:10px 8px 0 0}
p{color:#8b96a5}
</style></head><body>
<h1 style="font-size:18px">Helmet camera</h1>
<p>Aim the camera so the rider's face and head fill most of the frame.</p>
<img id="p" alt="Camera snapshot">
<div>
  <button onclick="shot(0)">Take photo</button>
  <button onclick="shot(1)">Take photo with flash</button>
</div>
<script>
function shot(f){document.getElementById('p').src='/capture?flash='+f+'&t='+Date.now();}
shot(0);
</script></body></html>)HTML";

void setupRoutes() {
  server.on("/", []() { server.send_P(200, "text/html", STATUS_HTML); });
  server.on("/capture", handleCapture);
  server.on("/last.jpg", handleLast);
  server.on("/check", handleCheck);
  server.onNotFound([]() { cors(); server.send(404, "text/plain", "not found"); });
}

// ==========================================================================
//  Wi-Fi
// ==========================================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);
  if (USE_STATIC_IP) WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS1);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Joining \"%s\"", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected. Camera at http://%s/  (RSSI %d dBm)\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    Serial.printf(">>> Put this in HelmetDashboard.ino: CAM_BASE_URL = \"http://%s\"\n",
                  WiFi.localIP().toString().c_str());
    if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println(F("Wi-Fi FAILED — retrying in loop (2.4 GHz only)."));
  }
}

// ==========================================================================
//  Setup / loop
// ==========================================================================
void setup() {
  // The flash LED + Wi-Fi burst can dip the rail and trip the brownout
  // detector on marginal supplies. Disabling it avoids reset loops, but a
  // proper 5 V supply is the real fix.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n\n=== Project Helmet — rider camera ==="));

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  camOK = initCamera();
  Serial.printf("Camera %s, PSRAM %s\n", camOK ? "ok" : "FAIL",
                psramFound() ? "yes" : "NO");

  connectWiFi();
  setupRoutes();
  server.begin();
  Serial.println(F("HTTP server up on port 80"));
}

void loop() {
  server.handleClient();

  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > 10000) {
    lastWifiTry = millis();
    Serial.println(F("Wi-Fi lost — reconnecting"));
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  delay(2);
}
