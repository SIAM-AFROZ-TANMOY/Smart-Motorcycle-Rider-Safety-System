/* ==========================================================================
 *  Project Helmet — ESP32-CAM snapshot server (AI-Thinker ESP32-CAM)
 *
 *  A dumb camera: it joins the same Wi-Fi as the helmet controller
 *  (updatedMain2.ino) and hands out JPEG snapshots on request. The controller
 *  is what sends them to the VLM agent and to Telegram.
 *
 *    http://espcam.local/              live preview page (if mDNS resolves)
 *    http://<cam-ip>/capture           one fresh JPEG
 *    http://<cam-ip>/capture?flash=1   same, with the on-board flash LED lit
 *    http://<cam-ip>/status            JSON (ip, rssi, uptime)
 *
 *  If the helmet dashboard keeps saying "Camera not reachable":
 *   1. Open this board's own Serial Monitor (115200 baud) and read the IP
 *      it prints every few seconds.
 *   2. On the helmet's /settings page, under "Camera (ESP32-CAM)", type that
 *      IP into the camera-address box instead of leaving it blank. This
 *      skips mDNS ("espcam.local") completely.
 *   3. If the Wi-Fi network is a PHONE HOTSPOT, check its "AP/client
 *      isolation" setting — many phones block devices from reaching each
 *      other even when both are connected, and no code change here can work
 *      around that. Use a normal Wi-Fi router if possible.
 *
 *  Arduino IDE: board "AI Thinker ESP32-CAM", PSRAM "Enabled".
 *  Needs only the ESP32 board package (esp_camera, WiFi, WebServer, ESPmDNS).
 *  Flashing needs GPIO0 tied to GND while uploading, then remove it and reset.
 * ========================================================================== */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

const char* WIFI_SSID = "Alfi";
const char* WIFI_PASS = "uiu#1234";
const char* HOSTNAME  = "espcam";            // -> http://espcam.local

// Set these if the picture is upside-down / mirrored for your mounting.
#define CAM_VFLIP    0
#define CAM_HMIRROR  0

// How many times to retry esp_camera_init() before giving up. The camera
// sensor draws a current spike on power-up; a marginal 5V supply can make
// the very first init attempt fail even though the hardware is fine.
#define CAM_INIT_RETRIES  3

// ---- AI-Thinker ESP32-CAM pin map ----
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
#define FLASH_LED_PIN      4                 // the bright white LED

WebServer server(80);
bool camOK = false;
uint32_t lastWifiTry = 0;
uint32_t lastIpPrint = 0;

const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM</title>
<style>
body{margin:0;padding:16px;background:#0e1116;color:#e6edf3;
     font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}
h1{font-size:18px;margin:0 0 10px}
img{max-width:100%;border-radius:10px;border:1px solid #242c38;display:block}
button{background:#21262d;color:#e6edf3;border:1px solid #242c38;border-radius:7px;
       padding:7px 12px;margin:10px 8px 0 0;cursor:pointer}
</style></head><body>
<h1>Helmet camera</h1>
<img id="im" alt="snapshot">
<button onclick="snap(0)">Snapshot</button>
<button onclick="snap(1)">Snapshot + flash</button>
<script>
function snap(f){document.getElementById('im').src='/capture?flash='+f+'&t='+Date.now();}
snap(0);
</script></body></html>)HTML";

bool initCameraOnce() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_VGA;            // 640x480: plenty for the VLM, ~30 KB
  c.jpeg_quality = 12;                       // 10 best .. 63 worst
  if (psramFound()) {
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }
  if (esp_camera_init(&c) != ESP_OK) return false;

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);
  }
  return true;
}

bool initCamera() {
  for (int i = 1; i <= CAM_INIT_RETRIES; i++) {
    if (initCameraOnce()) return true;
    Serial.printf("Camera init attempt %d/%d failed, retrying...\n", i, CAM_INIT_RETRIES);
    esp_camera_deinit();
    delay(300);
  }
  return false;
}

// Frames sit in a small queue; throw the stale one away so we send "now".
camera_fb_t* grabFresh() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
  return esp_camera_fb_get();
}

void handleCapture() {
  if (!camOK) { server.send(503, "text/plain", "camera not ready"); return; }

  bool flash = server.arg("flash") == "1";
  if (flash) { digitalWrite(FLASH_LED_PIN, HIGH); delay(180); }   // let exposure settle
  camera_fb_t* fb = grabFresh();
  if (flash) digitalWrite(FLASH_LED_PIN, LOW);

  if (!fb) { server.send(500, "text/plain", "capture failed"); return; }

  WiFiClient client = server.client();
  client.printf("HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\n"
                "Content-Length: %u\r\nCache-Control: no-store\r\n"
                "Connection: close\r\n\r\n", (unsigned)fb->len);
  const uint8_t* p = fb->buf;
  size_t left = fb->len;
  while (left && client.connected()) {       // chunked write: big buffers stall on lwIP
    size_t n = left > 1436 ? 1436 : left;
    size_t w = client.write(p, n);
    if (!w) break;
    p += w; left -= w;
  }
  esp_camera_fb_return(fb);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Joining \"%s\"", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(300); Serial.print('.'); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected. IP %s   (use this in the helmet's Camera address\n"
                  "field if espcam.local does not resolve)\n",
                  WiFi.localIP().toString().c_str());
    if (MDNS.begin(HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("Also at http://%s.local/\n", HOSTNAME);
    } else {
      Serial.println(F("mDNS failed to start - use the IP address above instead."));
    }
  } else {
    Serial.println(F("Wi-Fi FAILED - will keep retrying (2.4 GHz only)."));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== Helmet camera ==="));

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  camOK = initCamera();
  Serial.printf("Camera %s (PSRAM %s)\n", camOK ? "ok" : "FAILED", psramFound() ? "yes" : "no");
  if (!camOK) {
    Serial.println(F("Camera FAILED after retries - check the ribbon cable seating"));
    Serial.println(F("and the 5V supply (the sensor needs a solid 5V, not just USB-serial power)."));
  }

  connectWiFi();

  server.on("/", []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/capture", handleCapture);
  server.on("/status", []() {
    char b[160];
    snprintf(b, sizeof(b), "{\"camera\":%s,\"ip\":\"%s\",\"rssi\":%d,\"upMs\":%lu}",
             camOK ? "true" : "false", WiFi.localIP().toString().c_str(),
             WiFi.RSSI(), (unsigned long)millis());
    server.send(200, "application/json", b);
  });
  server.begin();
}

void loop() {
  server.handleClient();
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > 10000) {
    lastWifiTry = millis();
    Serial.println(F("Wi-Fi lost - reconnecting"));
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  // Reprint the IP every 10 s so it's easy to find in the Serial Monitor
  // without having to catch the one-time boot message.
  if (WiFi.status() == WL_CONNECTED && millis() - lastIpPrint > 10000) {
    lastIpPrint = millis();
    Serial.printf("IP %s   RSSI %d dBm   camera %s\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI(), camOK ? "ok" : "FAILED");
  }
}
