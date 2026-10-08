# ESP32 Sensor & Camera Dashboard

Two separate boards, one dashboard:

- **`esp32_sensor_dashboard/`** (this folder) — a regular ESP32 with an MQ3 alcohol/gas sensor and a touch sensor. Runs the lockable web dashboard.
- **`esp32_cam_stream/`** — an AI-Thinker ESP32-CAM. It's an independent device on the same Wi-Fi network that just serves an MJPEG video stream. The sensor dashboard embeds that stream by URL — the two boards don't talk to each other directly, they're just both reachable from your browser.

## Wiring (sensor board)

| Sensor | ESP32 pin | Notes |
|---|---|---|
| MQ3 gas sensor (analog out) | D32 | Also connect VCC/GND per module |
| Touch sensor (digital out, e.g. TTP223) | D33 | Also connect VCC/GND per module |

If you're using the ESP32's **built-in** capacitive touch pad on D33 instead of a separate touch module, set `USE_NATIVE_TOUCH = true` in the sketch and tune `NATIVE_TOUCH_THRESHOLD` (print `touchRead(33)` over Serial to find a good value — untouched is usually 60-90, touched drops well below that).

## Setup

### 1. Sensor board

1. Open `esp32_sensor_dashboard/esp32_sensor_dashboard.ino` in the Arduino IDE (ESP32 board package installed) or PlatformIO.
2. Set `WIFI_SSID` / `WIFI_PASS` at the top.
3. Select your ESP32 board and upload.
4. Open the Serial Monitor at 115200 baud — it prints the dashboard URL, e.g. `http://192.168.1.40`.

### 2. ESP32-CAM board (separate device)

1. Open `esp32_cam_stream/esp32_cam_stream.ino`.
2. Set `WIFI_SSID` / `WIFI_PASS` at the top (same network as the sensor board).
3. Board settings for AI-Thinker ESP32-CAM: board = "AI Thinker ESP32-CAM", partition scheme = "Huge APP (3MB No OTA/1MB SPIFFS)", PSRAM = "Enabled".
4. Wire GPIO0 to GND to put it in flashing mode, upload, then remove that wire and reset.
5. Open the Serial Monitor at 115200 baud — it prints the cam's IP and the exact stream URL, e.g. `http://192.168.1.55/stream`. You can sanity-check it standalone by opening `http://192.168.1.55/` in a browser first.

### 3. Merge them

1. Open the sensor dashboard in a browser and unlock it (touch the sensor).
2. Open **⚙ Settings**, paste the ESP32-CAM's stream URL (e.g. `http://192.168.1.55/stream`) into **Camera stream URL**, and Save.
3. The dashboard's Camera Feed card now shows the live feed. This is saved to flash, so it survives reboots.

## How it behaves

- **Locked by default.** The dashboard opens to a "Locked" screen. Touching the sensor unlocks it.
- **Auto re-lock.** Once unlocked, it automatically re-locks after an idle timer (30s by default). A countdown ("Re-locking in Xs") shows on the dashboard, and a "Lock now" button lets you lock it immediately.
- **Alcohol level.** The MQ3 raw reading is classified as **Sober**, **Caution**, or **Drunk** based on two thresholds.
- **Camera feed.** Shown as its own card in the (unlocked) dashboard, streamed live via an `<img>` pointed straight at the ESP32-CAM — the sensor board doesn't proxy the video, so both boards must be reachable from whatever device is viewing the dashboard.
- **Settings panel** (⚙ button, only visible while unlocked) lets you edit and save:
  - *Sober max* — readings below this are "Sober"
  - *Caution max* — readings below this (and above Sober max) are "Caution"; above it is "Drunk"
  - *Auto re-lock seconds*
  - *Camera stream URL*

  Saved settings are written to the sensor board's flash (NVS via `Preferences`), so they **persist across reboots and power loss** — no need to re-enter them after unplugging the board.

## Notes

- The gas reading is the raw ADC value (0-4095) and a derived percentage. MQ3 needs a warm-up period (a minute or so) before readings stabilize. The default thresholds (Sober < 1200, Caution < 2500, Drunk ≥ 2500) are starting points — calibrate them against your own sensor in clean air vs. near alcohol, then save the tuned values in Settings.
- The sensor board needs only `WiFi.h`, `WebServer.h`, and `Preferences.h` (bundled with the ESP32 Arduino core). The camera board needs `esp_camera.h` and `esp_http_server.h`, also bundled with the core — no third-party libraries either.
- If the ESP32-CAM's IP changes (e.g. router reassigns DHCP leases), the camera card will stop loading — reserve a static/DHCP-reserved IP for it on your router, or re-check Serial and update the URL in Settings.
- The camera stream server only handles one connected viewer at a time by design (simple, single-purpose sketch) — opening the dashboard in multiple browser tabs simultaneously means only one will show live video.
