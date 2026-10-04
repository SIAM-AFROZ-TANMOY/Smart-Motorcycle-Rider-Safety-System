# 🏍️ AI-Based Smart Motorcycle Rider Safety, Monitoring, Accident Detection & Anti-Theft System

An integrated **smart motorcycle safety and monitoring system** built using **ESP32, ESP32-CAM, multiple sensors, GPS, GSM, fingerprint authentication, and alert modules**.

The system combines rider authentication, helmet detection, alcohol detection, speed monitoring, automatic lighting, accident detection, emergency location sharing, drowsiness detection, voice alerts, and anti-theft protection into one embedded prototype.

---

## 🚀 Project Overview

Motorcycle riders face several safety risks such as riding without a helmet, alcohol consumption, excessive speed, rider fatigue, accidents, and unauthorized vehicle access.

To address these problems, we developed an integrated **Smart Motorcycle Rider Safety and Monitoring System** around an **ESP32-based controller**.

The system continuously monitors important rider and vehicle conditions and responds automatically when an unsafe situation is detected.

### 🔹 Main Features

- 🪖 AI-Based Helmet Detection
- 👥 Rider / Multi-Rider Detection
- 🍺 Alcohol Detection & Ignition Interlock
- 🔐 Fingerprint-Based Rider Authentication
- 🚨 Anti-Theft Detection & GSM Alert
- 🏍️ Real-Time Speed Monitoring
- 💡 Automatic Low-Light Headlight Control
- 💥 Accident Detection
- 📍 GPS-Based Accident Location Tracking
- 📱 Emergency SMS Notification
- 😴 Drowsiness Detection
- 🔊 Voice Safety Alerts
- 💾 Event Logging using MicroSD Card
- 🔔 Buzzer & LED Warning System

---

## 🧠 System Features

### 1. AI-Based Helmet and Rider Detection

The **ESP32-CAM with OV2640 camera** captures the rider's image and checks whether a helmet is being worn.

If a helmet is not detected, the system prevents the ignition relay from being activated.

The camera can also estimate whether more than one person is seated on the motorcycle.

---

### 2. Intelligent Rider Safety and Monitoring

The system monitors multiple safety conditions before and during riding.

#### 🍺 Alcohol Detection
The **MQ-3 alcohol sensor** checks for alcohol presence.

If the reading exceeds the configured threshold:

- Ignition remains disabled
- Buzzer warning is activated

#### 🏍️ Speed Monitoring
A **Hall Effect speed sensor** detects wheel rotation and allows the ESP32 to estimate the motorcycle's speed.

If the configured speed limit is exceeded, the system generates an audible warning.

#### 💡 Automatic Low-Light Detection
An **LDR sensor** monitors ambient light.

When the surrounding light becomes low, the system automatically increases the brightness of the connected LED/headlight.

#### 🔐 Rider Authentication
A fingerprint sensor verifies whether the rider is authorized.

Only registered fingerprints are allowed to enable the ignition.

---

### 3. Accident Detection and Emergency Response

Accident detection uses two different sensors:

- **MPU6050 6-Axis IMU**
- **SW-420 Vibration Sensor**

The MPU6050 monitors acceleration and orientation, while the SW-420 provides additional vibration/shock information.

When the system detects a probable accident:

1. Accident condition is identified
2. ESP32-CAM can capture an image
3. NEO-6M GPS obtains the location
4. SIM800L GSM sends an emergency SMS
5. The predefined emergency contact receives the location information

---

### 4. Drowsiness Detection and Voice Alert

The **ESP32-CAM** monitors the rider's eye state.

If prolonged eye closure or signs of drowsiness are detected, the system generates a voice warning through a speaker and amplifier.

This feature is designed to provide an immediate alert and help regain the rider's attention.

---

### 5. Anti-Theft Protection

The fingerprint authentication system also works as an anti-theft mechanism.

If an unauthorized fingerprint is repeatedly detected:

- 🔔 Buzzer is activated
- 📱 GSM module sends an SMS alert
- 🚫 Unauthorized access is prevented

---

## 🏗️ System Architecture

```text
                    ┌─────────────────────┐
                    │      ESP32-CAM      │
                    │  OV2640 Camera      │
                    │                     │
                    │ • Helmet Detection  │
                    │ • Rider Detection   │
                    │ • Drowsiness        │
                    └──────────┬──────────┘
                               │
                               ▼
┌────────────────────────────────────────────────────┐
│                    ESP32 DevKit                    │
│              Central Control Unit                  │
└────────────────────────────────────────────────────┘
       ▲          ▲          ▲          ▲
       │          │          │          │
    MQ-3       MPU6050     SW-420      LDR
  Alcohol      Motion     Vibration    Light
       │          │          │          │
       └──────────┴──────────┴──────────┘
                    │
                    ▼
             ┌──────────────┐
             │ ESP32 Logic  │
             └──────┬───────┘
                    │
       ┌────────────┼─────────────┐
       ▼            ▼             ▼
   Fingerprint   Hall Sensor    GPS/GSM
   Authentication Speed         Emergency
       │                         Response
       ▼
   Relay / Ignition
       │
       ├── Buzzer
       ├── Speaker
       ├── LEDs
       └── Headlight
