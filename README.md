# 🤖 Intelligent Vision-Based Robotic Manipulator

An advanced ESP32-based robotic manipulator designed for **landmine detection, inspection, and safe object handling**. The system combines **computer vision, live video streaming, robotic arm control, GPS waypoint navigation, metal detection, and wireless operation** into a single intelligent robotic platform.

The robot can operate in both **Manual Mode** and **Automatic Mode**, allowing users to remotely monitor, navigate, detect metallic objects, and manipulate hazardous items using a multi-DOF robotic arm.

---

# 📌 Project Overview

The Intelligent Vision-Based Robotic Manipulator is a mobile robotic platform that provides:

- 🎥 Live video streaming using ESP32-CAM
- 🤖 Multi-DOF robotic manipulator control
- 📍 GPS waypoint navigation
- 🧭 Compass-based directional guidance
- 🔍 Metal detection system
- 📶 Wi-Fi hotspot-based remote control
- 🎮 Manual and Automatic operation modes
- ⚡ Real-time communication between two ESP32 boards
- 🔄 FreeRTOS multitasking for smooth performance

The system is intended for research, educational robotics, surveillance, hazardous object handling, and landmine detection applications.

---

# 🏗️ System Architecture

## ESP32-CAM (Vision & Web Server Unit)

Responsible for:

- Hosting Wi-Fi Hotspot
- Web Dashboard
- Live Video Streaming
- User Interface
- Sending Commands to Main Controller

### Features

- ESP32 Access Point Mode
- MJPEG Live Video Streaming
- Manual Control Interface
- Automatic Navigation Interface
- GPS Waypoint Entry
- Servo Control Sliders
- Metal Detection Commands

---

## ESP32 Main Controller (Robot Control Unit)

Responsible for:

- Motor Control
- Robotic Arm Control
- GPS Processing
- Compass Navigation
- Metal Detection
- Autonomous Operations
- FreeRTOS Task Management

### Features

- Multi-threaded FreeRTOS Design
- Servo Control using PCA9685
- GPS Tracking
- Compass Heading Detection
- Motor Driver Control
- Metal Detection Logic
- Object Handling Operations

---

# ⚙️ Main Features

## 🎥 Live Video Streaming

The ESP32-CAM streams real-time video to the web dashboard.

- Browser-based viewing
- No mobile app required
- Real-time monitoring
- Low latency video feed

---

## 🎮 Manual Mode

Allows the operator to control the robot remotely.

### Vehicle Control

- Forward
- Backward
- Left
- Right
- Stop

### Manipulator Control

Individual control of:

- Base Servo
- Shoulder Servo
- Elbow Servo
- Wrist Servo
- Gripper Servo
- Auxiliary Servo

All servos are controlled using slider-based controls.

---

## 🤖 Automatic Mode

In automatic mode the robot can:

- Follow predefined GPS points
- Perform autonomous navigation
- Detect metallic objects
- Execute scanning operations
- Stop when detection occurs

---

## 📍 GPS Navigation

The system supports waypoint-based navigation.

### Features

- Store GPS coordinates
- Multiple waypoint support
- Autonomous path execution
- Real-time GPS monitoring

Waypoint data is transmitted from the dashboard to the robot through UART communication.

---

## 🧭 Digital Compass Navigation

The robot uses the QMC5883L compass module for:

- Heading estimation
- Direction correction
- Autonomous navigation
- Orientation tracking

---

## 🔍 Metal Detection System

Integrated metal detector for:

- Landmine detection
- Metallic object detection
- Hazard identification

### Functions

- Manual Scan
- Automatic Scan
- Detection Lock Mechanism
- Safety Stop

---

## 🦾 Robotic Manipulator

The robotic arm performs:

- Pick and Place Operations
- Object Retrieval
- Hazardous Object Handling
- Manipulation Tasks

### Arm Functions

- Arm Extension
- Arm Retraction
- Gripper Open
- Gripper Close
- Position Adjustment

---

# 🔄 Communication System

The project uses UART communication between two ESP32 boards.

## ESP32-CAM Commands

| Command | Function |
|----------|-----------|
| M | Manual Mode |
| A | Automatic Mode |
| F | Forward |
| B | Backward |
| L | Left |
| R | Right |
| S | Stop |
| Y | Auto Start |
| Z | Auto Stop |
| X | Auto Reset |
| T | Test Signal |
| SCAN | Start Metal Scan |
| Sx:pos | Move Servo |
| Gx:lat,lon | Save GPS Point |

---

# 🛠 Hardware Components

## Controllers

- ESP32-CAM
- ESP32 Development Board

## Sensors

- GPS Module
- QMC5883L Compass
- Metal Detector Sensor

## Actuators

- DC Motors
- Multi-DOF Servo Arm
- Gripper Servo

## Drivers

- PCA9685 Servo Driver
- Motor Driver Module

## Communication

- Wi-Fi
- UART Serial Communication

---

# 💻 Software & Libraries

### ESP32-CAM Side

- WiFi.h
- WebServer.h
- esp_camera.h
- esp_http_server.h

### Main Controller Side

- Wire.h
- Adafruit_PWMServoDriver.h
- TinyGPS++.h
- QMC5883LCompass.h
- ESP32Servo.h
- FreeRTOS

---

# 📡 Web Dashboard

The dashboard includes:

### Home Menu

- Automatic Mode
- Manual Mode

### Automatic Mode

- Start Mission
- Stop Mission
- System Test
- GPS Waypoint Configuration
- Live Camera Feed

### Manual Mode

- Vehicle Control Panel
- Robotic Arm Control Sliders
- Metal Scan Button
- Live Camera Feed

---

# 🔐 Wi-Fi Connection

Default Access Point:

```text
SSID     : ESP32_Robot
Password : 12345678
```

Connect to the hotspot and open:

```text
http://192.168.4.1
```

to access the dashboard.

---

# 🚀 Applications

- Landmine Detection
- Hazardous Environment Inspection
- Search and Rescue Robotics
- Surveillance Systems
- Military Robotics Research
- Educational Robotics
- Autonomous Navigation Research
- Remote Object Handling

---

# 🎯 Future Improvements

- AI-Based Object Detection
- Computer Vision Tracking
- Autonomous Path Planning
- Machine Learning Integration
- Cloud Monitoring
- Mobile Application Support
- Obstacle Avoidance
- ROS Integration

---

# 👨‍💻 Developer

**Muhammad Zulqarnain**

Robotics Engineer | IoT Developer | ESP32 Enthusiast

Areas of Interest:

- Robotics
- Embedded Systems
- ESP32 Development
- IoT Solutions
- Computer Vision
- Artificial Intelligence
- Autonomous Systems

---

# 📜 License

This project is developed for educational, research, and innovation purposes.

Feel free to modify, improve, and build upon this work while providing proper credit to the original developer.

---

⭐ If you found this project useful, don't forget to star the repository.
