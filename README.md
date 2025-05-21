# 🚀 NSCC_Model_Rocket

An embedded avionics and launch control system for a 3D-printed model rocket — designed and built by Nova Scotia Community College (NSCC) students. This project combines embedded programming, wireless communication, and aerospace telemetry to create a fully functional **flight computer** and **remote launch controller**.

---

## 📦 Hardware Overview

### 🚁 Flight Computer (Rocket Payload)
Powered by the **Arduino MKR GSM 1400**, the flight computer includes:

- **Cellular Communication**:
  - `Arduino MKR GSM 1400`: Sends GPS coordinates via cellular data for real-time tracking.

- **Positioning**:
  - `Arduino MKR GPS Shield`: Provides live GPS location and altitude data.

- **Sensor Suite & Datalogging**:
  - `Arduino MKR ENV Shield`: Records environmental data (temperature, humidity, pressure) and logs telemetry to SD card.
  - `MPU6050`: Captures 3-axis accelerometer and gyroscope data for orientation, thrust profiling, and flight dynamics.

- **Optional Payload**:
  - Mini GoPro Hero 4 for onboard video capture.

### 🧨 Launch Controller (Ground Station)
- Launch button with arming safety key
- Armed status indicator (LED)
- Wireless control to safely initiate launch from a distance

---

## ⚙️ Features
- Real-time celluar data GPS tracking
- Environmental and inertial telemetry logging
- Max altitude, velocity, acceleration (g-force) computation
- Modular embedded system design
- Safe and remote launch controller

---

## 📊 Data Captured
- Latitude / Longitude / Altitude
- Velocity and Acceleration (calculated from GPS and IMU)
- Environmental readings (Temperature, Pressure, Humidity)
- Time-stamped logs written to SD card

---

## 🌿 Stretch Goals
- App-based real-time tracking interface
- Autonomous glider dropped from weather balloon
- Waypoint navigation and return-to-launch functionality

---

## 🎥 Media & Documentation
- 📽️ Test launch footage  
- 🛠️ Rocket build time-lapse  
- 📈 Telemetry graphs and post-flight analysis

> This project serves as both a practical embedded systems challenge and an exciting aerospace exploration. Designed to be a learning showcase for future NSCC students.

---

## 📚 References
- [Matthew Malham – 3D Printed Rockets](https://example.com)
- [Rocketry Forum – Avionics Bay Threads](https://www.rocketryforum.com)
- [Overkill Launch Controller – Instructables](https://www.instructables.com)
- [Spaceport America Cup – YouTube](https://www.youtube.com)

---

## 🧑‍💻 Authors  
**Nova Scotia Community College (NSCC) Students**

**Aaron Thomas**  
- *Information Technology Programming* (Sept 2019 – Apr 2021)  
- *Internet of Things (IoT) Programming* (Sept 2024 – May 2025)  
- [LinkedIn](https://www.linkedin.com/in/aaron-thomas-software-developer/)  
- [GitHub](https://github.com/KeanuRevan)

---

## 📅 Project Duration
April 2025 – May 2025 (2 months)

---

## 🛡️ License
[MIT License](LICENSE)
