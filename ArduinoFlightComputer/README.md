# Flight Computer Setup and Code Overview

This document provides instructions for setting up your Arduino-based flight computer and an explanation of what the code does.

## 🧰 Hardware Requirements

* Arduino MKR board (e.g., MKR GSM 1400)
* MKR ENV Shield
* MKR GPS Shield
* Adafruit MPU6050 (Accelerometer/Gyro)
* MKR GSM/GPRS module (SIM required)
* SD Card and Adapter

## 🪛 Setup Instructions

### 1. Install Required Libraries

Install the following libraries using the Arduino Library Manager:

* `Arduino_MKRENV`
* `Arduino_MKRGPS`
* `MKRGSM`
* `Adafruit_MPU6050`
* `Adafruit_Sensor`
* `SD`
* `SPI`

### 2. Connect Hardware

* Stack the MKR ENV and GPS shields onto the MKR board.
* Connect the Adafruit MPU6050 via I2C (SDA to SDA, SCL to SCL).
* Insert a micro SD card into the SD module.
* Insert an activated SIM card into the GSM shield.

![ArduinoFlightComputerWiringDiagram](https://github.com/user-attachments/assets/474d8983-8e12-4539-9121-fda423e5bfe7)

### 3. Configure GSM and Webhook Settings

Update these variables in the code:

```cpp
const char PINNUMBER[]     = ""; // SIM pin if needed
const char GPRS_APN[]      = "america.bics"; // Your APN
const char GPRS_LOGIN[]    = "";
const char GPRS_PASSWORD[] = "";
const char WEBHOOK_PATH[]  = "https://webhook.site/your-custom-path"; // Your Webhook URL
```

### 4. Flash the Code

Upload the provided Arduino sketch to the board using the Arduino IDE.

### 5. Monitor Output

Open the Serial Monitor at 9600 baud to observe setup status and flight telemetry.

## 📦 SD Card Logs

Logs will be saved in a folder named `rocket`:

* `gpslog.csv`: GPS data
* `envlog.csv`: Environmental readings
* `motion.csv`: Acceleration, velocity, pitch, roll
* `flight.csv`: Summary of flight metrics

---

## 🧠 Code Overview

### Initialization (setup)

Initializes all hardware components and prepares CSV headers for data logging:

* ENV Shield: Temperature, humidity, pressure, UV, light
* GPS: Latitude, longitude, altitude, speed, satellites
* MPU6050: Accelerometer/Gyroscope (orientation, acceleration)
* SD card: File setup
* GSM: Connects to GPRS network

### Loop Function

The loop executes several key tasks on timed intervals:

#### 1. Environmental Logging (every 1s)

Reads and logs:

* Temperature
* Humidity
* Pressure
* Illuminance
* UVA/UVB/UVIndex

#### 2. GPS Logging (every 1s)

Logs:

* Latitude & Longitude
* Altitude
* Speed
* Number of satellites

Also sends a Google Maps location to the configured webhook path every 60s.

#### 3. Motion Logging & Flight Tracking (every 100ms)

Reads acceleration and calculates:

* Orientation (pitch and roll)
* Z-axis acceleration
* Velocity (integrated from net acceleration)
* Peak values

Detects launch by acceleration spike and logs flight duration, peak velocity, acceleration, and altitude.

---

## 🔁 Flight Logic Summary

| Phase     | Trigger                     | Actions                                        |
| --------- | --------------------------- | ---------------------------------------------- |
| Idle      | Before launch               | Baseline orientation is recorded               |
| Launched  | Accel Z > 2g                | Starts timer, tracks flight data               |
| In-Flight | While in air                | Continues to log motion and environmental data |
| Landed    | After \~5s, Accel Z near 1g | Logs summary to SD, resets flight status       |

---

## 🛰️ Webhook Integration

* Sends location to the URL specified in `WEBHOOK_PATH` every 60 seconds
* Helps with live tracking during flight

---

## 📎 Notes

* Ensure SD card is formatted to FAT32
* If SD file exists, it is deleted and replaced with a new one during setup
* Orientation is zeroed at startup; keep rocket still during this phase

---

#### Happy Launching! 🚀
