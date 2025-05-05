// Include necessary libraries for sensors and peripherals
#include <Arduino_MKRENV.h>        // Environmental sensor library
#include <Arduino_MKRGPS.h>        // GPS module library
#include <Adafruit_MPU6050.h>      // Accelerometer & gyroscope library
#include <Adafruit_Sensor.h>       // Unified sensor interface
#include <Wire.h>                  // I2C communication
#include <SD.h>                    // SD card file system
#include <SPI.h>                   // SPI communication for SD card

#define SD_CS_PIN 4                // SD card Chip Select pin
File dataFile;                     // File object for logging

// Initialize the MPU6050 accelerometer object
Adafruit_MPU6050 mpu;

// State and timing variables
unsigned long launchTime = 0;
bool launched = false;
bool inFlight = false;

// Flight data tracking variables
float maxAltitude = 0;
float peakAcceleration = 0;
float peakVelocity = 0;
float currentVelocity = 0;

// Filenames for CSV logs
String gpsFileName = "gpslog.csv";
String envFileName = "envlog.csv";
String flightLogFileName = "flight.csv";

// Data logging intervals (in milliseconds)
const unsigned long envInterval = 1000;  // Log environment data every 1s
const unsigned long gpsInterval = 1000;  // Log GPS data every 1s
const unsigned long mpuInterval = 100;   // Sample MPU every 100ms

// Time tracking for intervals
unsigned long lastEnvMillis = 0;
unsigned long lastGPSMillis = 0;
unsigned long lastMPUMillis = 0;
unsigned long previousMillis = 0;

// ---------- Utility Functions ----------

// Checks if a component initialized correctly; stops program on failure
void checkInit(bool status, const char* successMsg, const char* failMsg) {
  if (!status) {
    Serial.println(failMsg);
    while (1);  // Halt execution if init fails
  }
  Serial.println(successMsg);
}

// Converts an array of float values to a CSV-formatted string
String createCSV(const float values[], size_t count, int decimalPlaces = 2) {
  String csv = "";
  for (size_t i = 0; i < count; i++) {
    csv += String(values[i], decimalPlaces);
    if (i < count - 1) csv += ",";
  }
  return csv;
}

// Logs data to a file on the SD card
void logToSD(const String& filename, const String& data) {
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
  } else {
    Serial.print("Error opening ");
    Serial.println(filename);
  }
}

// Writes CSV header if file doesn't already exist
void ensureCSVHeader(const char* filename, const char* header) {
  if (SD.exists(filename)) {
    SD.remove(filename);
    Serial.print("Deleted existing ");
    Serial.println(filename);
  }
  logToSD(filename, header);
  Serial.print("Created ");
  Serial.println(filename);
}

// ---------- Setup Function ----------
void setup() {
  Serial.begin(9600);                // Start serial communication
  while (!Serial);                   // Wait for serial monitor

  // Initialize all sensors and SD card, halt if any fail
  checkInit(ENV.begin(), "MKR ENV Shield initialized.", "Failed to initialize MKR ENV Shield!");
  checkInit(GPS.begin(GPS_MODE_SHIELD), "MKR GPS initialized.", "Failed to init GPS!");
  checkInit(SD.begin(SD_CS_PIN), "SD card initialized.", "SD init failed!");
  checkInit(mpu.begin(), "MPU6050 initialized.", "MPU6050 failed!");

  // Configure accelerometer and gyroscope range
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Ensure headers are added to CSV files if not present
  ensureCSVHeader(envFileName.c_str(), "Temperature,Humidity,Pressure,Illuminance,UVA,UVB,UVIndex");
  ensureCSVHeader(gpsFileName.c_str(), "Latitude,Longitude,Altitude,Speed,Satellites");
  ensureCSVHeader(flightLogFileName.c_str(), "FlightTime(s),MaxAltitude(m),PeakAcceleration(g),PeakVelocity(m/s)");
}

// ---------- Main Loop ----------
void loop() {
  unsigned long currentMillis = millis();  // Get current time

  // === ENVIRONMENTAL SENSOR LOGGING ===
  if (currentMillis - lastEnvMillis >= envInterval) {
    lastEnvMillis = currentMillis;

    // Read environmental values from MKR ENV Shield
    float envValues[] = {
      ENV.readTemperature(),
      ENV.readHumidity(),
      ENV.readPressure(),
      ENV.readIlluminance(),
      ENV.readUVA(),
      ENV.readUVB(),
      ENV.readUVIndex()
    };

    // Convert to CSV format and log to SD card
    String envCSV = createCSV(envValues, 7);
    logToSD(envFileName, envCSV);
  }

  // === GPS DATA LOGGING ===
  if (currentMillis - lastGPSMillis >= gpsInterval && GPS.available()) {
    lastGPSMillis = currentMillis;

    // Read GPS data
    float latitude = GPS.latitude();
    float longitude = GPS.longitude();
    float altitude = GPS.altitude();
    float speed = GPS.speed();
    int satellites = GPS.satellites();

    float gpsValues[] = { latitude, longitude, altitude, speed, (float)satellites };
    String gpsCSV = createCSV(gpsValues, 5, 7);
    logToSD(gpsFileName, gpsCSV);

    // Track highest altitude reached
    if (altitude > maxAltitude) maxAltitude = altitude;
  }

  // === MPU6050 ACCELERATION & FLIGHT LOGIC ===
  if (currentMillis - lastMPUMillis >= mpuInterval) {
    float deltaTime = (currentMillis - previousMillis) / 1000.0;  // Delta time in seconds
    lastMPUMillis = currentMillis;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Convert vertical acceleration to g-force
    float accelZ = a.acceleration.z / 9.81;

    if (deltaTime > 0) {
      // Detect launch: acceleration exceeds threshold
      if (!inFlight && accelZ > 2.0) {
        launched = true;
        inFlight = true;
        launchTime = currentMillis;
        Serial.println("Launch detected!");
      }

      if (inFlight) {
        // Compute net acceleration and integrate to estimate velocity
        float netAccel = (accelZ - 1.0) * 9.81;  // Subtract gravity
        if (netAccel < 0) netAccel = 0;

        currentVelocity += netAccel * deltaTime;
        if (currentVelocity > peakVelocity) peakVelocity = currentVelocity;
        if (accelZ > peakAcceleration) peakAcceleration = accelZ;
      }

      // Detect landing: after 5s and back to near 1g acceleration
      if (launched && inFlight && (currentMillis - launchTime > 5000) && abs(accelZ - 1.0) < 0.1) {
        inFlight = false;

        float flightTime = (currentMillis - launchTime) / 1000.0;  // Total flight duration
        float flightData[] = { flightTime, maxAltitude, peakAcceleration, peakVelocity };
        String flightCSV = createCSV(flightData, 4, 2);
        logToSD(flightLogFileName, flightCSV);  // Save final flight log

        Serial.println("Flight Complete:");
        Serial.println(flightCSV);

        while (1);  // Stop the loop after flight completes
      }

      previousMillis = currentMillis;  // Update time tracker
    }
  }
}
