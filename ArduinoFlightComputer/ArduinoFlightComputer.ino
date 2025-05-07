// --- Include required libraries ---
#include <Arduino_MKRENV.h>        // Environmental sensor (temperature, humidity, pressure, etc.)
#include <Arduino_MKRGPS.h>        // GPS module support
#include <MKRGSM.h>                // GSM and GPRS functionality
#include <Adafruit_MPU6050.h>      // MPU6050 accelerometer and gyroscope
#include <Adafruit_Sensor.h>       // Sensor event handling
#include <Wire.h>                  // I2C communication
#include <SD.h>                    // SD card file system
#include <SPI.h>                   // SPI communication (used for SD)

// --- SD card configuration ---
#define SD_CS_PIN 4                // Chip Select pin for SD card module
File dataFile;                     // File object used for logging data

// --- GSM and GPRS settings ---
const char PINNUMBER[]     = ""; // SIM PIN code (if required)
const char GPRS_APN[]      = "america.bics"; // APN for cellular network (adjust to your provider)
const char GPRS_LOGIN[]    = ""; // Usually blank
const char GPRS_PASSWORD[] = ""; // Usually blank

// --- GSM/GPRS clients ---
GSMClient client;
GPRS gprs;
GSM gsmAccess;

// --- Webhook configuration ---
const char* host = "webhook.site";  // Host to send data to
const char* path = "/c3e7db99-2d82-4a8b-92b5-ba3dc5df0e9d";  // Unique webhook path

// --- Sensor objects ---
Adafruit_MPU6050 mpu;  // Accelerometer and gyroscope sensor

// --- Flight state tracking ---
unsigned long launchTime = 0;
bool launched = false;
bool inFlight = false;
float maxAltitude = 0;
float peakAcceleration = 0;
float peakVelocity = 0;
float currentVelocity = 0;

// --- Log file names ---
String gpsFileName = "gpslog.csv";
String envFileName = "envlog.csv";
String flightLogFileName = "flight.csv";

// --- Data collection intervals (in milliseconds) ---
const unsigned long envInterval = 1000;       // 1 second
const unsigned long gpsInterval = 1000;       // 1 second
const unsigned long webHookInterval = 60000;  // 1 minute
const unsigned long mpuInterval = 100;        // 100 ms

// --- Time tracking for intervals ---
unsigned long lastEnvMillis = 0;
unsigned long lastGPSMillis = 0;
unsigned long lastMPUMillis = 0;
unsigned long lastWebHookMillis = 0;
unsigned long previousMillis = 0;

// --- Utility: check initialization and halt on failure ---
void checkInit(bool status, const char* successMsg, const char* failMsg) {
  if (!status) {
    Serial.println(failMsg);
    while (1);  // Infinite loop if initialization fails
  }
  Serial.println(successMsg);
}

// --- Utility: Create comma-separated string from array of floats ---
String createCSV(const float values[], size_t count, int decimalPlaces = 2) {
  String csv = "";
  for (size_t i = 0; i < count; i++) {
    csv += String(values[i], decimalPlaces);
    if (i < count - 1) csv += ",";
  }
  return csv;
}

// --- Utility: Append a line of data to SD card file ---
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

// --- Utility: Ensure CSV file starts with a header (delete old if needed) ---
void ensureCSVHeader(const char* filename, const char* header) {
  if (SD.exists(filename)) {
    SD.remove(filename);  // Delete old file
    Serial.print("Deleted existing ");
    Serial.println(filename);
  }
  logToSD(filename, header);  // Write header
  Serial.print("Created ");
  Serial.println(filename);
}

// --- Utility: Send string message via HTTP POST to webhook.site ---
void sendMessage(String message) {
  if (client.connect(host, 80)) {
    Serial.println("Connected to webhook.site");

    // Construct HTTP POST request
    client.print("POST " + String(path) + " HTTP/1.1\r\n");
    client.print("Host: webhook.site\r\n");
    client.print("Content-Type: text/plain\r\n");
    client.print("Content-Length: " + String(message.length()) + "\r\n");
    client.print("Connection: close\r\n\r\n");
    client.print(message);

    Serial.println("Message sent to webhook.site");
  } else {
    Serial.println("Connection to webhook.site failed");
    return;
  }

  // Wait for response from server (optional)
  while (client.connected() || client.available()) {
    if (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
  }

  client.stop();
  Serial.println("Connection closed");
}

// --- SETUP: Runs once at power-up ---
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  // Initialize sensors and modules
  checkInit(ENV.begin(), "MKR ENV Shield initialized.", "Failed to initialize MKR ENV Shield!");
  checkInit(GPS.begin(GPS_MODE_SHIELD), "MKR GPS initialized.", "Failed to init GPS!");
  checkInit(SD.begin(SD_CS_PIN), "SD card initialized.", "SD init failed!");
  checkInit(mpu.begin(), "MPU6050 initialized.", "MPU6050 failed!");
  checkInit(gsmAccess.begin(PINNUMBER) == GSM_READY, "GSM connected", "GSM connection failed");
  checkInit(gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY, "GPRS connection failed", "GPRS connection failed");

  // Set ranges for accelerometer and gyroscope
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Create CSV log files with headers
  ensureCSVHeader(envFileName.c_str(), "Temperature,Humidity,Pressure,Illuminance,UVA,UVB,UVIndex");
  ensureCSVHeader(gpsFileName.c_str(), "Latitude,Longitude,Altitude,Speed,Satellites");
  ensureCSVHeader(flightLogFileName.c_str(), "FlightTime(s),MaxAltitude(m),PeakAcceleration(g),PeakVelocity(m/s)");

  // Send test message
  sendMessage("Hello from Setup");
}

// --- LOOP: Main program loop ---
void loop() {
  unsigned long currentMillis = millis();

  // --- Environmental Sensor Logging (every 1s) ---
  if (currentMillis - lastEnvMillis >= envInterval) {
    lastEnvMillis = currentMillis;

    float envValues[] = {
      ENV.readTemperature(),
      ENV.readHumidity(),
      ENV.readPressure(),
      ENV.readIlluminance(),
      ENV.readUVA(),
      ENV.readUVB(),
      ENV.readUVIndex()
    };

    String envCSV = createCSV(envValues, 7);
    logToSD(envFileName, envCSV);
  }

  // --- GPS Logging + webhook location update (every 1s + webhook every 1min) ---
  if (currentMillis - lastGPSMillis >= gpsInterval && GPS.available()) {
    lastGPSMillis = currentMillis;

    float latitude = GPS.latitude();
    float longitude = GPS.longitude();
    float altitude = GPS.altitude();
    float speed = GPS.speed();
    int satellites = GPS.satellites();

    float gpsValues[] = { latitude, longitude, altitude, speed, (float)satellites };
    String gpsCSV = createCSV(gpsValues, 5, 7);
    logToSD(gpsFileName, gpsCSV);

    // Update max altitude
    if (altitude > maxAltitude) maxAltitude = altitude;

    // Send location to webhook every minute
    if (currentMillis - lastWebHookMillis >= webHookInterval) {
      lastWebHookMillis = currentMillis;
      sendMessage("https://www.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6));
    }
  }

  // --- Flight detection and acceleration/velocity logging ---
  if (currentMillis - lastMPUMillis >= mpuInterval) {
    float deltaTime = (currentMillis - previousMillis) / 1000.0;  // Time in seconds since last sample
    lastMPUMillis = currentMillis;
    previousMillis = currentMillis;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);  // Read accelerometer and gyro

    float accelZ = a.acceleration.z / 9.81;  // Convert to Gs

    if (deltaTime > 0) {
      // --- Launch detection: if strong upward acceleration ---
      if (!inFlight && accelZ > 2.0) {
        launched = true;
        inFlight = true;
        launchTime = currentMillis;
        Serial.println("Launch detected!");
      }

      // --- Track flight: velocity, acceleration peaks ---
      if (inFlight) {
        float netAccel = (accelZ - 1.0) * 9.81;  // Subtract gravity
        if (netAccel < 0) netAccel = 0;

        currentVelocity += netAccel * deltaTime;
        if (currentVelocity > peakVelocity) peakVelocity = currentVelocity;
        if (accelZ > peakAcceleration) peakAcceleration = accelZ;
      }

      // --- Detect landing: nearly 1G and 5s after launch ---
      if (launched && inFlight && (currentMillis - launchTime > 5000) && abs(accelZ - 1.0) < 0.1) {
        inFlight = false;

        float flightTime = (currentMillis - launchTime) / 1000.0;
        float flightData[] = { flightTime, maxAltitude, peakAcceleration, peakVelocity };
        String flightCSV = createCSV(flightData, 4);
        logToSD(flightLogFileName, flightCSV);
        Serial.println("Flight complete.");
      }
    }
  }
}