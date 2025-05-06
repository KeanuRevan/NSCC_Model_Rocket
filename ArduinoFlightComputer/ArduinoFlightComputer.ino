// Include necessary libraries for sensors and peripherals
#include <Arduino_MKRENV.h>        // Environmental sensor library
#include <Arduino_MKRGPS.h>        // GPS module library
#include <MKRGSM.h>
#include <Adafruit_MPU6050.h>      // Accelerometer & gyroscope library
#include <Adafruit_Sensor.h>       // Unified sensor interface
#include <Wire.h>                  // I2C communication
#include <SD.h>                    // SD card file system
#include <SPI.h>                   // SPI communication for SD card

#define SD_CS_PIN 4                // SD card Chip Select pin
File dataFile;                     // File object for logging

const char PINNUMBER[]     = ""; // SIM PIN if any
const char GPRS_APN[]      = "america.bics"; // <-- Replace with your carrier's APN
const char GPRS_LOGIN[]    = "";
const char GPRS_PASSWORD[] = "";

GSMClient client;
GPRS gprs;
GSM gsmAccess;

// Your webhook host and path
const char* host = "webhook.site";
const char* path = "/c3e7db99-2d82-4a8b-92b5-ba3dc5df0e9d";

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
const unsigned long webHookInterval = 60000;  // POST data to WebHook every 1 minute(s) https://webhook.site/#!/view/c3e7db99-2d82-4a8b-92b5-ba3dc5df0e9d/40891b76-6c79-49f0-8bfa-c7a4ec6be504/1
const unsigned long mpuInterval = 100;   // Sample MPU every 100ms

// Time tracking for intervals
unsigned long lastEnvMillis = 0;
unsigned long lastGPSMillis = 0;
unsigned long lastMPUMillis = 0;
unsigned long lastWebHookMillis = 0;
unsigned long previousMillis = 0;

float dateTimeTestValue = 0.0; //TEST VALUE HERE

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

// POST to WebHook
void sendMessage(String message) {
  if (client.connect(host, 80)) {
    Serial.println("Connected to webhook.site");

    client.print("POST " + String(path) + " HTTP/1.1\r\n");
    client.print("Host: webhook.site\r\n");
    client.print("Content-Type: text/plain\r\n");
    client.print("Content-Length: " + String(message.length()) + "\r\n");
    client.print("Connection: close\r\n");
    client.print("\r\n");
    client.print(message);

    Serial.println("Message sent to webhook.site");
  } else {
    Serial.println("Connection to webhook.site failed");
    return;
  }

  // Read and print the server response
  while (client.connected() || client.available()) {
    if (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
  }

  client.stop();
  Serial.println("Connection closed");
}

// ---------- Setup Function ----------
void setup() {
  Serial.begin(9600);                // Start serial communication
  while (!Serial);                   // Wait for serial monitor

  Serial.println("Initializing...");

  // Initialize all sensors and SD card, halt if any fail
  checkInit(ENV.begin(), "MKR ENV Shield initialized.", "Failed to initialize MKR ENV Shield!");
  checkInit(GPS.begin(GPS_MODE_SHIELD), "MKR GPS initialized.", "Failed to init GPS!");
  checkInit(SD.begin(SD_CS_PIN), "SD card initialized.", "SD init failed!");
  checkInit(mpu.begin(), "MPU6050 initialized.", "MPU6050 failed!");
  // Connect cellular
  checkInit(gsmAccess.begin(PINNUMBER) == GSM_READY, "GSM connected", "GSM connection failed");
  checkInit(gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY, "GPRS connected", "GPRS connection failed");

  // Configure accelerometer and gyroscope range
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Ensure headers are added to CSV files if not present
  ensureCSVHeader(envFileName.c_str(), "DateTime,Temperature,Humidity,Pressure,Illuminance,UVA,UVB,UVIndex");
  ensureCSVHeader(gpsFileName.c_str(), "DateTime,Latitude,Longitude,Altitude,Speed,Satellites");
  ensureCSVHeader(flightLogFileName.c_str(), "DateTime,FlightTime(s),MaxAltitude(m),PeakAcceleration(g),PeakVelocity(m/s)");

  sendMessage("Hello from Setup");
}

// ---------- Main Loop ----------
void loop() {
  unsigned long currentMillis = millis();  // Get current time

  // === ENVIRONMENTAL SENSOR LOGGING ===
  if (currentMillis - lastEnvMillis >= envInterval) {
    lastEnvMillis = currentMillis;

    // Read environmental values from MKR ENV Shield
    float envValues[] = {
      dateTimeTestValue, //TEST VALUE HERE
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

    float gpsValues[] = { dateTimeTestValue, latitude, longitude, altitude, speed, (float)satellites }; //TEST VALUE HERE
    String gpsCSV = createCSV(gpsValues, 6, 7); // String gpsCSV = createCSV(gpsValues, 5, 7); //TEST VALUE HERE
    logToSD(gpsFileName, gpsCSV);

    // Track highest altitude reached
    if (altitude > maxAltitude) maxAltitude = altitude;

    // POST GPS data to WebHook
    if (currentMillis - lastWebHookMillis >= webHookInterval) {

    }
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
        float flightData[] = { dateTimeTestValue, flightTime, maxAltitude, peakAcceleration, peakVelocity }; //TEST VALUE HERE
        String flightCSV = createCSV(flightData, 5, 2); // String flightCSV = createCSV(flightData, 4, 2); //TEST VALUE HERE
        logToSD(flightLogFileName, flightCSV);  // Save final flight log

        Serial.println("Flight Complete:");
        Serial.println(flightCSV);

        while (1);  // Stop the loop after flight completes
      }

      previousMillis = currentMillis;  // Update time tracker
    }
  }
}
