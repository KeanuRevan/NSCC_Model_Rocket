#include <Arduino_MKRENV.h>
#include <Arduino_MKRGPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#define SD_CS_PIN 4  // Your SD card chip select pin
File dataFile;

// Sensor objects
Adafruit_MPU6050 mpu;

unsigned long launchTime = 0;
bool launched = false;
bool inFlight = false;

// Flight Log vaules
float maxAltitude = 0;
float peakAcceleration = 0;
float peakVelocity = 0;
float currentVelocity = 0;

String gpsFileName = "gpslog.csv";
String envFileName = "envlog.csv";
String flightLogFileName = "flightlog.csv";

// Timing intervals in milliseconds
const unsigned long envInterval = 1000;
const unsigned long gpsInterval = 1000;
const unsigned long mpuInterval = 100;

unsigned long lastEnvMillis = 0;
unsigned long lastGPSMillis = 0;
unsigned long lastMPUMillis = 0;
unsigned long previousMillis = 0;

// ---------- UTILITIES ----------
void checkInit(bool status, const char* successMsg, const char* failMsg) {
  if (!status) {
    Serial.println(failMsg);
    while (1);
  }
  Serial.println(successMsg);
}

String createCSV(const float values[], size_t count, int decimalPlaces = 2) {
  String csv = "";
  for (size_t i = 0; i < count; i++) {
    csv += String(values[i], decimalPlaces);
    if (i < count - 1) csv += ",";
  }
  return csv;
}

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

void ensureCSVHeader(const char* filename, const char* header) {
  if (!SD.exists(filename)) {
    logToSD(filename, header);
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(9600);
  while (!Serial);

  checkInit(ENV.begin(), "MKR ENV Shield initialized.", "Failed to initialize MKR ENV Shield!");
  checkInit(GPS.begin(GPS_MODE_SHIELD), "MKR GPS initialized.", "Failed to init GPS!");
  checkInit(SD.begin(SD_CS_PIN), "SD card initialized.", "SD init failed!");
  checkInit(mpu.begin(), "MPU6050 initialized.", "MPU6050 failed!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  ensureCSVHeader(envFileName.c_str(), "Temperature,Humidity,Pressure,Illuminance,UVA,UVB,UVIndex");
  ensureCSVHeader(gpsFileName.c_str(), "Latitude,Longitude,Altitude,Speed,Satellites");
  ensureCSVHeader(flightLogFileName.c_str(), "FlightTime(s),MaxAltitude(m),PeakAcceleration(g),PeakVelocity(m/s)");
}

// ---------- LOOP ----------
void loop() {
  // === ENV DATA ===
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

  // === GPS DATA ===
  if (GPS.available()) {
    float latitude = GPS.latitude();
    float longitude = GPS.longitude();
    float altitude = GPS.altitude(); // in meters
    float speed = GPS.speed();       // in km/h
    int satellites = GPS.satellites();

    float gpsValues[] = { latitude, longitude, altitude, speed, (float)satellites };
    String gpsCSV = createCSV(gpsValues, 5, 7);
    logToSD(gpsFileName, gpsCSV);

    // Update max altitude
    if (altitude > maxAltitude) maxAltitude = altitude;
  }

  // === ACCELERATION / FLIGHT TRACKING ===
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelZ = a.acceleration.z / 9.81;  // Convert to g-force
  unsigned long currentMillis = millis();
  float deltaTime = (currentMillis - previousMillis) / 1000.0;

  if (deltaTime > 0) {
    if (!inFlight && accelZ > 2.0) {
      launched = true;
      inFlight = true;
      launchTime = currentMillis;
      Serial.println("Launch detected!");
    }

    if (inFlight) {
      float netAccel = (accelZ - 1.0) * 9.81; // subtract gravity
      if (netAccel < 0) netAccel = 0;

      currentVelocity += netAccel * deltaTime;
      if (currentVelocity > peakVelocity) peakVelocity = currentVelocity;
      if (accelZ > peakAcceleration) peakAcceleration = accelZ;
    }

    // Detect landing: stable 1g and >5s since launch
    if (launched && inFlight && (currentMillis - launchTime > 5000) && abs(accelZ - 1.0) < 0.1) {
      inFlight = false;
      float flightTime = (currentMillis - launchTime) / 1000.0;

      float flightData[] = { flightTime, maxAltitude, peakAcceleration, peakVelocity };
      String flightCSV = createCSV(flightData, 4, 2);
      logToSD(flightLogFileName, flightCSV);

      Serial.println("Flight Complete:");
      Serial.println(flightCSV);
      while (1);  // Stop after logging
    }

    previousMillis = currentMillis;
  }

  delay(500);
}
