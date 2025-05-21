// --- Include required libraries ---
#include <Arduino_MKRENV.h>
#include <Arduino_MKRGPS.h>
#include <MKRGSM.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

// --- SD card configuration ---
#define SD_CS_PIN 4
File dataFile;
const char* rocketFolder = "rocket";

// --- GSM and GPRS settings ---
const char PINNUMBER[]     = "";
const char GPRS_APN[]      = "america.bics";
const char GPRS_LOGIN[]    = "";
const char GPRS_PASSWORD[] = "";

// --- GSM/GPRS clients ---
GSMClient client;
GPRS gprs;
GSM gsmAccess;

// --- Webhook configuration ---
const char* host = "webhook.site";
const char* path = "/353053be-6988-4ef9-96fd-ac0db6dd7a62";

// --- Sensor objects ---
Adafruit_MPU6050 mpu;

// --- Flight state tracking ---
unsigned long launchTime = 0;
bool launched = false;
bool inFlight = false;
float maxAltitude = 0;
float peakAcceleration = 0;
float peakVelocity = 0;
float currentVelocity = 0;

// --- Orientation baseline ---
float initialPitch = 0;
float initialRoll = 0;

// --- Log file names (with rocket folder) ---
String gpsFileName = String(rocketFolder) + "/gpslog.csv";
String envFileName = String(rocketFolder) + "/envlog.csv";
String flightLogFileName = String(rocketFolder) + "/flight.csv";
String motionLogFileName = String(rocketFolder) + "/motion.csv";

// --- Data collection intervals ---
const unsigned long envInterval = 1000;
const unsigned long gpsInterval = 1000;
const unsigned long webHookInterval = 60000;
const unsigned long mpuInterval = 100;

// --- Time tracking ---
unsigned long lastEnvMillis = 0;
unsigned long lastGPSMillis = 0;
unsigned long lastMPUMillis = 0;
unsigned long lastWebHookMillis = 0;
unsigned long previousMillis = 0;

// --- Utility Functions ---
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
  dataFile = SD.open(filename.c_str(), FILE_WRITE);
  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
  } else {
    Serial.print("Error opening ");
    Serial.println(filename);
  }
}

void ensureCSVHeader(const String& filename, const char* header) {
  if (SD.exists(filename.c_str())) {
    SD.remove(filename.c_str());
    Serial.print("Deleted existing ");
    Serial.println(filename);
  }
  logToSD(filename, header);
  Serial.print("Created ");
  Serial.println(filename);
}

void sendMessage(String message) {
  if (client.connect(host, 80)) {
    Serial.println("Connected to webhook.site");
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

  while (client.connected() || client.available()) {
    if (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
  }

  client.stop();
  Serial.println("Connection closed");
}

// --- SETUP ---
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  checkInit(ENV.begin(), "MKR ENV Shield initialized.", "Failed to initialize MKR ENV Shield!");
  checkInit(GPS.begin(GPS_MODE_SHIELD), "MKR GPS initialized.", "Failed to init GPS!");
  checkInit(SD.begin(SD_CS_PIN), "SD card initialized.", "SD init failed!");

  // Create rocket directory if it doesn't exist
  if (!SD.exists(rocketFolder)) {
    SD.mkdir(rocketFolder);
    Serial.println("Created 'rocket' folder on SD card.");
  }

  checkInit(mpu.begin(), "MPU6050 initialized.", "MPU6050 failed!");
  checkInit(gsmAccess.begin(PINNUMBER) == GSM_READY, "GSM connected", "GSM connection failed");
  checkInit(gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY, "GPRS connection failed", "GPRS connection failed");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  ensureCSVHeader(envFileName, "Temperature,Humidity,Pressure,Illuminance,UVA,UVB,UVIndex");
  ensureCSVHeader(gpsFileName, "Latitude,Longitude,Altitude,Speed,Satellites");
  ensureCSVHeader(flightLogFileName, "FlightTime(s),MaxAltitude(m),PeakAcceleration(g),PeakVelocity(m/s)");
  ensureCSVHeader(motionLogFileName, "AccelerationZ(g),Velocity(m/s),Pitch(degrees),Roll(degrees)");

  delay(1000); // Let MPU stabilize

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  initialPitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  initialRoll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  Serial.println("MPU orientation zeroed.");
  sendMessage("Hello from Setup");
}

// --- LOOP ---
void loop() {
  unsigned long currentMillis = millis();

  // --- Environmental Logging ---
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

  // --- GPS Logging and Webhook ---
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

    if (altitude > maxAltitude) maxAltitude = altitude;

    if (currentMillis - lastWebHookMillis >= webHookInterval) {
      lastWebHookMillis = currentMillis;
      sendMessage("https://www.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6));
    }
  }

  // --- MPU Logging, Flight Logic ---
  if (currentMillis - lastMPUMillis >= mpuInterval) {
    float deltaTime = (currentMillis - previousMillis) / 1000.0;
    lastMPUMillis = currentMillis;
    previousMillis = currentMillis;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float accelZ = a.acceleration.z / 9.81;

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

    float zeroedPitch = pitch - initialPitch;
    float zeroedRoll = roll - initialRoll;

    float motionData[] = { accelZ, currentVelocity, zeroedPitch, zeroedRoll };
    logToSD(motionLogFileName, createCSV(motionData, 4, 2));

    if (deltaTime > 0) {
      if (!inFlight && accelZ > 2.0) {
        launched = true;
        inFlight = true;
        launchTime = currentMillis;
        Serial.println("Launch detected!");
      }

      if (inFlight) {
        float netAccel = (accelZ - 1.0) * 9.81;
        if (netAccel < 0) netAccel = 0;

        currentVelocity += netAccel * deltaTime;
        if (currentVelocity > peakVelocity) peakVelocity = currentVelocity;
        if (accelZ > peakAcceleration) peakAcceleration = accelZ;
      }

      if (launched && inFlight && (currentMillis - launchTime > 5000) && abs(accelZ - 1.0) < 0.1) {
        inFlight = false;
        float flightTime = (currentMillis - launchTime) / 1000.0;
        float flightData[] = { flightTime, maxAltitude, peakAcceleration, peakVelocity };
        logToSD(flightLogFileName, createCSV(flightData, 4));
        Serial.println("Flight complete.");
      }
    }
  }
}