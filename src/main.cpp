#include <Wire.h>
#include <math.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "DFRobot_BloodOxygen_S.h"
#include <TinyGPSPlus.h>
#include <time.h>
// ===== BACKEND INTEGRATION =====
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ===== BACKEND CONFIG =====
#define WIFI_SSID       "AVA-kerolos"
#define WIFI_PASSWORD   "ASDF12345##"

#define BASE_URL "https://wearable-project-c8ajetbpdyava4d5.spaincentral-01.azurewebsites.net"
#define API_KEY "c4f73d21b6a04baf9a3e9c7fd54fc1c4e5a2e91bbf734c2ea197c8fbd9d42ef2"
#define VITALS_POST_URL   BASE_URL "/vitals/update"
#define LOCATION_POST_URL BASE_URL "/patient/location/update"
#define CONTACTS_GET_URL  BASE_URL "/device/caregiver/contact/"
#define STEPS_POST_URL BASE_URL "/api/activity/weekly-steps"
#define DEVICE_STATUS_URL BASE_URL "/device/device/status"
#define FALL_ALERT_URL  BASE_URL "/test/fall-detection"
#define USER_ID 4  // patient_id / user_id 
int batteryPercent = 90; // مؤقت
unsigned long lastDevicePing = 0;
#define DEVICE_PING_INTERVAL 60000
// ===== BACKEND EMERGENCY CONTACTS =====
#define MAX_CONTACTS 3
String emergencyNumbers[MAX_CONTACTS];
int emergencyCount = 0;
// for refresh num/////////
unsigned long lastContactsFetch = 0;
#define CONTACTS_REFRESH_INTERVAL (6UL * 60UL * 60UL * 1000UL)  //

// ===== BACKEND SEND TIMER =====
unsigned long lastVitalsSend = 0;
#define VITALS_SEND_INTERVAL 10000   //ms
/////////time egypt////////////
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 2 * 3600;     //  UTC+2
const int   daylightOffset_sec = 0;

/* ================= PINS ================= */
#define GSR_PIN        39
#define VIBRATION_PIN  18
#define BUTTON_PIN     19

/* ================= SIM800 ================= */
#define SIM800_RX 16
#define SIM800_TX 17
#define SIM_BAUD  9600


/* ================= GPS ================= */
#define GPS_RX 4
#define GPS_TX 5
#define GPS_BAUD 9600
#define GPS_TIMEOUT 15000

TinyGPSPlus gps;

/* ================= MAX30102 ================= */
#define I2C_ADDRESS 0x57
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire, I2C_ADDRESS);

/* ================= OLED ================= */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* ================= SENSORS ================= */
Adafruit_MLX90614 mlx;
Adafruit_MPU6050 mpu;

/* ================= FALL ================= */
#define FREE_FALL_THRESHOLD  10.0
#define IMPACT_THRESHOLD     12.0
#define FREE_FALL_TIME       120

enum FallState {
  FREE_FALL,
  IMPACT,
  FALL_DETECTED,
  GET_GPS,
  SEND_SMS
};

FallState fallState = FREE_FALL;
unsigned long freeFallStart = 0;

/* ================= ALERT ================= */
#define ALERT_TIMEOUT 20000
bool alertActive = false;
unsigned long alertStartTime = 0;

/* ================= GPS DATA ================= */
unsigned long gpsStartTime = 0;
bool gpsFix = false;

double latitude = 0, longitude = 0;
double lastLatitude = 0, lastLongitude = 0;
bool hasLastLocation = false;

/* ================= GSR ================= */
int gsrBaseline = 0;
bool gsrBaselineReady = false;
#define GSR_WINDOW 10
int gsrBuffer[GSR_WINDOW];
int gsrIndex = 0;
bool gsrFilled = false;

// ===== STEPS =====
unsigned long lastStepTime = 0;
int stepCount = 0;
unsigned long lastStepsSend = 0;
#define STEPS_SEND_INTERVAL 60000   // 
#define STEP_THRESHOLD 0.9     //    (1.2 ~ 1.5)
#define STEP_DELAY     500     // ms (debounce)
#define MAG_WINDOW 5
float magBuffer[MAG_WINDOW];
int magIndex = 0;
bool magFilled = false;

float getFilteredMag(float newVal) {
  magBuffer[magIndex++] = newVal;
  if (magIndex >= MAG_WINDOW) {
    magIndex = 0;
    magFilled = true;
  }

  float sum = 0;
  int count = magFilled ? MAG_WINDOW : magIndex;
  for (int i = 0; i < count; i++) sum += magBuffer[i];
  return sum / count;
}
/* ================= HEART ICON ================= */
void drawHeart(int x, int y, bool pulse) {
  int r = pulse ? 5 : 4;
  display.fillCircle(x, y, r, SSD1306_WHITE);
  display.fillCircle(x + r * 2, y, r, SSD1306_WHITE);
  display.fillTriangle(x - r, y, x + r * 3, y, x + r, y + r * 3, SSD1306_WHITE);
}

/* ================= STRESS ================= */
int getFilteredGSR(int newValue) {
  gsrBuffer[gsrIndex++] = newValue;
  if (gsrIndex >= GSR_WINDOW) {
    gsrIndex = 0;
    gsrFilled = true;
  }

  int sum = 0;
  int count = gsrFilled ? GSR_WINDOW : gsrIndex;
  for (int i = 0; i < count; i++) {
    sum += gsrBuffer[i];
  }
  return sum / count;
}
void updateGSRBaseline(int filteredGSR) {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 5000) return; //   

  gsrBaseline = (gsrBaseline * 9 + filteredGSR) / 10;
  lastUpdate = millis();
}
String getStressLevel(int rawGSR) {

  int filteredGSR = getFilteredGSR(rawGSR);

  if (!gsrBaselineReady) return "normal";

  updateGSRBaseline(filteredGSR);

  int diff = filteredGSR - gsrBaseline;

  if (diff > 300) return "high";
  if (diff < -300) return "low";

  return "normal";
}

/* ================= MAIN PAGE ================= */
void drawMainPage(int hr, int spo2, float temp, String stress) {
  static bool pulse = false;
  static unsigned long last = 0;

  if (millis() - last > 600) {
    pulse = !pulse;
    last = millis();
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  drawHeart(30, 6, pulse);

  display.setTextSize(2);
  display.setCursor(52, 0);
  display.print(hr);

  display.setTextSize(1);
  display.setCursor(0, 32);
  display.print("SpO2: "); display.print(spo2); display.print("%");

  display.setCursor(64, 32);
  display.print("T: "); display.print(temp, 1); display.print("C");

  display.setCursor(0, 48);
  display.print("Stress: "); display.print(stress);

  display.display();
}

/* ================= ALERT PAGE ================= */
void drawAlertPage(unsigned long remaining) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(32, 0);
  display.print("ALERT");

  display.drawTriangle(64, 18, 50, 42, 78, 42, SSD1306_WHITE);
  display.drawLine(64, 24, 64, 34, SSD1306_WHITE);
  display.fillCircle(64, 38, 2, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(18, 56);
  display.print("Cancel in ");
  display.print(remaining / 1000);
  display.print(" s");

  display.display();
}

/* ================= GPS PAGE ================= */
void drawGPSPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 0);
  display.print("EMERGENCY");
  display.setTextSize(1);
  display.setCursor(20, 30);
  display.print("Getting GPS Fix");
  display.display();
}


/* ================= SIM800 INIT ================= */
void initSIM800() {
  delay(5000);
  Serial2.println("AT"); delay(1000);
  Serial2.println("ATE0"); delay(1000);
  Serial2.println("AT+CFUN=1"); delay(2000);
  Serial2.println("AT+CSCS=\"GSM\""); delay(1000);
  Serial2.println("AT+CMGF=1"); delay(1000);
  Serial2.println("AT+CNMI=1,2,0,0,0"); delay(1000);
}

/* ================= SEND SMS ================= */
bool sendSMS(const String& msg, const char* number) {


  // 
  while (Serial2.available()) Serial2.read();

  Serial2.print("AT+CMGS=\"");
  Serial2.print(number);
  Serial2.println("\"");

  //  
  unsigned long t = millis();
  bool prompt = false;
  while (millis() - t < 5000) {
    if (Serial2.available()) {
      char c = Serial2.read();
      if (c == '>') {
        prompt = true;
        break;
      }
    }
  }

  if (!prompt) {
    Serial.println("SMS prompt not received");
    return false;
  }

  //  
  Serial2.print(msg);
  Serial2.write(26);   // CTRL+Z

  //  
  t = millis();
  while (millis() - t < 15000) {
    if (Serial2.available()) {
      String r = Serial2.readString();
      if (r.indexOf("+CMGS") != -1) {
        Serial.println("SMS SENT OK");
        delay(8000); // مهم جدًا لOrange
        return true;
      }
      if (r.indexOf("ERROR") != -1) {
        Serial.println("SMS ERROR");
        return false;
      }
    }
  }

  Serial.println("SMS TIMEOUT");
  return false;
}
///////////////////////////////////////////////////////////////
void sendDeviceStatus(const String& status, int batteryPercent) {

  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(DEVICE_STATUS_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  http.addHeader("X-API-KEY", API_KEY);

  StaticJsonDocument<128> doc;
  doc["user_id"] = USER_ID;
  doc["status"] = status;
  doc["battery"] = batteryPercent;

  String payload;
  serializeJson(doc, payload);

  Serial.println("---- SEND DEVICE STATUS ----");
  Serial.println(payload);

  int code = http.POST(payload);
  Serial.print("[DEVICE STATUS] HTTP Code: ");
  Serial.println(code);

  http.end();
}

// ===== FETCH EMERGENCY CONTACTS FROM BACKEND =====
bool fetchEmergencyNumbers() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[FETCH] WiFi not connected");
    return false;
  }

  HTTPClient http;
  String url = String(CONTACTS_GET_URL) + String(USER_ID);

  http.begin(url);
  http.addHeader("Accept", "application/json");
  http.addHeader("X-API-KEY", API_KEY);

  int code = http.GET();
  Serial.print("[FETCH] HTTP Code: ");
  Serial.println(code);

  if (code != 200) {
    http.end();
    return false;
  }

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, http.getString());
  http.end();

  if (err) {
    Serial.print("[FETCH] JSON Error: ");
    Serial.println(err.c_str());
    return false;
  }

  emergencyCount = 0;

  JsonArray arr = doc.as<JsonArray>();
  for (JsonObject obj : arr) {
    if (emergencyCount >= MAX_CONTACTS) break;

    String number = obj["phone_number"].as<String>();
    if (number.length() > 5) {
      emergencyNumbers[emergencyCount++] = number;

      Serial.print("[FETCH] Caregiver ");
      Serial.print(emergencyCount);
      Serial.print(" = ");
      Serial.println(number);
    }
  }

  Serial.print("[FETCH] Total numbers = ");
  Serial.println(emergencyCount);

  return emergencyCount > 0;
}

//      steeps//
float lastMag = 0;

void detectStep(float mag) {
  unsigned long now = millis();

  //  
  if (mag > STEP_THRESHOLD &&
      lastMag <= STEP_THRESHOLD &&
      (now - lastStepTime) > STEP_DELAY) {

    stepCount++;
    lastStepTime = now;

    Serial.print("[STEP] Count = ");
    Serial.println(stepCount);
  }

  lastMag = mag;
}
///////////////// day/////////////
String getCurrentWeekDay() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "sun"; // fallback
  }

  const char* days[] = {
    "sun", "mon", "tue", "wed", "thu", "fri", "sat"
  };

  return String(days[timeinfo.tm_wday]);
}
/////////////preparesim///////////////
void prepareSIM800ForSMS() {
  Serial.println("[SIM800] Preparing module for SMS");

  while (Serial2.available()) Serial2.read();

  Serial2.println("AT");
  delay(500);

  Serial2.println("AT+CFUN=1");   // 
  delay(1500);

  Serial2.println("AT+CSCS=\"GSM\""); // Character set
  delay(500);

  Serial2.println("AT+CMGF=1");   // Text mode
  delay(500);

  Serial2.println("AT+CREG?");    // Network registration (debug)
  delay(500);

  Serial2.println("AT+CSQ");      // Signal quality (debug)
  delay(500);
}
/* ================= EMERGENCY SMS ================= */
void sendEmergencySMS(int hr, int spo2, float temp, String stress) {
  prepareSIM800ForSMS();
  String msg = "FALL DETECTED!\n";
  msg += "Patient needs help.\n\n";
  msg += "HR:" + String(hr) + "\n";
  msg += "SpO2:" + String(spo2) + "\n";
  msg += "Temp:" + String(temp, 1) + "C\n";
  msg += "Stress:" + stress + "\n\n";

  if (gpsFix) {
    msg += "Location:\n";
    msg += "Lat:" + String(latitude, 6) + "\n";
    msg += "Lon:" + String(longitude, 6) + "\n";
    msg += "Search in map: " + String(latitude, 6) + "," + String(longitude, 6);
  } else if (hasLastLocation) {
    msg += "Last Location:\n";
    msg += String(lastLatitude, 6) + ",";
    msg += String(lastLongitude, 6);
  } else {
    msg += "Location: NA";
  }

 // ===== SEND SMS USING BACKEND CONTACTS =====
if (emergencyCount == 0) {
  fetchEmergencyNumbers();
}

for (int i = 0; i < emergencyCount; i++) {
  Serial.print("[SMS] Sending to ");
  Serial.println(emergencyNumbers[i]);
  sendSMS(msg, emergencyNumbers[i].c_str());
  delay(8000);  // SAME DELAY AS ORIGINAL
}

}

// ===== SEND VITALS TO BACKEND =====
void sendVitalsToBackend(int hr, int spo2, float temp, String stress) {



  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[VITALS] WiFi NOT connected");
    return;
  }

   HTTPClient http;
http.begin(VITALS_POST_URL);
http.addHeader("Content-Type", "application/json");
http.addHeader("Accept", "application/json");
http.addHeader("X-API-KEY", API_KEY);   // ⭐⭐⭐ 
  StaticJsonDocument<256> doc;
  doc["user_id"] = USER_ID;
  doc["hr"] = hr;
  doc["spo2"] = spo2;
  doc["temp"] = temp;
  doc["stress"] = stress;

  String payload;
  serializeJson(doc, payload);

  Serial.println("---- SEND VITALS ----");
  Serial.println(payload);

  int code = http.POST(payload);
  Serial.print("[VITALS] HTTP Code: ");
  Serial.println(code);

  http.end();
}

void updateGPS() {
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  if (gps.location.isUpdated()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();

    lastLatitude = latitude;
    lastLongitude = longitude;
    hasLastLocation = true;

    gpsFix = true;

    // ===== DEBUG =====
    Serial.print("[GPS] FIX OK  Lat=");
    Serial.print(latitude, 6);
    Serial.print(" Lon=");
    Serial.println(longitude, 6);
  }
}
// send location to backend//
void sendLocationToBackend() {



  if (!gpsFix || WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
http.begin(LOCATION_POST_URL);
http.addHeader("Content-Type", "application/json");
http.addHeader("Accept", "application/json");
http.addHeader("X-API-KEY", API_KEY);   // ⭐⭐⭐ 
  StaticJsonDocument<128> doc;
  doc["patient_id"] = USER_ID;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;

  String payload;
  serializeJson(doc, payload);

  Serial.println("---- SEND LOCATION ----");
  Serial.println(payload);

  int code = http.POST(payload);
  Serial.print("[LOCATION] HTTP Code: ");
  Serial.println(code);

  http.end();
}
      // send steps to back end///
void sendStepsToBackend(int steps) {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[STEPS] WiFi NOT connected");
    return;
  }

  HTTPClient http;
  http.begin(STEPS_POST_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  http.addHeader("X-API-KEY", API_KEY);

  StaticJsonDocument<256> doc;
  doc["user_id"] = USER_ID;
  doc["week"] = getCurrentWeekDay();
  doc["steps"] = steps;

  String payload;
  serializeJson(doc, payload);

  Serial.println("---- SEND STEPS ----");
  Serial.println(payload);

  int code = http.POST(payload);
  Serial.print("[STEPS] HTTP Code: ");
  Serial.println(code);

  http.end();
}
// send fall alert to backend
void sendFallAlertToBackend() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[FALL ALERT] WiFi not connected");
    return;
  }

  HTTPClient http;
  http.begin(FALL_ALERT_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  http.addHeader("X-API-KEY", API_KEY);

  StaticJsonDocument<128> doc;
  doc["user_id"] = USER_ID;
  doc["alert_type"] = "fall_detection";

  String payload;
  serializeJson(doc, payload);

  Serial.println("---- SEND FALL ALERT ----");
  Serial.println(payload);

  int code = http.POST(payload);
  Serial.print("[FALL ALERT] HTTP Code: ");
  Serial.println(code);

  http.end();
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
delay(1000);
Serial.println("SYSTEM START");


  // ===== WIFI CONNECT =====
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

unsigned long wifiStart = millis();
while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 5000) {
  delay(200);
}

if (WiFi.status() == WL_CONNECTED) {
  Serial.println("[WIFI] Connected");
} else {
  Serial.println("[WIFI] Offline mode");
}
  fetchEmergencyNumbers();
  lastContactsFetch = millis();
  sendDeviceStatus("online", 90);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  Serial2.begin(SIM_BAUD, SERIAL_8N1, SIM800_RX, SIM800_TX);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  mlx.begin();
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  MAX30102.begin();
  MAX30102.sensorStartCollect();

  pinMode(GSR_PIN, INPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(GSR_PIN);
    delay(20);
  }
  gsrBaseline = sum / 50;
  gsrBaselineReady = true;

  initSIM800();
}

bool isValidVitals(int hr, int spo2) {
  if (hr <= 0 || spo2 <= 0) return false;
  if (hr < 40 || hr > 200) return false;
  if (spo2 < 70 || spo2 > 100) return false;
  return true;
}

/* ================= LOOP ================= */
void loop() {
  
  updateGPS();   // 

  static int hr, spo2, gsr;
  static float temp;
  static String stress;

  MAX30102.getHeartbeatSPO2();
  hr = MAX30102._sHeartbeatSPO2.Heartbeat;
  spo2 = MAX30102._sHeartbeatSPO2.SPO2;
  temp = mlx.readObjectTempC();
  int rawGSR = analogRead(GSR_PIN);
  stress = getStressLevel(rawGSR);

  // ===== PERIODIC VITALS SEND =====
if (millis() - lastVitalsSend > VITALS_SEND_INTERVAL) {

  if (isValidVitals(hr, spo2)) {
    sendVitalsToBackend(hr, spo2, temp, stress);
    sendLocationToBackend();
  } else {
    Serial.println("[VITALS] Invalid reading (HR/SPO2 = -1), skipped");
  }

  lastVitalsSend = millis();
}


  if (alertActive) {
    unsigned long elapsed = millis() - alertStartTime;
    drawAlertPage(ALERT_TIMEOUT - elapsed);

   static unsigned long lastBtn = 0;
if (digitalRead(BUTTON_PIN) == LOW && millis() - lastBtn > 200){
      lastBtn = millis();
      alertActive = false;
      digitalWrite(VIBRATION_PIN, LOW);
      fallState = FREE_FALL;
      delay(300);
    }
    else if (elapsed >= ALERT_TIMEOUT) {
      alertActive = false;
      digitalWrite(VIBRATION_PIN, LOW);
      fallState = GET_GPS;
      gpsStartTime = millis();
    }
    return;
  }

if (fallState == GET_GPS) {
  drawGPSPage();

  if (gpsFix) {
  Serial.println("[GPS] Live fix used");
  fallState = SEND_SMS;
}
else if (hasLastLocation && millis() - gpsStartTime > 3000) {
  Serial.println("[GPS] Using last known location");
  fallState = SEND_SMS;
}
else if (millis() - gpsStartTime > GPS_TIMEOUT) {
  Serial.println("[GPS] No location available");
  fallState = SEND_SMS;
}
  return;
}

  if (fallState == SEND_SMS) {

    // ===== SEND FALL EVENT TO BACKEND =====
    sendEmergencySMS(hr, spo2, temp, stress);
    delay(3000);
    sendFallAlertToBackend();
    delay(2000);
    fallState = FREE_FALL;
    return;
  }

  sensors_event_t acc, gyro, t;
  mpu.getEvent(&acc, &gyro, &t);

  float mag = sqrt(
  acc.acceleration.x * acc.acceleration.x +
  acc.acceleration.y * acc.acceleration.y +
  acc.acceleration.z * acc.acceleration.z
);

float magWithoutGravity = abs(mag - 9.8);
float filteredMag = getFilteredMag(magWithoutGravity);
detectStep(filteredMag);

if (millis() - lastStepsSend > STEPS_SEND_INTERVAL) {
  sendStepsToBackend(stepCount);
  lastStepsSend = millis();
}

if (millis() - lastDevicePing > DEVICE_PING_INTERVAL) {
  sendDeviceStatus("online", 90);
  lastDevicePing = millis();
}

if (fallState == FREE_FALL && mag < FREE_FALL_THRESHOLD) {
    freeFallStart = millis();
    fallState = IMPACT;
  }
  else if (fallState == IMPACT &&
           mag > IMPACT_THRESHOLD &&
           millis() - freeFallStart > FREE_FALL_TIME) {
    fallState = FALL_DETECTED;
  }
  else if (fallState == FALL_DETECTED) {
  alertActive = true;
  alertStartTime = millis();
  digitalWrite(VIBRATION_PIN, HIGH);
  drawAlertPage(ALERT_TIMEOUT);
}

if (millis() - lastContactsFetch > CONTACTS_REFRESH_INTERVAL) {
  Serial.println("[FETCH] Refreshing emergency contacts...");
  fetchEmergencyNumbers();
  lastContactsFetch = millis();
}

  drawMainPage(hr, spo2, temp, stress);
  delay(200);

}
