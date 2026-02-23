/*************************************************
 * College Bus Tracking System
 * ESP32 + NEO-6M GPS + Firebase Realtime Database
 *************************************************/

#include <WiFi.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Firebase_ESP_Client.h>

/* ========== WIFI CONFIG ========== */
#define WIFI_SSID     "ESP32TEST"
#define WIFI_PASSWORD "123456789"

/* ========== FIREBASE CONFIG ========== */
#define API_KEY "AIzaSyD8B0-xOgfyRO5mkhWrqTnEOWzhm2snUbM"

#define DATABASE_URL \
"https://college-bus-tracker-d66e9-default-rtdb.asia-southeast1.firebasedatabase.app/"

/* Firebase Email/Password Auth */
#define USER_EMAIL    "esp32bus@gmail.com"
#define USER_PASSWORD "esp32bus123"

/* ========== BUS CONFIG ========== */
#define BUS_ID "BUS001"

/* ========== GPS CONFIG ========== */
#define GPS_RX_PIN 16   // ESP32 RX2  ← GPS TX
#define GPS_TX_PIN 17   // ESP32 TX2  → GPS RX
#define GPS_BAUD   9600

/* ========== OBJECTS ========== */
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

/* ========== TIMING ========== */
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 30000; // 30 seconds

/* ================================================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Start GPS UART
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  connectWiFi();
  setupFirebase();

  Serial.println("🚍 College Bus Tracking System Started");
  Serial.println("Waiting for GPS fix...");
}

/* ================================================= */
void loop() {

  // Read GPS data continuously
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  // Send data only when GPS fix is valid
  if (gps.location.isValid() && gps.satellites.value() >= 3) {
    if (millis() - lastSendTime >= SEND_INTERVAL) {
      lastSendTime = millis();
      sendToFirebase();
    }
  }
}

/* ================================================= */
void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - startAttempt < 15000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ WiFi Failed — Restarting ESP32");
    ESP.restart();
  }
}

/* ================================================= */
void setupFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("✅ Firebase Connected");
}

/* ================================================= */
void sendToFirebase() {

  FirebaseJson json;

  json.set("latitude", gps.location.lat());
  json.set("longitude", gps.location.lng());
  json.set("speed_kmph", gps.speed.kmph());
  json.set("satellites", gps.satellites.value());
  json.set("timestamp", (unsigned long)(millis() / 1000));

  String path = "/bus_tracking/" BUS_ID "/location";

  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("📡 GPS data sent to Firebase");
    printGPS();
  } else {
    Serial.print("❌ Firebase Error: ");
    Serial.println(fbdo.errorReason());
  }
}

/* ================================================= */
void printGPS() {
  Serial.println("══════════════════════════");
  Serial.print("Latitude   : ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("Longitude  : ");
  Serial.println(gps.location.lng(), 6);
  Serial.print("Speed (km/h): ");
  Serial.println(gps.speed.kmph());
  Serial.print("Satellites : ");
  Serial.println(gps.satellites.value());
  Serial.println("══════════════════════════");
}