/*
 *  ESP32 + GPS + Firebase Bus Tracking System
 *  Updates location every 30 seconds
 */

#include <WiFi.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Firebase_ESP_Client.h>

/* WIFI CONFIG */
#define WIFI_SSID     "ESP32TEST"
#define WIFI_PASSWORD "123456789"

/* FIREBASE CONFIG*/
#define API_KEY "AIzaSyD8B0-xOgfyRO5mkhWrqTnEOWzhm2snUbM"

#define DATABASE_URL \
"https://college-bus-tracker-d66e9-default-rtdb.asia-southeast1.firebasedatabase.app/"

/* Firebase Auth user (created in Firebase Console) */
#define USER_EMAIL    "esp32bus@gmail.com"
#define USER_PASSWORD "esp32bus123"

/*  BUS CONFIG  */
#define BUS_ID "BUS001"

/*  GPS CONFIG   */
#define GPS_RX_PIN 16   // GPS TX → ESP32 RX
#define GPS_TX_PIN 17   // GPS RX → ESP32 TX

/*   OBJECTS  */
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

/*  TIMER  */
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 30000; // 30 seconds


void setup() {
  Serial.begin(115200);
  delay(1000);

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  connectWiFi();
  setupFirebase();

  Serial.println("🚍 College Bus Tracking System Started");
  Serial.println("Waiting for GPS fix...");
}

/* the below code shoild be un comented when the gps module is connected*/
/*
void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  if (gps.location.isValid()) {
    if (millis() - lastUpdate >= updateInterval) {
      lastUpdate = millis();
      sendToFirebase();
    }
  }
}
*/

/* the below code is to test fire base and should be commented when the GPS module is connected*/
void loop() {

  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();
    sendToFirebase();   // simulated location
  }

}





/*Below is the old version of wifi connecting*/
/*void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\n✅ WiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}
*/

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - startAttemptTime < 15000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ WiFi Failed! Restarting...");
    ESP.restart();
  }
}

void setupFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("✅ Firebase Connected");
}

/* the below code shoild be un comented when the gps module is connected*/
/*
void sendToFirebase() {

  FirebaseJson json;

  json.set("latitude", gps.location.lat());
  json.set("longitude", gps.location.lng());
  json.set("speed_kmph", gps.speed.kmph());
  json.set("satellites", gps.satellites.value());
  json.set("timestamp", (unsigned long)(millis() / 1000));

  String path = "/bus_tracking/" BUS_ID "/location";

  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("📡 Location sent to Firebase");
    displayGPS();
  } else {
    Serial.print("❌ Firebase Error: ");
    Serial.println(fbdo.errorReason());
  }
}
*/

/* the below code is to test fire base and should be commented when the GPS module is connected*/
void sendToFirebase() {

  // Simulated coordinates (Hyderabad → small movement)
  static float lat = 17.3850;
  static float lng = 78.4867;

  lat += 0.0001;   // simulate movement
  lng += 0.0001;

  FirebaseJson json;

  json.set("latitude", lat);
  json.set("longitude", lng);
  json.set("speed_kmph", 35);
  json.set("satellites", 5);
  json.set("timestamp", (unsigned long)(millis() / 1000));

  String path = "/bus_tracking/" BUS_ID "/location";

  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("📡 Simulated location sent to Firebase");
  } else {
    Serial.print("❌ Firebase Error: ");
    Serial.println(fbdo.errorReason());
  }
}

void displayGPS() {
  Serial.println("════════════════════════════");
  Serial.print("Latitude: ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("Longitude: ");
  Serial.println(gps.location.lng(), 6);
  Serial.print("Speed (km/h): ");
  Serial.println(gps.speed.kmph());
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.println("════════════════════════════");
}