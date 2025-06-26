#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Firebase_ESP_Client.h>
#include <WiFi.h>
#include <Arduino.h>

// Pustaka tambahan untuk Firebase
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// Masukkan kredensial jaringan Wi-Fi tethering
#define WIFI_SSID "toya" // Nama hotspot smartphone Anda
#define WIFI_PASSWORD "anakneko" // Password hotspot smartphone Anda

// Masukkan URL Realtime Database Firebase dan API Key Anda
#define DATABASE_URL "https://semhas-44784-default-rtdb.asia-southeast1.firebasedatabase.app/" // URL Realtime Database Anda
#define API_KEY "AIzaSyAVtQaJS4STKy_5gI2WeHMYzQrCfmxdTNQ" // Ganti dengan API Key Firebase Anda

// Objek untuk data Firebase dan konfigurasi
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

// Definisikan pin RX dan TX untuk komunikasi dengan GPS Neo-6M
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// Buat objek untuk serial hardware dan TinyGPS++
HardwareSerial SerialGPS(1);
TinyGPSPlus gps;

void setup() {
  // Inisialisasi serial monitor
  Serial.begin(115200);
  delay(1000);
  
  // Inisialisasi serial untuk GPS Neo-6M
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("Menginisialisasi GPS Neo-6M...");

  // Menghubungkan ke WiFi dari tethering smartphone
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Mengatur API Key dan URL Realtime Database untuk Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  // Melakukan signup
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase signup successful.");
    signupOK = true;
  } else {
    Serial.printf("Firebase signup failed: %s\n", config.signer.signupError.message.c_str());
  }

  // Menetapkan callback untuk token generation
  config.token_status_callback = tokenStatusCallback;

  // Inisialisasi Firebase
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  // Baca data dari GPS
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  // Jika ada data lokasi yang tersedia
  if (gps.location.isUpdated()) {
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();

    Serial.print("Latitude : ");
    Serial.println(latitude, 8); // Tampilkan latitude dengan 8 desimal
    Serial.print("Longitude : ");
    Serial.println(longitude, 8); // Tampilkan longitude dengan 8 desimal

    // Kirim data ke Firebase setiap 1 detik
    if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
      sendDataPrevMillis = millis();

      // Mengirim latitude ke Firebase
      // Konversi latitude dan longitude menjadi string
      String latitudeStr = String(latitude, 8); // dengan 8 desimal
      String longitudeStr = String(longitude, 8); // dengan 8 desimal

      // Mengirim latitude ke Firebase sebagai string
      if (Firebase.RTDB.setString(&fbdo, "gps/latitude", latitudeStr)) {
        Serial.println("Latitude sent to Firebase successfully.");
      } else {
        Serial.println("Failed to send latitude to Firebase.");
        Serial.println("Reason: " + fbdo.errorReason());
      }

      // Mengirim longitude ke Firebase sebagai string
      if (Firebase.RTDB.setString(&fbdo, "gps/longitude", longitudeStr)) {
        Serial.println("Longitude sent to Firebase successfully.");
      } else {
        Serial.println("Failed to send longitude to Firebase.");
        Serial.println("Reason: " + fbdo.errorReason());
      }
    }
  } else {
    Serial.println("Mencari sinyal GPS...");
  }

  delay(5000); // Update setiap 5 detik
}