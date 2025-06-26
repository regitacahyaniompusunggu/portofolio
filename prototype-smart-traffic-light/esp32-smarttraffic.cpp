#include <Firebase_ESP_Client.h>
#include <Arduino.h>
#include <math.h>

#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif

// Firebase and WiFi configuration
#define WIFI_SSID "haru" //Nama hotspot smartphone Anda
#define WIFI_PASSWORD "anakmama" //Password hotspot smartphone Anda
#define DATABASE_URL "https://semhas-44784-default-rtdb.asia-southeast1.firebasedatabase.app/" // URL Realtime Database Anda
#define API_KEY "AIzaSyAVtQaJS4STKy_5gI2WeHMYzQrCfmxdTNQ" // Ganti dengan API Key Firebase Anda

// Define fixed coordinates (latitude1, longitude1)
const float latitude1 = -0.46386467; // contoh latitude tetap
const float longitude1 = 117.14925617; // contoh longitude tetap

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool signupOK = false;
unsigned long lastReadTime = 0; // To track Firebase reading time
bool interupsi = false; // Flag untuk menentukan apakah ada interupsi

// Traffic light pin definitions
const int rp1 = 2, yp1 = 4, gp1 = 5;
const int rp2 = 18, yp2 = 19, gp2 = 21;
const int rp3 = 14, yp3 = 12, gp3 = 13;
const int rp4 = 27, yp4 = 26, gp4 = 25;

// Timing for traffic light phases (milliseconds)
const unsigned long waktuMerah = 15000;
const unsigned long waktuKuning = 3000;
const unsigned long waktuHijau = 10000;
const unsigned long waktumerah = 500;
const unsigned long waktum = 3000;

// Callback function to handle token status updates
void tokenStatusCallback(token_info_t info) {
  if (info.status == token_status_ready) {
    signupOK = true;
    Serial.println("Firebase token is ready");
  } else {
    Serial.print("Token status: ");
    Serial.println(info.status);
    Serial.print("Token type: ");
    Serial.println(info.type);
    Serial.printf("Token error: %s\n", info.error.message.c_str());
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize WiFi and Firebase connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nConnected with IP: " + WiFi.localIP());

  // Firebase setup
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback; // Set callback function
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase signup successful.");
    signupOK = true;
  } else {
    Serial.printf("Firebase signup failed: %s\n", config.signer.signupError.message.c_str());
  }
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Initialize traffic light pins as output
  pinMode(rp1, OUTPUT); pinMode(yp1, OUTPUT); pinMode(gp1, OUTPUT);
  pinMode(rp2, OUTPUT); pinMode(yp2, OUTPUT); pinMode(gp2, OUTPUT);
  pinMode(rp3, OUTPUT); pinMode(yp3, OUTPUT); pinMode(gp3, OUTPUT);
  pinMode(rp4, OUTPUT); pinMode(yp4, OUTPUT); pinMode(gp4, OUTPUT);
}

void loop() {
  if (!interupsi) {
    // Run traffic light control independently
    controlTrafficLights();
  }

  // Only read from Firebase every 5 seconds without interrupting the traffic lights
  if (millis() - lastReadTime > 5000) { // Ubah interval menjadi 5000 milidetik (5 detik)
    readFromFirebase();
    lastReadTime = millis();
  }
}

void readFromFirebase() {
    double latitude2, longitude2; // Ubah tipe data menjadi double untuk presisi tinggi
    bool readSuccess = true;

    // Membaca latitude dari Firebase sebagai string
    if (Firebase.RTDB.getString(&fbdo, "gps/latitude")) {
        String latitudeString = fbdo.stringData();
        latitude2 = atof(latitudeString.c_str()); // Konversi string ke double
        Serial.print("Latitude dari Firebase: ");
        Serial.println(latitude2, 8); // Tampilkan dengan presisi 8 desimal
    } else {
        Serial.println("Gagal membaca latitude dari Firebase. Alasan: " + fbdo.errorReason());
        readSuccess = false;
    }

    // Membaca longitude dari Firebase sebagai string
    if (Firebase.RTDB.getString(&fbdo, "gps/longitude")) {
        String longitudeString = fbdo.stringData();
        longitude2 = atof(longitudeString.c_str()); // Konversi string ke double
        Serial.print("Longitude dari Firebase: ");
        Serial.println(longitude2, 8); // Tampilkan dengan presisi 8 desimal
    } else {
        Serial.println("Gagal membaca longitude dari Firebase. Alasan: " + fbdo.errorReason());
        readSuccess = false;
    }

    // Lakukan perhitungan jarak dan arah jika data berhasil dibaca
    if (readSuccess) {
        double distance = haversine(latitude1, longitude1, latitude2, longitude2);
        Serial.print("Jarak antara titik tetap dan Firebase: ");
        Serial.print(distance * 1000); // Konversi ke meter
        Serial.println(" meter");

        if (distance * 1000 <= 100) { // Validasi jarak sebelum menghitung arah
            double bearing = calculateBearing(latitude1, longitude1, latitude2, longitude2);
            Serial.print("Arah dari titik tetap ke Firebase: ");
            Serial.print(bearing);
            Serial.println(" derajat");

            // Panggil fungsi untuk memproses data lebih lanjut jika jarak memenuhi syarat
            checkProximityAndDirection(bearing);
        } else {
            Serial.println("Jarak lebih dari 100 meter.");
            if (interupsi) {
                interupsi = false;
                controlTrafficLights();
            }
        }
    }
}
void checkProximityAndDirection(float bearing) {
  interupsi = true;

  // Tentukan arah berdasarkan bearing
  String direction;
  if (bearing >= 315 || bearing < 45) {
    direction = "Utara";
    setTrafficLight(rp1, yp1, gp1);
  } else if (bearing >= 45 && bearing < 135) {
    direction = "Timur";
    setTrafficLight(rp2, yp2, gp2);
  } else if (bearing >= 135 && bearing < 225) {
    direction = "Selatan";
    setTrafficLight(rp3, yp3, gp3);
  } else {
    direction = "Barat";
    setTrafficLight(rp4, yp4, gp4);
  }

  Serial.println("Interupsi aktif: Lampu lalu lintas diatur berdasarkan arah.");
  Serial.print("Arah kedatangan: ");
  Serial.println(direction);
}

// Function to set a specific traffic light green and others red
void setTrafficLight(int rp, int yp, int gp) {
  // Set all traffic lights to red
  digitalWrite(rp1, HIGH); digitalWrite(yp1, LOW); digitalWrite(gp1, LOW);
  digitalWrite(rp2, HIGH); digitalWrite(yp2, LOW); digitalWrite(gp2, LOW);
  digitalWrite(rp3, HIGH); digitalWrite(yp3, LOW); digitalWrite(gp3, LOW);
  digitalWrite(rp4, HIGH); digitalWrite(yp4, LOW); digitalWrite(gp4, LOW);

  // Set the specific light to green
  digitalWrite(rp, LOW);
  digitalWrite(gp, HIGH);
}

// Traffic light control function
void controlTrafficLights() {
  // Intersection 1
  digitalWrite(rp1, HIGH);
  digitalWrite(yp1, LOW);
  digitalWrite(gp1, LOW);
  digitalWrite(rp2, HIGH);
  digitalWrite(yp2, LOW);
  digitalWrite(gp2, LOW);
  digitalWrite(rp3, HIGH);
  digitalWrite(yp3, LOW);
  digitalWrite(gp3, LOW);
  digitalWrite(rp4, HIGH);
  digitalWrite(yp4, LOW);
  digitalWrite(gp4, LOW);

  // Simpang 1
  digitalWrite(rp1, HIGH);
  digitalWrite(yp1, LOW);
  digitalWrite(gp1, LOW);
  delay(waktum);

  digitalWrite(rp1, LOW);
  digitalWrite(yp1, HIGH);
  digitalWrite(gp1, LOW);
  delay(waktuKuning);

  digitalWrite(rp1, LOW);
  digitalWrite(yp1, LOW);
  digitalWrite(gp1, HIGH);
  delay(waktuHijau);

  digitalWrite(rp1, LOW);
  digitalWrite(yp1, HIGH);
  digitalWrite(gp1, LOW);
  delay(waktuKuning);

  digitalWrite(rp1, HIGH);
  digitalWrite(yp1, LOW);
  digitalWrite(gp1, LOW);
  delay(waktumerah);

  // Simpang 2
  digitalWrite(rp2, LOW);
  digitalWrite(yp2, HIGH);
  digitalWrite(gp2, LOW);
  delay(waktuKuning);

  digitalWrite(rp2, LOW);
  digitalWrite(yp2, LOW);
  digitalWrite(gp2, HIGH);
  delay(waktuHijau);

  digitalWrite(rp2, LOW);
  digitalWrite(yp2, HIGH);
  digitalWrite(gp2, LOW);
  delay(waktuKuning);

  digitalWrite(rp2, HIGH);
  digitalWrite(yp2, LOW);
  digitalWrite(gp2, LOW);
  delay(waktumerah);

  // Simpang 3
  digitalWrite(rp3, LOW);
  digitalWrite(yp3, HIGH);
  digitalWrite(gp3, LOW);
  delay(waktuKuning);

  digitalWrite(rp3, LOW);
  digitalWrite(yp3, LOW);
  digitalWrite(gp3, HIGH);
  delay(waktuHijau);

  digitalWrite(rp3, LOW);
  digitalWrite(yp3, HIGH);
  digitalWrite(gp3, LOW);
  delay(waktuKuning);

  digitalWrite(rp3, HIGH);
  digitalWrite(yp3, LOW);
  digitalWrite(gp3, LOW);
  delay(waktumerah);

  // Simpang 4
  digitalWrite(rp4, LOW);
  digitalWrite(yp4, HIGH);
  digitalWrite(gp4, LOW);
  delay(waktuKuning);

  digitalWrite(rp4, LOW);
  digitalWrite(yp4, LOW);
  digitalWrite(gp4, HIGH);
  delay(waktuHijau);

  digitalWrite(rp4, LOW);
  digitalWrite(yp4, HIGH);
  digitalWrite(gp4, LOW);
  delay(waktuKuning);

  digitalWrite(rp4, HIGH);
  digitalWrite(yp4, LOW);
  digitalWrite(gp4, LOW);
  delay(waktumerah);
}

// Function to calculate the Haversine distance between two points
float haversine(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371.0; // Earth radius in kilometers
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  double dLon = radians((double)lon2 - (double)lon1);
  double radLat1 = radians((double)lat1);
  double radLat2 = radians((double)lat2);

  double y = sin(dLon) * cos(radLat2);
  double x = cos(radLat1) * sin(radLat2) - sin(radLat1) * cos(radLat2) * cos(dLon);

  double bearing = atan2(y, x);
  return fmod(degrees(bearing) + 360.0, 360.0);
}