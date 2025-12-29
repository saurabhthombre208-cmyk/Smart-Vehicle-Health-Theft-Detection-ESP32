#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/* ---------- WIFI DETAILS ---------- */
char ssid[] = "saurabh";
char pass[] = "123456789";
WiFiClient client;

/* ---------- THINGSPEAK DETAILS ---------- */
unsigned long myChannelNumber = 3180770;
const char * myWriteAPIKey = "S9OLVU27F8TYKKIG";

/* ---------- TELEGRAM BOT ---------- */
#define BOT_TOKEN "8259919176:AAHM445usdtHSb5BnTBJ0rcIIciTL2OFLHI"
String chatID = "1439793760";

/* ---------- HARDWARE ---------- */
#define RELAY_PIN   26
#define BUZZER_PIN  25
#define BATTERY_PIN 34

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

/* ---------- GLOBAL VARIABLES ---------- */
float lati = 0.0, longi = 0.0;
bool theftDetected = false;
unsigned long lastMovementTime = 0;

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  /* MPU6050 INIT */
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  /* WIFI INIT */
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
}

/* ---------- LOOP ---------- */
void loop() {
  detectTheft();
  readGPS();
  sendToThingSpeak();
  delay(15000);   // ThingSpeak minimum update time
}

/* ---------- THEFT DETECTION ---------- */
void detectTheft() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Gravity compensated vibration */
  float vibration =
    abs(a.acceleration.x) +
    abs(a.acceleration.y) +
    abs(a.acceleration.z - 9.8);

  if (vibration > 1.5) {
    if (millis() - lastMovementTime > 2000 && !theftDetected) {
      theftDetected = true;
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(RELAY_PIN, HIGH);   // Engine lock
      sendTelegramAlert();
      Serial.println("🚨 THEFT DETECTED");
    }
  } else {
    lastMovementTime = millis();
  }
}

/* ---------- GPS ---------- */
void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      lati  = gps.location.lat();
      longi = gps.location.lng();
    }
  }
}

/* ---------- BATTERY ---------- */
float readBatteryVoltage() {
  int adc = analogRead(BATTERY_PIN);
  float voltage = (adc / 4095.0) * 3.3 * 4.2; // calibrated scaling
  return voltage;
}

/* ---------- THINGSPEAK ---------- */
void sendToThingSpeak() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String url = "http://api.thingspeak.com/update";
    url += "?api_key=" + String(myWriteAPIKey);
    url += "&field1=" + String(readBatteryVoltage());
    url += "&field2=" + String(theftDetected);
    url += "&field3=" + String(lati, 6);
    url += "&field4=" + String(longi, 6);

    http.begin(url);
    http.GET();
    http.end();

    Serial.println("☁️ Data sent to ThingSpeak");
  }
}

/* ---------- TELEGRAM ALERT ---------- */
void sendTelegramAlert() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String message = "🚨 VEHICLE THEFT ALERT\n";
    message += "🔋 Battery: " + String(readBatteryVoltage()) + " V\n";
    message += "📍 Location:\n";
    message += "https://maps.google.com/?q=";
    message += String(lati, 6) + "," + String(longi, 6);

    String url = "https://api.telegram.org/bot";
    url += BOT_TOKEN;
    url += "/sendMessage?chat_id=" + chatID;
    url += "&text=" + message;

    http.begin(url);
    http.GET();
    http.end();

    Serial.println("📩 Telegram Alert Sent");
  }
}
