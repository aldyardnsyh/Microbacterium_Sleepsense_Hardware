#include <Wire.h>
#include <WiFi.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//WiFi Setup
const char *WIFI_SSID = "Wifi Asu";
const char *WIFI_PASS = "666asukon666";

const int PUBLISH_FREQUENCY = 5000;
unsigned long timer;

//MQTT setup
const char* mqtt_server = "192.168.26.236";  // Ganti dengan IP laptop kamu
const int mqtt_port = 1883;
const char* mqtt_client_id = "arduino_sleepsense";
const char* mqtt_username = "";
const char* mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);

// Sensor setup
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
float acRed = 0, dcRed = 0, acIr = 0, dcIr = 0;
float ratio = 0;
int spo2 = 0;

// Finger detection threshold
const int FINGER_THRESHOLD = 10000;
bool fingerDetected = false;
bool previousFingerState = false;

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // OLED init
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Tampilkan pesan startup
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("SleepSense");
  display.println("Initializing...");
  display.display();
  delay(1000);
  
  // MAX30105 init
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Sensor ERROR!");
    display.println("Check connections");
    display.display();
    while (1);
  }
  
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeGreen(0);
  
  // Connect to WiFi - ONLY call this once
  connectToWiFi();
  
  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnectMQTT();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ready!");
  display.println("Place finger on");
  display.println("sensor...");
  display.display();
  
  timer = millis();
}

void connectToWiFi() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting to WiFi");
  display.println(WIFI_SSID);
  display.display();

  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  int maxAttempts = 20; // 20 attempts * 500ms = 10 seconds timeout
  int dotCount = 0;
  
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    display.setCursor(0, 30);
    display.print("Loading");
    for (int i = 0; i < dotCount; i++) {
      display.print(".");
    }
    display.display();
    
    delay(500);
    attempts++;
    Serial.print(".");
    
    dotCount = (dotCount + 1) % 4;
    display.fillRect(0, 30, 128, 10, SSD1306_BLACK);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Connected!");
    display.println(WiFi.localIP());
    display.display();
    delay(1000);
  } else {
    Serial.println();
    Serial.println("WiFi connection failed!");
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi failed!");
    display.println("Continuing without");
    display.println("connectivity");
    display.display();
    delay(2000);
  }
}

void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping MQTT connection");
    return;
  }
  
  int attempts = 0;
  int maxAttempts = 3;  // Maximum 3 attempts
  
  while (!client.connected() && attempts < maxAttempts) {
    attempts++;
    Serial.print("Attempting MQTT connection...");
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting to MQTT");
    display.print("Attempt ");
    display.print(attempts);
    display.print("/");
    display.println(maxAttempts);
    display.display();
    
    if (client.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("MQTT Connected!");
      display.display();
      delay(1000);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("MQTT failed!");
      display.print("Error code: ");
      display.println(client.state());
      display.println("Retrying...");
      display.display();
      
      delay(2000);
    }
  }
  
  if (!client.connected() && attempts >= maxAttempts) {
    Serial.println("Failed to connect to MQTT after multiple attempts");
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MQTT connection");
    display.println("failed.");
    display.println("Continuing without MQTT");
    display.display();
    delay(2000);
  }
}

void resetMeasurements() {
  // Reset semua pengukuran saat jari dicabut
  beatAvg = 0;
  spo2 = 0;
  
  // Reset array rates
  for (byte x = 0; x < RATE_SIZE; x++) {
    rates[x] = 0;
  }
  rateSpot = 0;
  
  // Reset buffer
  lastBeat = 0;
  beatsPerMinute = 0;
}

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  
  if (!fingerDetected) {
    display.println("No finger detected");
    display.println("Place finger on");
    display.println("sensor...");
  } else {
    display.println("SleepSense Monitor");
    display.println("----------------");
    
    display.print("Heart Rate: ");
    if (beatAvg > 0) {
      display.print(beatAvg);
      display.println(" BPM");
    } else {
      display.println("--");
    }
    
    display.print("SpO2: ");
    if (spo2 > 0 && spo2 <= 100) {
      display.print(spo2);
      display.println("%");
    } else {
      display.println("--");
    }
    
    // Status connection
    display.println("----------------");
    if (WiFi.status() == WL_CONNECTED) {
      if (client.connected()) {
        display.println("MQTT: Connected");
      } else {
        display.println("MQTT: Disconnected");
      }
    } else {
      display.println("WiFi: Disconnected");
    }
  }
  
  display.display();
}

void loop() {
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();
  
  // Detect Finger
  fingerDetected = (irValue > FINGER_THRESHOLD);
  
  // Reset display if finger disconnected
  if (fingerDetected != previousFingerState) {
    if (!fingerDetected && previousFingerState) {
      resetMeasurements();
      Serial.println("Finger removed - measurements reset");
    }
    
    // Update status jari sebelumnya
    previousFingerState = fingerDetected;
    
    // Update display karena status jari berubah
    updateDisplay();
  }
  
  //BPM
  if (fingerDetected) {
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);
      
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
        
        updateDisplay();
      }
    }
    
    // SpO2
    static float redBuffer[100], irBuffer[100];
    static int bufferIndex = 0;
    redBuffer[bufferIndex] = redValue;
    irBuffer[bufferIndex] = irValue;
    bufferIndex = (bufferIndex + 1) % 100;
    
    if (bufferIndex == 0) {
      dcRed = calculateDC(redBuffer, 100);
      dcIr = calculateDC(irBuffer, 100);
      acRed = calculateAC(redBuffer, 100, dcRed);
      acIr = calculateAC(irBuffer, 100, dcIr);
      
      if (acIr > 0 && dcIr > 0 && acRed > 0 && dcRed > 0) {
        ratio = (acRed / dcRed) / (acIr / dcIr);
        spo2 = (int)(110 - 25 * ratio);
        spo2 = constrain(spo2, 0, 100);
        
        // Update display saat SpO2 dihitung
        updateDisplay();
      }
    }
    
    // Kirim ke MQTT tiap 5 detik (hanya jika jari terdeteksi)
    if (millis() - timer > PUBLISH_FREQUENCY) {
      // Hanya kirim data jika valid dan MQTT terhubung
      if (beatAvg > 0 && spo2 > 0 && client.connected()) {
        char payload[100];
        sprintf(payload, "{\"bpm\": %d, \"spo2\": %d}", beatAvg, spo2);
        
        if (client.publish("sleepsense/pulse_oximeter", payload)) {
          Serial.println("Data sent to MQTT!");
        } else {
          Serial.println("Failed to publish data to MQTT");
        }
      } else if (!client.connected()) {
        Serial.println("MQTT not connected, data not sent");
      } else {
        Serial.println("Data belum valid, MQTT update dilewati");
      }
      timer = millis();
    }
  }
  
  // Check MQTT connection
  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    Serial.println("MQTT connection lost, attempting to reconnect...");
    reconnectMQTT();
  }
  
  client.loop();
}

// Hitung DC - algoritma asli tetap digunakan
float calculateDC(float buffer[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += buffer[i];
  return sum / size;
}

// Hitung AC - algoritma asli tetap digunakan
float calculateAC(float buffer[], int size, float dc) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += abs(buffer[i] - dc);
  return sum / size;
}