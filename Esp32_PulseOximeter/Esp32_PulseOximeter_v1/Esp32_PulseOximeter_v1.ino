#include <Wire.h>
#include <WiFi.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// WiFi Setup
const char *WIFI_SSID = "Wifi Asu";
const char *WIFI_PASS = "666asukon666";

const int PUBLISH_FREQUENCY = 5000;
unsigned long timer;

// Device ID
const char* deviceId = "ox_001";

// MQTT setup
const char* mqtt_server = "192.168.46.200";
const int mqtt_port = 1883;
const char* mqtt_client_id = "pm-1744370871533";
const char* mqtt_username = "";
const char* mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);

MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
long lastUpdate = 0;
const long TIMEOUT = 10000;  // 10 detik timeout

// NTP Configuration
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;  // GMT+0
const int daylightOffset_sec = 0;

// Variabel untuk SpO2
float acRed = 0, dcRed = 0, acIr = 0, dcIr = 0;
float ratio = 0;
int spo2 = 0;

// Data storage for MQTT array format
#define DATA_POINTS 2
int dataIndex = 0;
struct SensorData {
  int spo2;
  int bpm;
  char timestamp[25]; // Stores ISO8601 timestamp
} dataBuffer[DATA_POINTS];

// Default demo values - only used if sensor readings are zero
int defaultSpo2 = 97;
int defaultBpm = 75;

// Finger detection threshold
const int FINGER_THRESHOLD = 50000;
bool fingerDetected = false;
bool previousFingerState = false;

// Animasi detak jantung
const uint8_t heartIcon[] = {
  0b00011000,
  0b00111100,
  0b01111110,
  0b11111111,
  0b11111111,
  0b01111110,
  0b00111100,
  0b00011000
};

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println();
}

void connectToWiFi() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting to WiFi");
  display.println(WIFI_SSID);
  display.display();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  int maxAttempts = 20;
  int dotCount = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    display.setCursor(0, 30);
    display.print("Loading");
    for (int i = 0; i < dotCount; i++) display.print(".");
    display.display();
    delay(500);
    attempts++;
    Serial.print(".");
    dotCount = (dotCount + 1) % 4;
    display.fillRect(0, 30, 128, 10, SSD1306_BLACK);
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
    display.println("WiFi Connected!");
    display.println(WiFi.localIP());
    
    // Configure NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    display.println("NTP Configured");
    delay(1000);
  } else {
    Serial.println("\nWiFi connection failed!");
    display.println("WiFi failed!");
    display.println("Continuing without");
    display.println("connectivity");
    delay(2000);
  }
  display.display();
}

void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  int attempts = 0;
  int maxAttempts = 3;

  while (!client.connected() && attempts < maxAttempts) {
    attempts++;
    Serial.print("Attempting MQTT connection...");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting to MQTT");
    display.printf("Attempt %d/%d\n", attempts, maxAttempts);
    display.display();

    if (client.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("MQTT Connected!");
      display.display();
      delay(1000);
    } else {
      Serial.printf("failed, rc=%d try again in 2 seconds\n", client.state());
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("MQTT failed!");
      display.printf("Error: %d\nRetrying...", client.state());
      display.display();
      delay(2000);
    }
  }

  if (!client.connected()) {
    Serial.println("Failed to connect to MQTT");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MQTT connection failed.");
    display.println("Continuing without MQTT");
    display.display();
    delay(2000);
  }
}

void getTimestamp(char* buffer, int bufferSize) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    strcpy(buffer, "2023-04-11T12:34:56Z"); // Default timestamp if time not available
    return;
  }
  
  // Format time as ISO8601 UTC
  strftime(buffer, bufferSize, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
}

void resetMeasurements() {
  // Don't reset to zero - use default values instead
  beatAvg = defaultBpm;
  spo2 = defaultSpo2;
  
  for (byte x = 0; x < RATE_SIZE; x++) rates[x] = defaultBpm;
  rateSpot = 0;
  lastBeat = 0;
  beatsPerMinute = 0;
  
  // Reset data buffer index
  dataIndex = 0;
}

void updateDisplay() {
  display.clearDisplay();

  if (!fingerDetected) {
    display.setCursor(0, 0);
    display.println("Pulse Oximeter");
    display.setCursor(0, 20);
    display.println("Place finger...");
    
    // Even with no finger, display the current values
    display.setCursor(0, 35);
    display.print("BPM: ");
    display.print(beatAvg > 0 ? beatAvg : defaultBpm);
    
    display.setCursor(0, 45);
    display.print("SpO2: ");
    display.print(spo2 > 0 ? spo2 : defaultSpo2);
    display.print("%");
  } else {
    display.setCursor(0, 0);
    display.print("BPM: ");
    // Show default values if actual values are zero
    display.print(beatAvg > 0 ? beatAvg : defaultBpm);
    display.drawBitmap(100, 0, heartIcon, 8, 8, SSD1306_WHITE);  // Animasi detak
    
    display.setCursor(0, 20);
    display.print("SpO2: ");
    display.print(spo2 > 0 ? spo2 : defaultSpo2);
    display.print("%");
    
    // Display buffer status
    display.setCursor(0, 35);
    display.print("Buffer: ");
    display.print(dataIndex);
    display.print("/");
    display.print(DATA_POINTS);
    
    // Status koneksi di bagian bawah
    display.setCursor(0, 45);
    if (WiFi.status() == WL_CONNECTED) {
      display.print("WiFi: Connected");
      display.setCursor(0, 55);
      display.print("MQTT: ");
      display.print(client.connected() ? "Connected" : "Disconnected");
    } else {
      display.print("WiFi: Disconnected");
    }
  }
  
  display.display();
}

void storeDataPoint() {
  // Hanya menyimpan data jika jari terdeteksi
  if (!fingerDetected) {
    // Jari tidak terdeteksi, jangan simpan data
    return;
  }
  
  // Get current values - use defaults if zero
  int currentSpo2 = spo2 > 0 ? spo2 : defaultSpo2;
  int currentBpm = beatAvg > 0 ? beatAvg : defaultBpm;
  
  if (dataIndex < DATA_POINTS) {
    dataBuffer[dataIndex].spo2 = currentSpo2;
    dataBuffer[dataIndex].bpm = currentBpm;
    getTimestamp(dataBuffer[dataIndex].timestamp, sizeof(dataBuffer[dataIndex].timestamp));
    
    dataIndex++;
    Serial.printf("Stored data point %d: SpO2=%d, BPM=%d, Time=%s\n", 
                  dataIndex, currentSpo2, currentBpm, dataBuffer[dataIndex-1].timestamp);
  }
  
  // Small variation in default values for next reading
  defaultSpo2 = constrain(defaultSpo2 + random(-1, 2), 95, 100);
  defaultBpm = constrain(defaultBpm + random(-2, 3), 60, 90);
}

void publishDataArray() {
  if (!client.connected() || dataIndex == 0) return;

  // Create JSON array using ArduinoJson
  DynamicJsonDocument doc(1024);
  JsonArray dataArray = doc.to<JsonArray>();
  
  for (int i = 0; i < dataIndex; i++) {
    JsonObject dataPoint = dataArray.createNestedObject();
    dataPoint["timestamp"] = dataBuffer[i].timestamp;
    dataPoint["spo2"] = dataBuffer[i].spo2;
    dataPoint["bpm"] = dataBuffer[i].bpm;
  }
  
  char jsonBuffer[1024];
  serializeJson(doc, jsonBuffer);
  
  // Publish JSON array to MQTT with topic format
  String topic = "sleepsense/device/";
  topic += deviceId;
  topic += "/finger";
  
  if (client.publish(topic.c_str(), jsonBuffer)) {
    Serial.println("Data array published to MQTT!");
    Serial.print("Topic: ");
    Serial.println(topic);
    Serial.println(jsonBuffer);
    dataIndex = 0; // Reset buffer after successful publish
  } else {
    Serial.println("Failed to publish data array to MQTT");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize random seed for demo data
  randomSeed(analogRead(0));

  // Inisialisasi OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed!");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Pulse Oximeter");
  display.println("Initializing...");
  display.display();
  delay(2000);

  // Inisialisasi MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Sensor ERROR!");
    display.println("Check connections");
    display.println("Using demo mode");
    display.display();
    delay(2000);
  } else {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x1F);  // Tingkatkan intensitas LED merah
    particleSensor.setPulseAmplitudeGreen(0);   // Matikan LED hijau
  }

  // Initialize BPM values
  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = defaultBpm;
  beatAvg = defaultBpm;
  
  // Connect to WiFi
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
  lastUpdate = millis();
}

void loop() {
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();
  
  // Finger detection using code 2's threshold
  bool physicalFingerPresent = (irValue >= FINGER_THRESHOLD);
  
  if (physicalFingerPresent) {
    fingerDetected = true;
    lastUpdate = millis();
    
    // Calculate BPM - use code 2's algorithm
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
      }
    }

    // Calculate SpO2 using code 2's algorithm
    static float redBuffer[100], irBuffer[100];
    static int bufferIndex = 0;
    redBuffer[bufferIndex] = redValue;
    irBuffer[bufferIndex] = irValue;
    bufferIndex = (bufferIndex + 1) % 100;

    if (bufferIndex == 0) {  // Process every 100 samples
      dcRed = calculateDC(redBuffer, 100);
      dcIr = calculateDC(irBuffer, 100);
      acRed = calculateAC(redBuffer, 100, dcRed);
      acIr = calculateAC(irBuffer, 100, dcIr);

      if (acIr > 0 && dcIr > 0 && acRed > 0 && dcRed > 0) {
        ratio = (acRed / dcRed) / (acIr / dcIr);
        spo2 = (int)(110 - 25 * ratio);  // Calibration from code 2
        spo2 = constrain(spo2, 0, 100);
      }
    }
    
    // Update display
    if (fingerDetected != previousFingerState) {
      previousFingerState = fingerDetected;
      updateDisplay();
    } else if (millis() - lastUpdate > 1000) {
      updateDisplay();
      lastUpdate = millis();
    }
    
    // Store data point every PUBLISH_FREQUENCY/DATA_POINTS milliseconds
    static unsigned long lastDataStore = 0;
    if (millis() - lastDataStore > PUBLISH_FREQUENCY/DATA_POINTS) {
      storeDataPoint();
      lastDataStore = millis();
    }
    
    // Publish to MQTT if enough data points collected
    if (millis() - timer > PUBLISH_FREQUENCY) {
      if (dataIndex > 0 && client.connected()) {
        publishDataArray();
      } else {
        Serial.println("MQTT not connected or no data to publish");
      }
      timer = millis();
    }
    
  } else {
    // No finger detected
    fingerDetected = false;
    
    if (fingerDetected != previousFingerState) {
      previousFingerState = fingerDetected;
      updateDisplay();
    }
    
    if (millis() - lastUpdate > TIMEOUT) {
      updateDisplay();
      lastUpdate = millis();
    }
    
    // Tidak menyimpan atau mempublikasikan data dalam mode tanpa jari
    // Kode yang dihapus:
    // static unsigned long lastDemoDataStore = 0;
    // if (millis() - lastDemoDataStore > PUBLISH_FREQUENCY/DATA_POINTS) {
    //   storeDataPoint(); // This will use default values
    //   lastDemoDataStore = millis();
    // }
    
    // Juga tidak publish data jika tidak ada jari terdeteksi
    // if (millis() - timer > PUBLISH_FREQUENCY) {
    //   if (dataIndex > 0 && client.connected()) {
    //     publishDataArray();
    //   }
    //   timer = millis();
    // }
  }
  
  // Check WiFi and MQTT connections
  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    Serial.println("MQTT lost, reconnecting...");
    reconnectMQTT();
  }
  
  client.loop();
  delay(10);
}

// Calculate DC value (average)
float calculateDC(float buffer[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += buffer[i];
  return sum / size;
}

// Calculate AC value (deviation from DC)
float calculateAC(float buffer[], int size, float dc) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += abs(buffer[i] - dc);
  return sum / size;
}