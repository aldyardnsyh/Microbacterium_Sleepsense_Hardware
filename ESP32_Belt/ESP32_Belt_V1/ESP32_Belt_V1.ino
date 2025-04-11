#include <WiFi.h>
#define MQTT_MAX_PACKET_SIZE 4096  // Must be defined before including PubSubClient
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

// Pin definitions
#define RCWL_PIN 27       // Pin GPIO untuk sensor gerak RCWL-0516
#define ECG_OUTPUT_PIN 34 // Pin analog untuk output AD8232
#define ECG_LO_POS 26     // Pin Lead-Off Positif
#define ECG_LO_NEG 25     // Pin Lead-Off Negatif
#define PIEZO_PIN 35      // Pin analog untuk piezoelectric

// WiFi Setup
const char *WIFI_SSID = "Wifi Asu";
const char *WIFI_PASS = "666asukon666";

// Device ID
const char* deviceId = "belt_001";

// MQTT setup
const char* mqtt_server = "192.168.46.200";
const int mqtt_port = 1883;
const char* mqtt_client_id = "pm-1744370871533";
const char* mqtt_username = "";
const char* mqtt_password = "";

// Publish frequency
const int PUBLISH_FREQUENCY = 5000; // Publish every 5 seconds
unsigned long publishTimer;

// Data collection frequency
const int DATA_COLLECTION_INTERVAL = 200; // Collect data every 200ms
unsigned long dataCollectionTimer;

// Konversi ADC
const float ADC_REFERENCE = 3.3;    // Voltage reference in volts
const int ADC_RESOLUTION = 4096;    // 12-bit resolution (0-4095)

// Variabel untuk menyimpan nilai sensor
bool motionDetected = false;
int ecgRawValue = 0;
float ecgMv = 0.0;         // ECG value in millivolts
int piezoRawValue = 0;
float piezoMv = 0.0;       // Piezo value in millivolts
int rcwlRawValue = 0;      // Added for RCWL amplitude
bool ecgConnected = true;

// Data buffer untuk kumpulan data - reduced from 5 to 3 to prevent MQTT buffer overflow
const int MAX_DATA_POINTS = 3;     // Reduced buffer size to prevent MQTT overflows
typedef struct {
  char timestamp[25];
  float ecg;               // ECG dalam mV
  float piezoelectric_voltage; // Piezo dalam mV
  int rcwl_amplitude;      // Nilai amplitude RCWL (0-100 atau nilai tetap jika deteksi binary)
} SensorDataPoint;

SensorDataPoint dataBuffer[MAX_DATA_POINTS];
int dataBufferIndex = 0;

// Interval yang dioptimalkan
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 1000; // Update serial setiap 1 detik

// MQTT retry mechanism
const int MQTT_RETRY_DELAY = 2000;  // 2 seconds between retries
const int MQTT_MAX_RETRIES = 3;     // Maximum number of publish retries
int mqttRetryCounter = 0;

// NTP Configuration
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;  // GMT+7 for Indonesia
const int daylightOffset_sec = 0;

// WiFi reconnect parameters
unsigned long wifiReconnectTimer = 0;
const int WIFI_RECONNECT_INTERVAL = 30000; // Try to reconnect every 30 seconds

// MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// Convert ADC value to millivolts
float adcToMillivolts(int adcValue) {
  return (adcValue * ADC_REFERENCE * 1000.0) / ADC_RESOLUTION;
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  int maxAttempts = 20;
  
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    attempts++;
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
    
    // Configure NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("NTP Configured");
  } else {
    Serial.println("\nWiFi connection failed!");
    Serial.println("Continuing without connectivity");
  }
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentTime = millis();
    if (currentTime - wifiReconnectTimer >= WIFI_RECONNECT_INTERVAL) {
      Serial.println("WiFi connection lost, reconnecting...");
      WiFi.disconnect();
      delay(1000);
      connectToWiFi();
      wifiReconnectTimer = currentTime;
    }
  }
}

bool reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return false;
  
  int attempts = 0;
  int maxAttempts = 3;

  while (!client.connected() && attempts < maxAttempts) {
    attempts++;
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      return true;
    } else {
      Serial.printf("failed, rc=%d try again in 2 seconds\n", client.state());
      delay(2000);
    }
  }

  if (!client.connected()) {
    Serial.println("Failed to connect to MQTT");
    Serial.println("Continuing without MQTT");
    return false;
  }
  
  return true;
}

void getTimestamp(char* buffer, int bufferSize) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    strcpy(buffer, "2025-04-12T12:34:56Z"); // Default timestamp jika waktu tidak tersedia
    return;
  }
  
  // Format time as ISO8601 UTC
  strftime(buffer, bufferSize, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
}

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(115200);
  while (!Serial) ; // Wait for serial to initialize
  
  Serial.println("\n\n=================================");
  Serial.println("ESP32 MQTT Sensor Monitoring");
  Serial.println("=================================");
 
  // Setup pin sensor
  pinMode(RCWL_PIN, INPUT);
  pinMode(ECG_LO_POS, INPUT); // Pin lead-off detection
  pinMode(ECG_LO_NEG, INPUT); // Pin lead-off detection
 
  // Konfigurasi ADC
  analogReadResolution(12); // 12-bit resolution (0-4095)
 
  // Connect to WiFi
  connectToWiFi();
  
  // Setup MQTT with increased buffer size
  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
  client.setServer(mqtt_server, mqtt_port);
  reconnectMQTT();
  
  // Initialize all data buffer timestamps to empty
  for (int i = 0; i < MAX_DATA_POINTS; i++) {
    dataBuffer[i].timestamp[0] = '\0';
  }
  
  // Tunggu stabilisasi sensor
  delay(500);
 
  Serial.println("Sistem siap! Monitoring dimulai...");
  Serial.println("=================================");

  // Print header untuk output serial (CSV style)
  Serial.println("Timestamp,ECG(mV),Piezo(mV),RCWL,WiFi,MQTT");
  
  publishTimer = millis();
  dataCollectionTimer = millis();
  wifiReconnectTimer = millis();
}

void loop() {
  unsigned long currentTime = millis();
 
  // Check WiFi connection
  checkWiFiConnection();
  
  // Check MQTT connection if WiFi is connected
  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    Serial.println("MQTT disconnected, reconnecting...");
    reconnectMQTT();
  }
  
  // Collect data at regular intervals
  if (currentTime - dataCollectionTimer >= DATA_COLLECTION_INTERVAL) {
    dataCollectionTimer = currentTime;
    collectAndStoreData();
  }
 
  // Update tampilan serial dengan interval yang lebih lambat
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;
    updateSerialOutput();
  }
  
  // Publish data ke MQTT
  if (currentTime - publishTimer >= PUBLISH_FREQUENCY) {
    publishTimer = currentTime;
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      publishSensorData();
    } else {
      Serial.println("Cannot publish: Network not ready");
    }
  }
  
  client.loop();
}

void collectAndStoreData() {
  // Baca sensor gerak
  motionDetected = digitalRead(RCWL_PIN);
  
  // Assign RCWL amplitude - binary detection dikonversi ke nilai skala
  rcwlRawValue = motionDetected ? 100 : 0;
 
  // Cek koneksi sensor ECG dan baca nilai
  if ((digitalRead(ECG_LO_POS) == 1) || (digitalRead(ECG_LO_NEG) == 1)) {
    ecgConnected = false;
    ecgMv = 0.0; // Set to 0 jika tidak terkoneksi
  } else {
    ecgConnected = true;
    // Baca nilai ECG
    ecgRawValue = analogRead(ECG_OUTPUT_PIN);
    ecgMv = adcToMillivolts(ecgRawValue);
  }
 
  // Baca nilai piezo
  piezoRawValue = analogRead(PIEZO_PIN);
  piezoMv = adcToMillivolts(piezoRawValue);
  
  // Store data in the buffer
  getTimestamp(dataBuffer[dataBufferIndex].timestamp, 25);
  dataBuffer[dataBufferIndex].ecg = ecgMv;
  dataBuffer[dataBufferIndex].piezoelectric_voltage = piezoMv;
  dataBuffer[dataBufferIndex].rcwl_amplitude = rcwlRawValue;
  
  // Move to next data point
  dataBufferIndex = (dataBufferIndex + 1) % MAX_DATA_POINTS;
}

void updateSerialOutput() {
  // Get current timestamp
  char timestamp[25];
  getTimestamp(timestamp, sizeof(timestamp));
  
  // Print data in CSV format
  Serial.print(timestamp);
  Serial.print(",");
  
  // ECG value (in mV)
  if (ecgConnected) {
    Serial.print(ecgMv, 2); // Print with 2 decimal places
  } else {
    Serial.print("NC"); // Not Connected
  }
  Serial.print(",");
  
  // Piezo value (in mV)
  Serial.print(piezoMv, 2); // Print with 2 decimal places
  Serial.print(",");
  
  // RCWL amplitude
  Serial.print(rcwlRawValue);
  Serial.print(",");
  
  // WiFi status
  Serial.print(WiFi.status() == WL_CONNECTED ? "1" : "0");
  Serial.print(",");
  
  // MQTT status
  Serial.println(client.connected() ? "1" : "0");
}

bool publishSensorData() {
  if (!client.connected() || WiFi.status() != WL_CONNECTED) {
    Serial.println("Cannot publish: MQTT or WiFi not connected");
    return false;
  }
  
  // Buat JSON dokumen - cukup besar untuk menampung array data
  DynamicJsonDocument doc(3072);  // Reduced size to avoid memory fragmentation
  
  // Create JSON array
  JsonArray dataArray = doc.to<JsonArray>();
  
  // Count valid data points
  int validPoints = 0;
  int currentIndex = dataBufferIndex;  // Start from the oldest data
  
  // Add data points to array
  for (int i = 0; i < MAX_DATA_POINTS; i++) {
    int idx = (currentIndex + i) % MAX_DATA_POINTS;
    
    // Check if this is a valid data point (non-empty timestamp)
    if (strlen(dataBuffer[idx].timestamp) > 0) {
      JsonObject dataPoint = dataArray.createNestedObject();
      dataPoint["timestamp"] = dataBuffer[idx].timestamp;
      dataPoint["ecg"] = dataBuffer[idx].ecg;
      dataPoint["piezoelectric_voltage"] = dataBuffer[idx].piezoelectric_voltage;
      dataPoint["rcwl_amplitude"] = dataBuffer[idx].rcwl_amplitude;
      
      validPoints++;
      
      // Clear the timestamp after adding to array (mark as used)
      dataBuffer[idx].timestamp[0] = '\0';
    }
  }
  
  // Only publish if we have data
  if (validPoints > 0) {
    // Serialize JSON ke string
    char jsonBuffer[MQTT_MAX_PACKET_SIZE - 100];  // Leave some headroom
    size_t jsonSize = serializeJson(doc, jsonBuffer);
    
    // Check if JSON size is too large for MQTT buffer
    if (jsonSize >= MQTT_MAX_PACKET_SIZE - 100) {
      Serial.println("Warning: JSON payload may be too large for MQTT buffer");
    }
    
    // Publish ke MQTT dengan retry
    String topic = "sleepsense/device/";
    topic += deviceId;
    topic += "/belt";
    
    bool publishSuccess = false;
    int retryCount = 0;
    
    while (!publishSuccess && retryCount < MQTT_MAX_RETRIES) {
      if (client.publish(topic.c_str(), jsonBuffer)) {
        Serial.printf("Published %d data points to MQTT\n", validPoints);
        publishSuccess = true;
      } else {
        retryCount++;
        Serial.printf("MQTT publish failed, retry %d of %d\n", retryCount, MQTT_MAX_RETRIES);
        delay(MQTT_RETRY_DELAY);
        
        // Try to reconnect if client disconnected
        if (!client.connected()) {
          Serial.println("Lost MQTT connection during publish, reconnecting...");
          reconnectMQTT();
        }
      }
    }
    
    if (!publishSuccess) {
      Serial.println("Failed to publish data to MQTT after max retries");
      return false;
    }
    
    return true;
  } else {
    Serial.println("No data to publish");
    return true;  // Not a failure case
  }
}