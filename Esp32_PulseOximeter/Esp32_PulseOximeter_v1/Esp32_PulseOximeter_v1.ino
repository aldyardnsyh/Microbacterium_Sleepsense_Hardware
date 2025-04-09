#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAX30105
MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
long lastUpdate = 0;
const long TIMEOUT = 5000;

#define FINGER_THRESHOLD 5000

bool fingerDetected = false;
bool showWelcome = true;
unsigned long welcomeTime = 0;
const long WELCOME_DURATION = 3000;

// Debug variables
unsigned long lastDebugPrint = 0;

// SpO2 Variables - Buffer lebih besar untuk perhitungan yang lebih akurat
#define SPO2_BUFFER_SIZE 25
uint32_t redBuffer[SPO2_BUFFER_SIZE];
uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint8_t bufferIndex = 0;
int spo2 = 0;
bool bufferFilled = false;

// Kalman filter untuk smoothing nilai SpO2
float kalmanSpO2 = 95.0;
float kalmanCovariance = 1.0;
const float measurementNoise = 4.0;  // Noise dari pengukuran
const float processNoise = 0.01;     // Noise dari proses

// Heart icon
const uint8_t heartIcon[] = {
  0b00011000, 0b00111100, 0b01111110, 0b11111111,
  0b11111111, 0b01111110, 0b00111100, 0b00011000
};

void displayWelcomeScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(5, 5);
  display.println("Pulse");
  display.setCursor(5, 30);
  display.println("Oximeter");
  
  if (millis() % 1000 < 500) {
    display.drawBitmap(100, 20, heartIcon, 8, 8, SSD1306_WHITE);
  }
  
  display.display();
}

void displayFingerPrompt() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 5);
  display.println("Please put your");
  display.setCursor(0, 20);
  display.println("finger on sensor");
  
  int arrowPos = (millis() / 300) % 3;
  display.setCursor(20 + (arrowPos * 10), 40);
  display.println("v v v");

  // nilai IR untuk debugging
  display.setCursor(0, 55);
  display.print("IR: ");
  display.print(particleSensor.getIR());
  
  display.display();
}

void displayReadings(int beatAvg, int spo2, long ir, long red, float ratio) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // BPM dengan ikon jantung
  display.setCursor(0, 5);
  display.print("BPM: ");
  display.setTextSize(2);
  display.print(beatAvg);
  display.setTextSize(1);
  
  if (millis() - lastBeat < 200) {
    display.fillRect(100, 5, 8, 8, SSD1306_WHITE);
  } else {
    display.drawBitmap(100, 5, heartIcon, 8, 8, SSD1306_WHITE);
  }
  
  // SpO2
  display.setCursor(0, 25);
  display.print("SpO2: ");
  display.setTextSize(2);
  display.print(spo2);
  display.setTextSize(1);
  display.print("%");
  
  // Nilai debug untuk ratio, IR, dan Red
  display.setCursor(0, 45);
  display.print("R:");
  display.print(ratio, 2);
  
  display.setCursor(0, 55);
  display.print("IR:");
  display.print(ir);
  display.print(" R:");
  display.print(red);
  
  display.display();
}

// Fungsi untuk menghitung SpO2 dengan normalisasi yang lebih baik
float calculateRatio(uint32_t* redBuffer, uint32_t* irBuffer, int bufferSize) {
  // Nilai min dan max untuk normalisasi dan perhitungan AC
  uint32_t redMax = 0, redMin = UINT32_MAX;
  uint32_t irMax = 0, irMin = UINT32_MAX;
  float redDC = 0, irDC = 0;
  
  // nilai min, max, dan rata-rata
  for (int i = 0; i < bufferSize; i++) {
    // Red
    if (redBuffer[i] > redMax) redMax = redBuffer[i];
    if (redBuffer[i] < redMin) redMin = redBuffer[i];
    redDC += redBuffer[i];
    
    // IR
    if (irBuffer[i] > irMax) irMax = irBuffer[i];
    if (irBuffer[i] < irMin) irMin = irBuffer[i];
    irDC += irBuffer[i];
  }
  
  // Hitung nilai DC (rata-rata)
  redDC /= bufferSize;
  irDC /= bufferSize;
  
  // Hitung amplitudo AC peak-to-peak
  float redAC = (float)(redMax - redMin);
  float irAC = (float)(irMax - irMin);
  
  // Hitung rasio dengan normalisasi
  float acDcRed = redAC / redDC;
  float acDcIr = irAC / irDC;
  
  // Hindari pembagian dengan nol
  if (acDcIr == 0) return 0;
  
  // Hitung rasio R
  float ratio = acDcRed / acDcIr;
  
  return ratio;
}

// Konversi ratio ke nilai SpO2 menggunakan kurva empiris yang disesuaikan
int ratioToSpO2(float ratio) {
  // Formula kurva empiris:
  // SpO2 = -45.060 * R² + 30.354 * R + 94.845
  // Dengan adjustment untuk kalibrasi sensor
  
  float adjustedRatio = ratio;
  
  // Cegah rasio ekstrim yang menyebabkan hasil yang tidak masuk akal
  if (ratio < 0.4) adjustedRatio = 0.4;
  if (ratio > 1.5) adjustedRatio = 1.5;
  
  // Kurva kuadratik untuk hasil yang lebih akurat
  float spO2Value = -45.060 * adjustedRatio * adjustedRatio + 30.354 * adjustedRatio + 94.845;
  
  // Kurva alternatif jika hasilnya masih terlalu rendah:
  // float spO2Value = 104 - 17 * adjustedRatio;
  
  // Batasi nilai
  if (spO2Value > 100) spO2Value = 100;
  if (spO2Value < 70) spO2Value = 0; // Nilai di bawah 70 biasanya tidak valid
  
  return (int)spO2Value;
}

// Kalman filter untuk menghaluskan nilai SpO2
int kalmanFilter(int measurement) {
  // Predict
  float kalmanPrediction = kalmanSpO2;
  kalmanCovariance = kalmanCovariance + processNoise;
  
  // Update
  float kalmanGain = kalmanCovariance / (kalmanCovariance + measurementNoise);
  kalmanSpO2 = kalmanPrediction + kalmanGain * (measurement - kalmanPrediction);
  kalmanCovariance = (1 - kalmanGain) * kalmanCovariance;
  
  return (int)kalmanSpO2;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Memulai Pulse Oximeter");
  
  Wire.begin();

  // OLED Initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed!");
    while (1);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();
  
  welcomeTime = millis();
  showWelcome = true;
  
  // Sensor Initialization with Debug
  Serial.println("Initializing MAX30105...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Sensor not found!");
    display.println("Check connections");
    display.display();
    while (1);
  }
  Serial.println("MAX30105 found!");
  
  // Konfigurasi optimal untuk SpO2
  byte ledBrightness = 60;    // Optimal brightness for SpO2
  byte sampleAverage = 8;     // Increased sample averaging for stable signals
  byte ledMode = 2;           // Mode 2 = Red + IR
  byte sampleRate = 100;      // 100Hz is good balance between resolution and processing power
  int pulseWidth = 411;       // Maximum pulse width for better resolution
  int adcRange = 4096;        // Full ADC range for maximum sensitivity
  
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x2F);    // Higher red LED current 
  particleSensor.setPulseAmplitudeIR(0x2F);     // Higher IR LED current
  particleSensor.setPulseAmplitudeGreen(0);
  
  // Baca nilai sensor awal untuk debug
  Serial.print("Initial IR value: ");
  Serial.println(particleSensor.getIR());
  
  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
  
  Serial.println("Setup completed");
}

void loop() {
  // Tampilkan welcome screen
  if (showWelcome) {
    displayWelcomeScreen();
    if (millis() - welcomeTime > WELCOME_DURATION) {
      showWelcome = false;
    }
    delay(50);
    return;
  }

  // Baca nilai sensor
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // Debug print setiap detik
  if (millis() - lastDebugPrint > 1000) {
    Serial.print("IR Value: ");
    Serial.print(irValue);
    Serial.print(", Red Value: ");
    Serial.print(redValue);
    Serial.print(", SpO2: ");
    Serial.println(spo2);
    lastDebugPrint = millis();
  }

  // Deteksi jari dengan threshold
  bool currentFingerDetected = (irValue > FINGER_THRESHOLD);
  
  if (fingerDetected != currentFingerDetected) {
    fingerDetected = currentFingerDetected;
    Serial.print("Finger status changed: ");
    Serial.println(fingerDetected ? "Detected" : "Removed");
    
    if (!fingerDetected) {
      // Reset data
      for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
      beatAvg = 0;
      spo2 = 0;
      bufferFilled = false;
      bufferIndex = 0;
      kalmanSpO2 = 95.0;  // Reset Kalman filter
      kalmanCovariance = 1.0;
    }
  }

  if (!fingerDetected) {
    displayFingerPrompt();
  } else {
    lastUpdate = millis();

    // BPM calculation
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

    // SpO2 calculation with improved algorithm
    if (irValue > FINGER_THRESHOLD && redValue > 0) {
      // Store to buffer
      redBuffer[bufferIndex] = redValue;
      irBuffer[bufferIndex] = irValue;
      bufferIndex++;
      
      if (bufferIndex >= SPO2_BUFFER_SIZE) {
        bufferIndex = 0;
        bufferFilled = true;
      }
      
      float ratio = 0;
      if (bufferFilled) {
        // Hitung ratio R
        ratio = calculateRatio(redBuffer, irBuffer, SPO2_BUFFER_SIZE);
        
        // Konversi ratio ke SpO2
        int rawSpO2 = ratioToSpO2(ratio);
        
        // Terapkan filter Kalman untuk smooth output
        spo2 = kalmanFilter(rawSpO2);
      }
      
      displayReadings(beatAvg, spo2, irValue, redValue, ratio);
    }
  }

  delay(50);
}
