#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
long lastUpdate = 0;
const long TIMEOUT = 10000; // 10 detik timeout

// Variabel untuk SpO2
float ratioRMSRed = 0;
float ratioRMSIR = 0;
int spo2 = 0;

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

void setup() {
  Serial.begin(115200);
  Wire.begin();

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
  display.display();
  delay(2000);

  // Inisialisasi MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);  // LED Merah rendah
  particleSensor.setPulseAmplitudeGreen(0);   // LED Hijau mati

  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
  beatAvg = 0;
}

void loop() {
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();
  display.clearDisplay();

  if (irValue < 50000) {
    if (millis() - lastUpdate > TIMEOUT) {
      display.setCursor(0, 0);
      display.println("Pulse Oximeter");
      display.setCursor(0, 20);
      display.println("Place finger...");
      display.display();
    }
  } else {
    lastUpdate = millis();

    // Hitung BPM
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

    // Hitung SpO2 (estimasi sederhana)
    if (irValue > 50000 && redValue > 0) {
      float ratio = (float)redValue / (float)irValue;
      // Rumus sederhana: SpO2 = -45.060 * ratio + 100 (perlu kalibrasi)
      spo2 = (int)(-45.060 * ratio + 100);
      if (spo2 > 100) spo2 = 100;
      if (spo2 < 0) spo2 = 0;
    }

    // Tampilan OLED
    display.setCursor(0, 0);
    display.print("BPM: ");
    display.print(beatAvg);
    display.setCursor(0, 20);
    display.print("SpO2: ");
    display.print(spo2);
    display.print("%");
    display.drawBitmap(100, 0, heartIcon, 8, 8, SSD1306_WHITE); // Animasi detak
    display.display();
  }

  delay(10);
}