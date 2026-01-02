// OLED Libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Load Cell Library
#include "HX711.h"

// ===== OLED CONFIG =====
constexpr int SCREEN_WIDTH  = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_SDA = 21;
constexpr int OLED_SCL = 22;
constexpr uint8_t OLED_ADDRESS = 0x3C;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===== HX711 PINS =====
// Note: DAT/DOUT is output from HX711 to ESP32, CLK/SCK is clock from ESP32 to HX711
constexpr int HX711_DAT = 19;
constexpr int HX711_CLK = 18;

HX711 scale;

// ===== UI PINS =====
constexpr int ZERO_BUTTON_PIN = 27;  // ESP32 does NOT have GPIO24
constexpr int BUZZER_PIN      = 25;


// ===== SIMPLE BEEP HELPER =====
void beep(int ms = 80) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}

// Calibration constant
const float calFactor = 1000; // work on getting this tuned

void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize Pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ZERO_BUTTON_PIN, INPUT_PULLUP);
  

  // Start I2C
  Wire.begin(OLED_SDA, OLED_SCL);

  // Init OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 init failed");
    while (true) { delay(1000); }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Init HX711
  scale.begin(HX711_DAT, HX711_CLK);
  scale.set_gain(128);
  scale.set_scale(calFactor);

  // Startup screen
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Scale 1");
  display.display();
  beep(120);
  delay(800);

  // Tare on startup (make sure nothing is on the scale)
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Taring...");
  display.display();

  scale.tare();
  beep(200);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ready.");
  display.display();
  delay(300);
  
}

void loop() {
  // Button is INPUT_PULLUP => pressed = LOW
  if (digitalRead(ZERO_BUTTON_PIN) == LOW) {
    // Basic debounce
    delay(25);
    if (digitalRead(ZERO_BUTTON_PIN) == LOW) {
      scale.tare();
      beep(150);

      // Wait for release so it doesn't spam tare
      while (digitalRead(ZERO_BUTTON_PIN) == LOW) {
        delay(10);
      }
    }
  }

  long raw = 0;
  long val = 0;
  if (scale.is_ready()) {
    raw = scale.read();
    val = scale.get_value();
  }

  // Update OLED
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Raw:");

  display.setTextSize(2);
  display.setCursor(0, 16);
  display.println(raw);

  display.setTextSize(1);
  display.setCursor(0, 48);
  display.println(val);

  display.display();

  delay(100);
}
