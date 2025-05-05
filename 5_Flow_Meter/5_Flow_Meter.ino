/*
 * ESP32 Multiple Flow Sensor Interface with I2C LCD Display
 * - 4x YF-S201 flow sensors
 * - 1x FS400A flow sensor
 * - 20x4 I2C LCD display
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 20, 4);  // I2C address 0x27, 20 characters, 4 rows

// Flow Sensor Pin Configuration
const int YF_S201_PINS[4] = { 12, 13, 14, 26 };  // GPIO pins for YF-S201 sensors
const int FS400A_PIN = 27;                       // GPIO pin for FS400A sensor

// Flow Sensor Calibration Factors
// YF-S201: Typical factor is 7.5 (pulses per liter)
const float YF_S201_FACTOR = 7.5;
// FS400A: Typical factor is 3.5 (pulses per liter)
const float FS400A_FACTOR = 3.5;

// Variables to store flow data
volatile int flowPulses[5] = { 0, 0, 0, 0, 0 };
float flowRate[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };  // in liters per minute
unsigned long oldTime = 0;

// Interrupt Service Routines for each flow sensor
void IRAM_ATTR flowISR0() {
  flowPulses[0]++;
}
void IRAM_ATTR flowISR1() {
  flowPulses[1]++;
}
void IRAM_ATTR flowISR2() {
  flowPulses[2]++;
}
void IRAM_ATTR flowISR3() {
  flowPulses[3]++;
}
void IRAM_ATTR flowISR4() {
  flowPulses[4]++;
}

void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Flow Sensor System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // Initialize YF-S201 flow sensor pins
  for (int i = 0; i < 4; i++) {
    pinMode(YF_S201_PINS[i], INPUT_PULLUP);
  }

  // Initialize FS400A flow sensor pin
  pinMode(FS400A_PIN, INPUT_PULLUP);

  // Attach interrupts to flow sensor pins
  attachInterrupt(digitalPinToInterrupt(YF_S201_PINS[0]), flowISR0, FALLING);
  attachInterrupt(digitalPinToInterrupt(YF_S201_PINS[1]), flowISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(YF_S201_PINS[2]), flowISR2, FALLING);
  attachInterrupt(digitalPinToInterrupt(YF_S201_PINS[3]), flowISR3, FALLING);
  attachInterrupt(digitalPinToInterrupt(FS400A_PIN), flowISR4, FALLING);

  delay(1000);
  lcd.clear();

  // Display header
  lcd.setCursor(0, 0);
  lcd.print("Flow Rate (L/min)");
  oldTime = millis();
}

void loop() {
  // Calculate flow rate every second
  if ((millis() - oldTime) > 1000) {  // Update every second
    // Disable interrupts temporarily while calculating
    detachInterrupt(digitalPinToInterrupt(YF_S201_PINS[0]));
    detachInterrupt(digitalPinToInterrupt(YF_S201_PINS[1]));
    detachInterrupt(digitalPinToInterrupt(YF_S201_PINS[2]));
    detachInterrupt(digitalPinToInterrupt(YF_S201_PINS[3]));
    detachInterrupt(digitalPinToInterrupt(FS400A_PIN));

    // Calculate time elapsed in seconds
    float timeElapsed = (millis() - oldTime) / 1000.0;

    // Calculate flow rates for YF-S201 sensors
    for (int i = 0; i < 4; i++) {
      // Convert pulses to liters per minute
      flowRate[i] = (flowPulses[i] / YF_S201_FACTOR / timeElapsed) * 60;
      flowPulses[i] = 0;  // Reset pulse counter
    }

    // Calculate flow rate for FS400A sensor
    flowRate[4] = (flowPulses[4] / FS400A_FACTOR / timeElapsed) * 60;
    flowPulses[4] = 0;  // Reset pulse counter

    // Update display


    // Re-enable interrupts
    attachInterrupt(digitalPinToInterrupt(YF_S201_PINS[0]), flowISR0, FALLING);
    attachInterrupt(digitalPinToInterrupt(YF_S201_PINS[1]), flowISR1, FALLING);
    attachInterrupt(digitalPinToInterrupt(YF_S201_PINS[2]), flowISR2, FALLING);
    attachInterrupt(digitalPinToInterrupt(YF_S201_PINS[3]), flowISR3, FALLING);
    attachInterrupt(digitalPinToInterrupt(FS400A_PIN), flowISR4, FALLING);

    oldTime = millis();  // Reset timer
  }
  updateDisplay();
}

// Function to update LCD display with flow rates
void updateDisplay() {
  // Update YF-S201 flow rates (Line 1-2)
  lcd.setCursor(0, 1);
  lcd.print("YF1:");
  lcd.print(flowRate[4], 1);
  lcd.print(" YF2:");
  lcd.print(flowRate[0], 1);

  lcd.setCursor(0, 2);
  lcd.print("YF3:");
  lcd.print(flowRate[1], 1);
  lcd.print(" YF4:");
  lcd.print(flowRate[2], 1);

  // Update FS400A flow rate (Line 3)
  lcd.setCursor(0, 3);
  lcd.print("YF4: ");
  lcd.print(flowRate[3], 1);
  lcd.print(" L/min");

  // Also output to serial for debugging
  Serial.println("Flow Rates (L/min):");
  for (int i = 0; i < 4; i++) {
    Serial.print("YF-S201 #");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(flowRate[i]);
  }
  Serial.print("FS400A: ");
  Serial.println(flowRate[4]);
  Serial.println();
}