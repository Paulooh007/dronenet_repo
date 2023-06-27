#include <HardwareSerial.h>

// Configure DAC pins
const int DAC1_PIN = 25;
const int DAC2_PIN = 26;

// Set voltage range
const float VOLT_RANGE = 3.3;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Configure DAC pins
  pinMode(DAC1_PIN, OUTPUT);
  pinMode(DAC2_PIN, OUTPUT);
}

void loop() {
  // Wait for serial input
  while (Serial.available() < 1) {
    delay(10);
  }

  // Read serial input
  String input_str = Serial.readStringUntil('\n');
  input_str.trim();
  int comma_pos = input_str.indexOf(",");
  if (comma_pos < 0) {
    return;
  }

  String value1_str = input_str.substring(0, comma_pos);
  String value2_str = input_str.substring(comma_pos + 1);
  float value1 = value1_str.toFloat();
  float value2 = value2_str.toFloat();

  // Convert floats to voltage
  float voltage1 = (value1 / 255.0) * VOLT_RANGE;
  float voltage2 = (value2 / 255.0) * VOLT_RANGE;

  // Output voltages to DAC pins
  dacWrite(DAC1_PIN, (int)((voltage1 / VOLT_RANGE) * 255));
  dacWrite(DAC2_PIN, (int)((voltage2 / VOLT_RANGE) * 255));

  // Print values for debugging
  Serial.print("Voltage 1: ");
  Serial.println(voltage1);
  Serial.print("Voltage 2: ");
  Serial.println(voltage2);
}
