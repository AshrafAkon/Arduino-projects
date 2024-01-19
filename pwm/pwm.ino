#include <Arduino.h>

const int fanPin = 5;          // PWM pin for controlling the fan
const int interruptPin = 15;    // Digital pin for the RPM interrupt
volatile unsigned long rpmCount = 0; // Variable to store the RPM count
unsigned long lastRpmCalculationTime = 0;

void rpmInterrupt() {
  rpmCount++;
}

void setup() {
  Serial.begin(115200);        // Initialize serial communication

  // Configure PWM for fan control
  ledcSetup(0, 25000, 8);       // Channel 0, 25 kHz frequency, 8-bit resolution
  pinMode(fanPin, 0);     // Attach the fan pin to channel 0

  // Configure interrupt for RPM measurement
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rpmInterrupt, FALLING);
}

void loop() {
  // Check if there's data available to read from the Serial Monitor
  if (Serial.available() > 0) {
    // Read the input from the Serial Monitor
    int fanSpeed = Serial.parseInt();

    // Print the raw input for debugging
    Serial.print("Raw Input: ");
    Serial.println(fanSpeed);

    // Ensure the fan speed is within valid range (0-100%)
    fanSpeed = constrain(fanSpeed, 0, 100);

    // Map the percentage to the PWM range (0-255)
    int pwmValue = map(fanSpeed, 0, 100, 0, 255);

    // Set the fan speed using PWM
    ledcWrite(0, pwmValue);

    // Display the current fan speed on the Serial Monitor
    Serial.print("Fan Speed: ");
    Serial.print(fanSpeed);
    Serial.println("%");
  }

  // Calculate RPM every second
  unsigned long currentMillis = millis();
  if (currentMillis - lastRpmCalculationTime >= 1000) {
    // Calculate RPM
    unsigned long rpm = (rpmCount * 60) / 2;  // Each interrupt is counted twice per revolution

    // Print the raw RPM count for debugging
    Serial.print("Raw RPM Count: ");
    Serial.println(rpmCount);

    // Reset the RPM count and update the last calculation time
    rpmCount = 0;
    lastRpmCalculationTime = currentMillis;

    // Print the calculated RPM
    Serial.print("RPM: ");
    Serial.println(rpm);
  }
}
