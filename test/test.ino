#include <AVR_PWM.h>
#include <PWM_Generic_Debug.h>

#include <Arduino.h>

#include <SHT31.h>
#include "Wire.h"
#include <arduino-timer.h>
#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <OneWire.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define I2C_SDA 21
#define I2C_SCL 21

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define mosfetPwmPin 11
#define pwmFrequency 25000
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*
 * The setup function. We only start the sensors here
 */
const int buttonUpPin = 2; // Pin number for the button to increase PWM
const int buttonDownPin = 3;

const int fanPin = 5;                // PWM pin for controlling the fan
const int interruptPin = 15;         // Digital pin for the RPM interrupt
volatile unsigned long rpmCount = 0; // Variable to store the RPM count
unsigned long lastRpmCalculationTime = 0;
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50;   // the debounce time; increase if the output flickers
int lastButtonState = LOW;          // the previous reading from the input pin
int buttonState = LOW;
int pwmRate = 0;

void rpmInterrupt()
{
    rpmCount++;
}
void setup_display()
{

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();

    display.setTextSize(2);
    Serial.println("Display Inited");
    display.setTextColor(WHITE);
}
AVR_PWM *PWM_Instance;

void setup()
{
    Serial.begin(115200); // Initialize serial communication
    setup_display();
    pinMode(fanPin, OUTPUT);
    pinMode(mosfetPwmPin, OUTPUT);
    pinMode(buttonUpPin, INPUT);
    pinMode(buttonDownPin, INPUT);
    analogWrite(fanPin, 20);
    PWM_Instance = new AVR_PWM(mosfetPwmPin, pwmFrequency, 0);
    if (PWM_Instance)
    {
        PWM_Instance->setPWM();
    }
}
void upPwm()
{
    int reading = digitalRead(buttonUpPin);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState)
    {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    Serial.println(reading);
    Serial.print("last debounce time: ");
    Serial.println(lastDebounceTime);
    Serial.print("passedsince: ");
    Serial.println(millis() - lastDebounceTime);
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        // whatever the reading is at, it's been there for longer
        // than the debounce delay, so take it as the actual current state:
        buttonState = reading;
        // is the button still high ?
        if ((reading == HIGH) && pwmRate < 100)
        {
            pwmRate++;
            lastDebounceTime = millis();
        }
    }

    // set the LED using the state of the button:
    // digitalWrite(ledPin, buttonState);

    // save the reading.  Next time through the loop,
    // it'll be the lastButtonState:
    lastButtonState = reading;
}
unsigned long downLast;
int lastBtnState;
void downPwm()
{
    int reading = digitalRead(buttonDownPin);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastBtnState)
    {
        // reset the debouncing timer
        downLast = millis();
    }

    Serial.println(reading);
    Serial.print("last debounce time: ");
    Serial.println(downLast);
    Serial.print("passedsince: ");
    Serial.println(millis() - downLast);
    if ((millis() - downLast) > debounceDelay)
    {
        // whatever the reading is at, it's been there for longer
        // than the debounce delay, so take it as the actual current state:
        buttonState = reading;
        // is the button still high ?
        if ((reading == HIGH) && pwmRate > 0)
        {
            pwmRate--;
            downLast = millis();
        }
    }

    // set the LED using the state of the button:
    // digitalWrite(ledPin, buttonState);

    // save the reading.  Next time through the loop,
    // it'll be the lastButtonState:
    lastBtnState = reading;
}
void loop()
{
    int oldPwmRate = pwmRate;
    upPwm();
    downPwm();
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println(pwmRate);
    display.display();
    if (oldPwmRate != pwmRate)
    {
        PWM_Instance->setPWM(mosfetPwmPin, pwmFrequency, pwmRate);
    }

    // analogWrite(mosfetPwmPin, pwmRate);
    //  display.println(" C");
}
