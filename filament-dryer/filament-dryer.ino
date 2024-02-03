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
// Include the Arduino Stepper Library
// #include <Stepper.h>
#include <Unistep2.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define I2C_SDA 21
#define I2C_SCL 21

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define mosfetPwmPin 44

#define buttonUpPin 2 // Pin number for the button to
#define buttonDownPin 3
#define interruptPin 15
#define fanPin 5 // PWM pin for

const int stepDelay = 1800;
const float pwmFrequency = 266.00 * 100;
volatile unsigned long rpmCount = 0; // Variable to store the
unsigned long lastRpmCalculationTime = 0;
unsigned long lastDebounceTime = 0; // the last time the
unsigned long debounceDelay = 70;   // the debounce time; increase if the output flickers
int lastButtonState = LOW;          // the previous reading
int buttonState = LOW;
int pwmRate = 0;
char dashLine[] = "=====================================================================================";
boolean psuOn = false;
boolean tempThreasReached = false;
auto timer = timer_create_default();
bool timerSet = false;
int tempThesh = 3;

// Devices
SHT31 sht;
AVR_PWM *PWM_Instance;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Unistep2 stepper(8, 9, 10, 11, 4096, stepDelay);

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
    // display.setRotation(2);

    display.setTextSize(2);
    Serial.println("Display Inited");
    display.setTextColor(WHITE);
}
void setupSht()
{
    Wire.begin(0x44);
    sht.begin(); // Sensor I2C Address
    Serial.println("sht Inited");
}

void printPWMInfo(AVR_PWM *PWM_Instance)
{
    Serial.println(dashLine);
    Serial.print("Actual data: pin = ");
    Serial.print(PWM_Instance->getPin());
    Serial.print(", PWM DC = ");
    Serial.print(PWM_Instance->getActualDutyCycle());
    Serial.print(", PWMPeriod = ");
    Serial.print(PWM_Instance->getPWMPeriod());
    Serial.print(", PWM Freq (Hz) = ");
    Serial.println(PWM_Instance->getActualFreq(), 4);
    Serial.println(dashLine);
}
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

    // Serial.println(reading);
    // Serial.print("last debounce time: ");
    // Serial.println(lastDebounceTime);
    // Serial.print("passedsince: ");
    // Serial.println(millis() - lastDebounceTime);
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

    // Serial.println(reading);
    // Serial.print("last debounce time: ");
    // Serial.println(downLast);
    // Serial.print("passedsince: ");
    // Serial.println(millis() - downLast);
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
unsigned long lastTempCheck = 0;
float hum = 0.0;

float tempC = 0.0;
void checkTemp()
{
    if (millis() - lastTempCheck <= 1000)
    {
        return;
    }
    sht.read();
    hum = sht.getHumidity();

    tempC = sht.getTemperature();
    lastTempCheck = millis();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(tempC);
    display.println(" C");
    display.print(hum);
    display.println(" %");
    display.print("Set ");
    display.println(pwmRate);
    display.display();
}
void loop()
{
    stepper.run();
    int oldPwmRate = pwmRate;
    upPwm();
    downPwm();

    checkTemp();

    if (oldPwmRate != pwmRate)
    {
        PWM_Instance->setPWM(mosfetPwmPin, pwmFrequency,
                             float(pwmRate));

        // if (pwmRate == 100)
        // {
        //     analogWrite(mosfetPwmPin, 255);
        // }
        Serial.println((pwmRate * 65536) / 100);
        printPWMInfo(PWM_Instance);
    }

    // analogWrite(mosfetPwmPin, pwmRate);
    //  display.println(" C");

    // // Slow - 4-step CW sequence to o.

    if (stepper.stepsToGo() == 0)
    {
        // stepper.stop();

        // delay(5000);
        stepper.move(100000);
    }
}
