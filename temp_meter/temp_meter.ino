
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
#define I2C_SCL 10

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define PSU_PIN 12
#define TARGET_TEMP 82

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
// DallasTemperature sensors(&oneWire);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
boolean psuOn = false;
boolean tempThreasReached = false;
auto timer = timer_create_default();
bool timerSet = false;
int tempThesh = 3;
SHT31 sht;
/*
 * The setup function. We only start the sensors here
 */

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
void setupSht()
{
    Wire.begin(0x44);
    sht.begin(); // Sensor I2C Address
    Serial.println("sht Inited");
}

void setupPsu()
{
    pinMode(PSU_PIN, OUTPUT);
    digitalWrite(PSU_PIN, LOW);
}
void setup(void)
{

#ifdef ESP32
    Wire.setPins(21, 10);
#endif

    Serial.begin(115200);
    Serial.println("Filament Dryer");

    setup_display();
    setupSht();
    setupPsu();
    // start serial port
}

boolean withinThreashold(float temp)
{
    return temp >= TARGET_TEMP - tempThesh;
}

void turnOffPsu()
{
    digitalWrite(PSU_PIN, LOW);
    psuOn = false;
    Serial.println("Turned off PSU");
}
bool timerTurnOffPsu(void *arg)
{
    digitalWrite(PSU_PIN, LOW);
    psuOn = false;
    Serial.println("Turned off PSU from 3 sec timer");
    return true;
}

void turnOnPsu()
{
    if (!psuOn)
    {
        digitalWrite(PSU_PIN, HIGH);
        psuOn = true;
        Serial.println("Turned on PSU");
    }
}

float getTemp()
{
    return sht.getTemperature();
    // sensors.requestTemperatures(); // Send the command to get temperatures
    // int tempC = 0;
    // if (tempC == DEVICE_DISCONNECTED_C)
    // {
    //     return 0.0;
    // }
    // return sensors.getTempCByIndex(0);
}
boolean turnOnIfTempNotReached(void *argument)
{
    Serial.println("Inside Timer");
    float temp = getTemp();
    timerSet = false;
    if (temp >= TARGET_TEMP)
    {
        // temp reached our target. So heating element can remain off
        return false;
    }
    else
    {
        turnOnPsu();
        // schedune new timer
        timer.in(3000, timerTurnOffPsu);
    }
    return false;
}

void display_temp(float temp)
{
    // Serial.print("Temp: ");
    // Serial.println(temp);
    // sensors.requestTemperatures(); // Send the command to get temperatures
    // float rtdTemp = sensors.getTempCByIndex(0);
    display.setCursor(0, 20);
    display.print(temp);
    display.println(" C");
}

void loop(void)
{
    timer.tick();
    display.clearDisplay();

    // display_temp(30.0);
    

    // delay(2000);
    // return;
    sht.read();

    Serial.print("Temperature:");
    Serial.print(sht.getTemperature(), 1);

    float hum = sht.getHumidity();
    Serial.print(",");
    Serial.print("Humidity:");
    Serial.println(hum, 1);

    float tempC = getTemp();

    // if its within threadshold 80-8
    // turn off and wait for 15 seconds
    // if temp not reached target then turn on again for 3 second
    if (withinThreashold(tempC))
    {
        tempThesh = 5;
        // register in timer. So it can check in 5 seconds
        Serial.println("within threashold");

        if (!timerSet)
        {
            turnOffPsu();
            timer.in(15000, turnOnIfTempNotReached);
            timerSet = true;
        }
    }
    else if (!timerSet)
    {
        turnOnPsu();
    }
    display.clearDisplay();
    display_temp(tempC);
    display.print(hum);
    display.println(" %");
    display.display();

    delay(1000);
}
