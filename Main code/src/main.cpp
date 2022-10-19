#include <Arduino.h>        // Arduino library
#include <Wire.h>           // I2C library
#include <SPI.h>            // Serial Peripheral Interface library for communication with the SD card and GPS module
#include <SD.h>             // Library for the SD card
#include <SoftwareSerial.h> // Library for the GPS module

#include <ChainableLED.h>       // Library for the LED strip
#include <Adafruit_Sensor.h>    // Library for the BME280 sensor
#include <Adafruit_I2CDevice.h> // Library for the BME280 sensor
#include <Adafruit_BME280.h>    // Library for the BME280 sensor
#include <TinyGPSPlus.h>        // Library for the GPS module
#include "DS1307.h"             // Library for the RTC module

#include <config.h>       // Configuration file
#include <struct.h> // Structure file

#define GBTN_PIN 2 // define the pin for the green button
#define RBTN_PIN 3 // define the pin for the red button

#define LED_PIN 5       // define the alimentation pin for the LED
#define LED_DATA_PIN 6  // define the data pin for the LED
#define LEDS_NUM 1      // number of LEDs in the chain
#define LED_UPDATE_INTERVAL 31250 // define the time between each LED update 62500 = 1s, Value is between 0 and 62500

#define LUM_DATA_PIN A0   // first luminosity sensor pin
#define SECOND_LUM_PIN A1 // second luminosity sensor pin

#define BME_ADDRESS 0x76             // define the BME280 sensor address
#define REFRESH_DELAY 1500           // define the delay between each refresh of the data
#define SEALEVELPRESSURE_HPA 1024.90 // define the sea level pressure

#define SD_PIN 4 // define the pin for the SD card

#define TX_TO_GPS 8   // define the TX pin for the GPS module
#define RX_TO_GPS 7   // define the RX pin for the GPS module
#define GPS_BAUD 9600 // define the baud rate for the GPS module

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM);

Adafruit_BME280 bme; // initialize I2C communication for the bme280 sensor

SoftwareSerial ss(RX_TO_GPS, TX_TO_GPS); // initialize the pins for the GPS module

TinyGPSPlus gps; // initialize the GPS module

DS1307 clock; // initialize the RTC module

struct config config; // initialize the config structure

struct modeVar modeVar; // initialize the mode structure

struct sensorsData sensorsData; // initialize the sensors data structure

struct gpsData gpsData; // initialize the GPS data structure

struct rtcData rtcData; // initialize the RTC data structure

ISR(TIMER1_COMPA_vect) // check if button is pressed
{
    static float state = 0;
    // LED update
    switch (modeVar.ledMode)
    {
    case (ERROR_CAPTOR_ACCESS, ERROR_GPS, ERROR_CLOCK_ACCESS, ERROR_SD_FULL):
        if (state <= 0)
        {
            leds.setColorRGB(RED);
            state += (62500 / LED_UPDATE_INTERVAL);
        }
        if (state >= 1)// 1s
        {
            if (ERROR_CAPTOR_ACCESS) leds.setColorRGB(GREEN);
            if (ERROR_GPS) leds.setColorRGB(YELLOW);
            if (ERROR_CLOCK_ACCESS) leds.setColorRGB(BLUE);
            if (ERROR_SD_FULL) leds.setColorRGB(WHITE);
            state -= (LED_UPDATE_INTERVAL/62500);
        }
        break;
    case (ERROR_SD_WRITE, ERROR_DATA_INCOHERENCE):
        if (state <= 0)
        {
            leds.setColorRGB(RED);
            state += (LED_UPDATE_INTERVAL/62500);
        }
        if (state >= 1)
        {
            if (ERROR_SD_WRITE) leds.setColorRGB(WHITE);
            if (ERROR_DATA_INCOHERENCE) leds.setColorRGB(GREEN);
            state -= ((LED_UPDATE_INTERVAL / 2)/62500);
        }
        break;
    default:
        if (modeVar.ledMode == MAINTENANCE) leds.setColorRGB(ORANGE);
        if (modeVar.ledMode == ECONOMY) leds.setColorRGB(BLUE);
        if (modeVar.ledMode == STANDARD) leds.setColorRGB(GREEN);
        break;
    }
    // Button check
    if (digitalRead(RBTN_PIN) == LOW)
    {
        modeVar.rBtntimePressed++;
        if (modeVar.rBtntimePressed > (62500 * config.timeToSwitch / LED_UPDATE_INTERVAL))
        {
            modeVar.rBtntimePressed = 0;
            if (modeVar.actual != MAINTENANCE)
            {
                modeVar.previous = modeVar.actual;
                modeVar.actual = MAINTENANCE;
                Serial.println("ENTER MAINTENANCE MODE");
            }
            else
            {
                modeVar.actual = modeVar.previous;
                modeVar.previous = MAINTENANCE;
                Serial.println("EXIT MAINTENANCE MODE");
            }
        }
    }
    else if (digitalRead(GBTN_PIN) == LOW) // priority to the red button
    {
        modeVar.gBtntimePressed++;
        if (modeVar.gBtntimePressed > (62500 * config.timeToSwitch / LED_UPDATE_INTERVAL))
        {
            modeVar.gBtntimePressed = 0;
            if (modeVar.actual == STANDARD)
            {
                modeVar.previous = modeVar.actual;
                modeVar.actual = ECONOMY;
                Serial.println("ENTER ECONOMY MODE");
            }
            else if (modeVar.actual == ECONOMY)
            {
                modeVar.previous = modeVar.actual;
                modeVar.actual = STANDARD;
                Serial.println("ENTER STANDARD MODE");
            }
        }
    }
    else
    {
        modeVar.rBtntimePressed = 0;
        modeVar.gBtntimePressed = 0;
    }
}

void configMode()
{
    Serial.println("ENTER CONFIGURATION MODE");
    Serial.println("Enter help() to show the list of commands.");
    Serial.println("Enter exit to show the list of commands.");
}

void getData()
{
    
}

void setup()
{
    pinMode(RBTN_PIN, INPUT); // define the red button pin as an input
    pinMode(GBTN_PIN, INPUT); // define the green button pin as an input
    Serial.begin(9600);       // initialize the serial communication
    if (digitalRead(GBTN_PIN) == LOW)
        configMode(); // if the green button is pressed at startup, enter configuration mode
    // Timer configuration
    noInterrupts(); // disable all interrupts
    // initialize timer1
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0;  // initialize counter value to 0

    OCR1A = LED_UPDATE_INTERVAL; // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12);      // CTC mode
    TCCR1B |= (1 << CS12);       // 256 prescaler
    TIMSK1 |= (1 << OCIE1A);     // enable timer compare interrupt

    interrupts(); // enable all interrupts
}

void loop()
{
    // Get DATA
    // Store DATA
    delay(5000);
    modeVar.ledMode++;
}