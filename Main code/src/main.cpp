#include <Arduino.h>        // Arduino library
#include <Wire.h>           // I2C library
#include <SPI.h>            // Serial Peripheral Interface library for communication with the SD card and GPS module
#include <SD.h>             // Library for the SD card
#include <SoftwareSerial.h> // Library for the GPS module

#include <ChainableLED.h>            // Library for the LED strip
#include <Adafruit_Sensor.h>         // Library for the BME280 sensor
#include <Adafruit_BusIO_Register.h> // Library for the BME280 sensor
#include <Adafruit_BME280.h>         // Library for the BME280 sensor
#include <TinyGPSPlus.h>             // Library for the GPS module
#include <config.h>                  // Configuration file

#define GBTN_PIN 2 // define the pin for the green button
#define RBTN_PIN 3 // define the pin for the red button

#define LED_PIN 5      // define the alimentation pin for the LED
#define LED_DATA_PIN 6 // define the data pin for the LED
#define LEDS_NUM 1     // number of LEDs in the chain

#define LUM_PIN A0  // first luminosity sensor pin
#define LUM_PIN1 A1 // second luminosity sensor pin

#define SEALEVELPRESSURE_HPA (1013.25) // define the sea level pressure

#define SD_PIN 4 // define the pin for the SD card

#define GPS_TX 7 // define the TX pin for the GPS module
#define GPS_RX 8 // define the RX pin for the GPS module

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM);

SoftwareSerial GPS(GPS_RX, GPS_TX); // initialize the pins for the GPS module

TinyGPSPlus gps; // initialize the GPS module

ISR(TIMER1_COMPA_vect) // led state update interrupt
{

}

void setup()
{
    Serial.begin(9600); // initialize the serial communication

    GPS.begin(9600); // initialize the serial communication with the GPS module

    pinMode(GBTN_PIN, INPUT); // initialize the pin for the green button

    pinMode(LUM_PIN, OUTPUT);  // initialize the pin for the luminosity sensor
    pinMode(LUM_PIN1, OUTPUT); //

    pinMode(TEMP_PIN, OUTPUT);  // initialize the pin for the temperature sensor
    pinMode(TEMP_PIN1, OUTPUT); //

    // SD.begin(SDPIN); // initialize the SD card

    // Check if the green button is pressed at startup
    if (digitalRead(GBTN_PIN) == LOW)
    {
        leds.setColorRGB(0, 0, 255, 0);

        Serial.println("\nButton is pressed\n");

        delay(1000);
    }

    SD.begin(SD_PIN); // initialize the SD card

    // Timer configuration
    noInterrupts(); // disable all interrupts
    // initialize timer1
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0;  // initialize counter value to 0

    OCR1A = 65535;           // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12);  // CTC mode
    TCCR1B |= (1 << CS12);   // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

    Serial.println("\nTest code is running\n");
    interrupts(); // enable all interrupts
}

void loop()
{

}