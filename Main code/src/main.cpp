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

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM);

ISR(TIMER1_COMPA_vect) // check if button is pressed
{

}

void setup()
{
    Serial.begin(9600); // initialize the serial communication
    // Check if the green button is pressed at startup
    if (digitalRead(GBTN_PIN) == LOW)
    {
        leds.setColorRGB(0, 0, 255, 0);

        Serial.println("\nButton is pressed\n");

        delay(1000);
    }
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

    interrupts(); // enable all interrupts
}

void loop()
{
}