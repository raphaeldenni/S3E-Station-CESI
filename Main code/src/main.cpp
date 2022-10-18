#include <Arduino.h> // Arduino library
#include <SPI.h> // Serial Peripheral Interface library for communication with the SD card and GPS module
#include <SD.h>      // Library for the SD card
#include <SoftwareSerial.h> // Library for the GPS module

#include <ChainableLED.h> // Library for the LED strip
#include <Adafruit_BME280.h> // Library for the BME280 sensor
#include <TinyGPSPlus.h>     // Library for the GPS module

#define GBTN_PIN 2 // initialize the pin for the green button
#define RBTN_PIN 3 // initialize the pin for the red button

#define LED_PIN 5   // initialize the pin for the LED
#define LED_DATA_PIN 6  // initialize the pin for the LED
#define LEDS_NUM 1 // number of LEDs in the chain

#define LUM_PIN A0  // first luminosity sensor pin
#define LUM_PIN1 A1 // second luminosity sensor pin

#define TEMP_PIN A2  // first temperature sensor pin
#define TEMP_PIN1 A3 // second temperature sensor pin

//#define SDPIN 4 // initialize the pin for the SD card

#define GPS_RX 7 // initialize the pin for the GPS module
#define GPS_TX 8 // initialize the pin for the GPS module

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM);

SoftwareSerial GPS(GPS_RX, GPS_TX); // initialize the pins for the GPS module

TinyGPSPlus gps; // initialize the GPS module

int mode = 0; // initialize the variable for the LED mode

ISR(TIMER1_COMPA_vect) // led state update interrupt
{

}

void btnIntPressed() // green button interrupt
{

}

void setup()
{
    // put your setup code here, to run once:
    pinMode(GBTN_PIN, INPUT);

    pinMode(LUM_PIN, OUTPUT);
    pinMode(LUM_PIN1, OUTPUT);

    pinMode(SENSOR_SCL_PIN, OUTPUT);
    pinMode(TEMP_PIN1, OUTPUT);

    // initialiser le timer1
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0; // initialize counter value to 0

    OCR1A = 32000; // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS12); // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

    attachInterrupt(digitalPinToInterrupt(RBTN_PIN), btnIntPressed, FALLING);

    Serial.begin(9600); // initialize the serial communication

}

void loop()
{
    // put your main code here, to run repeatedly:

}