#include <Arduino.h> // Arduino library
#include <SPI.h>     // Serial Peripheral Interface library for communication with the SD card and GPS module
#include <SD.h>      // Library for the SD card

#include <ChainableLED.h>    // Library for the LED strip
#include <Adafruit_BME280.h> // Library for the BME280 sensor

#define GBTN_PIN 2 // initialize the pin for the green button
#define RBTN_PIN 3 // initialize the pin for the red button

#define LED_PIN 5   // initialize the pin for the LED
#define LED_PIN1 6  // initialize the pin for the LED
#define LEDS_NUM 1 // number of LEDs in the chain

#define LUM_PIN A0  // first luminosity sensor pin
#define LUM_PIN1 A1 // second luminosity sensor pin

#define TEMP_PIN A2  // first temperature sensor pin
#define TEMP_PIN1 A3 // second temperature sensor pin

#define SDPIN 4 // initialize the pin for the SD card

ChainableLED leds(LED_PIN, LED_PIN1, LEDS_NUM);

int ledmode = 0; // initialize the variable for the LED mode

ISR(TIMER1_COMPA_vect) // led state update interrupt
{
    // led mode 0: off
    if (ledmode == 0)
    {
        leds.setColorRGB(0, 0, 0, 0);
    }
    // led mode 1: green
    else if (ledmode == 1)
    {
        leds.setColorRGB(0, 0, 255, 0);
    }
    // led mode 2: red
    else if (ledmode == 2)
    {
        leds.setColorRGB(0, 255, 0, 0);
    }
    // led mode 3: yellow
    else if (ledmode == 3)
    {
        leds.setColorRGB(0, 255, 255, 0);
    }
    // led mode 4: blue
    else if (ledmode == 4)
    {
        leds.setColorRGB(0, 0, 0, 255);
    }
}

void btnIntPressed()
{
    leds.setColorRGB(255, 0, 0, 0);

    Serial.println("\nButton interrupt is pressed\n");
}

void checkSensors()
{
    // Check Luminosity sensor pins
    Serial.println("LUM : " + String(analogRead(LUM_PIN)));
    Serial.println("LUM1 : " + String(analogRead(LUM_PIN1)));

    delay(1000);

    // Check Temperature sensor pins
    Serial.println("TEMP : " + String(analogRead(TEMP_PIN)));
    Serial.println("TEMP1 : " + String(analogRead(TEMP_PIN1)));

    delay(1000);
}

void checkSD()
{
    // Check SD card
    File testFile = SD.open("testlog.txt", FILE_WRITE);

    if (testFile)
    {
        testFile.println("test");
        testFile.close();
        Serial.println("\nSD card is working\n");
    }
    else
    {
        Serial.println("\nError opening testlog.txt\n");
    }

    delay(1000);
}

void setup()
{
    Serial.begin(9600); // initialize the serial communication

    pinMode(GBTN_PIN, INPUT);

    pinMode(LUM_PIN, OUTPUT);
    pinMode(LUM_PIN1, OUTPUT);

    pinMode(TEMP_PIN, OUTPUT);
    pinMode(TEMP_PIN1, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(RBTN_PIN), btnIntPressed, LOW); // attach interrupt to the green button

    SD.begin(SDPIN); // initialize the SD card

    // Check if the green button is pressed at startup
    if (digitalRead(GBTN_PIN) == LOW)
    {
        leds.setColorRGB(0, 0, 255, 0);

        Serial.println("\nButton is pressed\n");

        delay(1000);
    }

    // initialize timer1
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0;  // initialize counter value to 0

    OCR1A = 32000;           // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12);  // CTC mode
    TCCR1B |= (1 << CS12);   // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

    Serial.println("\nTest code is running\n");

}

void loop()
{
    checkSensors();

    checkSD();
}
