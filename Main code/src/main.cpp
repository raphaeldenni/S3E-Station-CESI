#include <Arduino.h> // Arduino library
#include <SPI.h> // Serial Peripheral Interface library for communication with the SD card and GPS module

#include <ChainableLED.h> // Library for the LED strip
#include <SD.h> // Library for the SD card
#include <Adafruit_BME280.h> // Library for the BME280 sensor

#define GBTN_PIN 2 // initialize the pin for the green button
#define RBTN_PIN 3 // initialize the pin for the red button

#define LED_PIN 4   // initialize the pin for the LED
#define LED_PIN1 5  // initialize the pin for the LED
#define LEDS_NUM 1 // number of LEDs in the chain

#define LUM_PIN A0  // first luminosity sensor pin
#define LUM_PIN1 A1 // second luminosity sensor pin

#define TEMP_PIN A2  // first temperature sensor pin
#define TEMP_PIN1 A3 // second temperature sensor pin

ChainableLED leds(LED_PIN, LED_PIN1, LEDS_NUM);

int ledMode = 0; // initialize the LED mode variable

ISR(TIMER1_COMPA_vect) // led state update interrupt
{
    static bool state = false;
    if (state)
    { // if the state is true
        leds.setColorRGB(0, 0, 0, 0);
        state = false;
    }
    else
    {
        leds.setColorRGB(0, 255, 0, 0);
        state = true;
    }
}

void btnPressed()
{
    leds.setColorRGB(0, 0, 0, 0);
    delay(5000);
    leds.setColorRGB(0, 255, 0, 0);
    Serial.println("\nButton interrupt is pressed\n");
}

void setup()
{
    // put your setup code here, to run once:


    pinMode(GBTN_PIN, INPUT);

    pinMode(LUM_PIN, OUTPUT);
    pinMode(LUM_PIN1, OUTPUT);

    pinMode(TEMP_PIN, OUTPUT);
    pinMode(TEMP_PIN1, OUTPUT);

    // initialiser le timer1
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0; // initialize counter value to 0

    OCR1A = 32000; // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS12); // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

    attachInterrupt(digitalPinToInterrupt(RBTN_PIN), btnPressed, FALLING);

    Serial.begin(9600);

    // Check if the green button is pressed at startup
    if (digitalRead(GBTN_PIN) == LOW)
    {
        leds.setColorRGB(0, 0, 255, 0);
        Serial.println("\nGreen button is pressed\n");
    }

    Serial.println("\nMain code is running\n");

}

void loop()
{
    // put your main code here, to run repeatedly:

    // Check Luminosity sensor pins
    Serial.println("LUM : " + String(analogRead(LUM_PIN)));
    Serial.println("LUM1 : " + String(analogRead(LUM_PIN1)));

    delay(1000);

    // Check Temperature sensor pins
    Serial.println("TEMP : " + String(analogRead(TEMP_PIN)));
    Serial.println("TEMP1 : " + String(analogRead(TEMP_PIN1)));

    delay(1000);
}