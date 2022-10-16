#include <Arduino.h> // Arduino library
#include <SPI.h> // Serial Peripheral Interface library for communication with the SD card and GPS module

#include <ChainableLED.h> // Library for the LED strip
#include <SD.h> // Library for the SD card
#include <Adafruit_BME280.h> // Library for the BME280 sensor

#define PINBTNG 2 // initialize the pin for the green button
#define PINBTNR 3 // initialize the pin for the red button

#define PINLED 4   // initialize the pin for the LED
#define PINLED1 5  // initialize the pin for the LED
#define NUMBLEDS 1 // number of LEDs in the chain

#define PINSLUM A0  // first luminosity sensor pin
#define PINSLUM1 A1 // second luminosity sensor pin

#define PINSTEMP A2  // first temperature sensor pin
#define PINSTEMP1 A3 // second temperature sensor pin

ChainableLED leds(PINLED, PINLED1, NUMBLEDS);

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
    Serial.println("Button interrupt is pressed");
}

void setup()
{
    // put your setup code here, to run once:
    pinMode(PINBTNG, INPUT);

    pinMode(PINSLUM, OUTPUT);
    pinMode(PINSLUM1, OUTPUT);

    pinMode(PINSTEMP, OUTPUT);
    pinMode(PINSTEMP1, OUTPUT);
    
    // initialiser le timer1
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0; // initialize counter value to 0

    OCR1A = 32000; // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS12); // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

    attachInterrupt(digitalPinToInterrupt(PINBTNR), btnPressed, FALLING);

    Serial.begin(9600);

    // Check if the green button is pressed at startup
    if (digitalRead(PINBTNG) == LOW)
    {
        leds.setColorRGB(0, 0, 255, 0);
        Serial.println("Button is pressed");
    }
}

void loop()
{
    // put your main code here, to run repeatedly:

    // Check Luminosity sensor pins
    Serial.println("LUM : " + String(analogRead(PINSLUM)));
    Serial.println("LUM1 : " + String(analogRead(PINSLUM1)));

    delay(1000);

    // Check Temperature sensor pins
    Serial.println("TEMP : " + String(analogRead(PINSTEMP)));
    Serial.println("TEMP1 : " + String(analogRead(PINSTEMP1)));

    delay(1000);
}

void btnPressed() 
{

  leds.setColorRGB(0, 0, 0, 0);
  delay(5000);
  leds.setColorRGB(0, 255, 0, 0);
  Serial.println("Button interrupt is pressed");

}