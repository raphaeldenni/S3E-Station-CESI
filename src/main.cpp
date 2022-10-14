#include <Arduino.h>
#include <ChainableLED.h>
#include <Adafruit_BME280.h>

#define PINBTNG 2
#define PINBTNR 3

#define PINLED 4
#define PINLED1 5
#define NUMBLEDS 1

#define PINSLUM A0 // first luminosity sensor pin
#define PINSLUM1 A1 // second luminosity sensor pin

#define PINSTEMP A2 // first temperature sensor pin
#define PINSTEMP1 A3 // second temperature sensor pin

ChainableLED leds(PINLED, PINLED1, NUMBLEDS);

void btnPressed();

void setup() {
  // put your setup code here, to run once:
  pinMode(PINBTNG, INPUT);

  pinMode(PINSLUM, OUTPUT);
  pinMode(PINSLUM1, OUTPUT);

  pinMode(PINSTEMP, OUTPUT);
  pinMode(PINSTEMP1, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PINBTNR), btnPressed, FALLING);

  Serial.begin(9600);
  
  // Check if the green button is pressed at startup
  if (digitalRead(PINBTNG) == LOW) {
    leds.setColorRGB(0, 0, 255, 0);
    Serial.println("Button is pressed");

  }

}

void loop() {
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

void btnPressed() {
  leds.setColorRGB(0, 0, 0, 0);
  delay(5000);
  leds.setColorRGB(0, 255, 0, 0);
  Serial.println("Button interrupt is pressed");
}