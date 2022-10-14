#include <Arduino.h>

#define PINBTN 3

#define PINSLUM A0 // first luminosity sensor pin
#define PINSLUM1 A1 // second luminosity sensor pin

#define PINSTEMP A2 // first temperature sensor pin
#define PINSTEMP1 A3 // second temperature sensor pin

void setup() {
  // put your setup code here, to run once:
  pinMode(PINBTN, INPUT);

  pinMode(PINSLUM, OUTPUT);
  pinMode(PINSLUM1, OUTPUT);

  pinMode(PINSTEMP, OUTPUT);
  pinMode(PINSTEMP1, OUTPUT);

  Serial.begin(9600);
  
  if (digitalRead(PINBTN) == LOW) {
    Serial.println("Button is pressed");

  }

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("LUM : " + String(analogRead(PINSLUM)));
  Serial.println("LUM1 : " + String(analogRead(PINSLUM1)));

  delay(1000);

  Serial.println("TEMP : " + String(analogRead(PINSTEMP)));
  Serial.println("TEMP1 : " + String(analogRead(PINSTEMP1)));

  delay(1000);

}