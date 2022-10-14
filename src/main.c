#include <Arduino.h>

#define PINBTN 3

#define PINLUM A0
#define PINLUM1 A1

#define PINTEMP A2
#define PINTEMP1 A3

void setup() {
  // put your setup code here, to run once:
  pinMode(PINBTN, INPUT);

  pinMode(PINLUM, OUTPUT);
  pinMode(PINLUM1, OUTPUT);

  pinMode(PINTEMP, OUTPUT);
  pinMode(PINTEMP1, OUTPUT);

  Serial.begin(9600);
  
  if (digitalRead(PINBTN) == LOW) {
    Serial.println("Button is pressed");

  }

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("LUM : " + String(analogRead(PINLUM)));
  Serial.println("LUM1 : " + String(analogRead(PINLUM1)));

  delay(1000);

  Serial.println("TEMP : " + String(analogRead(PINTEMP)));
  Serial.println("TEMP1 : " + String(analogRead(PINTEMP1)));

  delay(1000);

}