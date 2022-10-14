#include <Arduino.h>
#define PINBTN 3

void setup() {
  // put your setup code here, to run once:
  pinMode(PINBTN, INPUT);

  if (digitalRead(PINBTN) == LOW) {
    Serial.begin(9600);
    Serial.println("Button is pressed");

  }

}

void loop() {
  // put your main code here, to run repeatedly:
}