#include <Arduino.h> // Arduino library
#include <SPI.h>     // Serial Peripheral Interface library for communication with the SD card and GPS module
#include <SD.h>      // Library for the SD card
#include <SoftwareSerial.h> // Library for the GPS module

#include <ChainableLED.h>    // Library for the LED strip
#include <Adafruit_BME280.h> // Library for the BME280 sensor

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

int ledMode = 0; // initialize the variable for the LED mode

ISR(TIMER1_COMPA_vect) // led state update interrupt
{
    static bool ledState = false;
    switch (ledMode)
    {
    case 1: // standard mode
        leds.setColorRGB(0, 0, 255, 0); // LED to green
        break;
    case 2: // configuration mode
        leds.setColorRGB(0, 255, 255, 0); // LED to yellow
        break;
    case 3: // economy mode
        leds.setColorRGB(0, 0, 0, 255); // LED to blue
        break;
    case 4: // maintenance mode
        leds.setColorRGB(0, 255, 127, 0); // LED to orange
        break;
    case 5: // clock access error mode
        if (ledState)
        {
            ledState = false;
            leds.setColorRGB(0, 255, 0, 0); // LED to red
        }
        else
        {
            ledState = true;
            leds.setColorRGB(0, 0, 0, 255); // LED to blue
        }
        break;
    case 6: // GPS access error mode
        if (ledState)
        {
            ledState = false;
            leds.setColorRGB(0, 255, 0, 0); // LED to red
        }
        else
        {
            ledState = true;
            leds.setColorRGB(0, 255, 127, 0); // LED to yellow
        }
        break;
    case 7: // captor acess error mode
        if (ledState)
        {
            ledState = false;
            leds.setColorRGB(0, 255, 0, 0); // LED to red
        }
        else
        {
            ledState = true;
            leds.setColorRGB(0, 0, 255, 0); // LED to green
        }
        break;
    case 8: // Data incoherence mode
        if (ledState)
        {
            ledState = false;
            leds.setColorRGB(0, 255, 0, 0); // LED to red
        }
        else
        {
            leds.setColorRGB(0, 0, 255, 0); // LED to green
            ledMode = 11;
        }
        break;
    case 9: // SD card access error mode
        if (ledState)
        {
            ledState = false;
            leds.setColorRGB(0, 255, 0, 0); // LED to red
        }
        else
        {
            ledState = true;
            leds.setColorRGB(0, 255, 255, 255); // LED to white
        }
        break;
    case 10: // SD card access or edit error mode
        if (ledState)
        {
            ledState = false;
            leds.setColorRGB(0, 255, 0, 0); // LED to red
        }
        else
        {
            leds.setColorRGB(0, 255, 255, 255); // LED to white
            ledMode = 11;
        }
        break;
    case 11: // delay 2x précedent mode
            ledState != ledState;
        break;
    default:
        leds.setColorRGB(0, 0, 0, 0); // LED to off
        break;
    }
}

void btnIntPressed()
{
    leds.setColorRGB(0, 0, 0, 255);

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
/*
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
}*/

void checkGPS()
{
    // Check GPS module
    unsigned char gpsData[64];

    int i = 0;

    while (GPS.available())
    {
        gpsData[i++] = GPS.read();

        i+=1;

        if (i == 64) break;
        
    };

    Serial.print("\nGPS : ");
    Serial.write(gpsData, i);
    Serial.print("\n\n");

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

    //SD.begin(SDPIN); // initialize the SD card

    // Check if the green button is pressed at startup
    if (digitalRead(GBTN_PIN) == LOW)
    {
        leds.setColorRGB(0, 0, 255, 0);

        Serial.println("\nButton is pressed\n");

        delay(1000);
    }
    noInterrupts(); // disable all interrupts
    // initialize timer1
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0;  // initialize counter value to 0

    OCR1A = 64000;           // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12);  // CTC mode
    TCCR1B |= (1 << CS12);   // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

    Serial.println("\nTest code is running\n");
    interrupts(); // enable all interrupts
}

void loop()
{
    checkSensors(); // check the multi-sensor

    //checkSD(); // check the SD card

    checkGPS(); // check the GPS module
}
