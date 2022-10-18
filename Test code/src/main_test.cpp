#include <Arduino.h> // Arduino library
#include <SPI.h>     // Serial Peripheral Interface library for communication with the SD card and GPS module
#include <SD.h>      // Library for the SD card
#include <SoftwareSerial.h> // Library for the GPS module

#include <ChainableLED.h>    // Library for the LED strip
#include <Adafruit_BME280.h> // Library for the BME280 sensor

#define GBTN_PIN 2 // define the pin for the green button
#define RBTN_PIN 3 // define the pin for the red button

#define LED_PIN 5   // define the pin for the LED
#define LED_DATA_PIN 6  // initialize the pin for the LED
#define LEDS_NUM 1 // number of LEDs in the chain

#define LUM_PIN A0  // first luminosity sensor pin
#define LUM_PIN1 A1 // second luminosity sensor pin

#define TEMP_PIN A2  // first temperature sensor pin
#define TEMP_PIN1 A3 // second temperature sensor pin

#define SDPIN 4 // define the pin for the SD card

#define GPS_RX 7 // define the pin for the GPS module
#define GPS_TX 8 // define the pin for the GPS module

#define LED_STANDARD 1 // define the value of the standard mode
#define LED_CONFIGURATION 2  // define the value of the configuration mode
#define LED_ECONOMY 3  // define the value of the economy mode
#define LED_MAINTENANCE 4  // define the value of the maintenance mode
#define LED_ERROR_CLOCK_ACCESS 5  // define the value of the clock error
#define LED_ERROR_GPS 6  // define the value of the GPS error mode
#define LED_ERROR_CAPTOR_ACCESS 7  // define the value of the captor acess error mode
#define LED_ERROR_DATA_INCOHERENCE 8  // define the value of the INCOHERENCE error mode
#define LED_ERROR_SD_FULL 9  // define the value of the SD card FULL error mode
#define LED_ERROR_SD_WRITE 10  // define the value of the BME280 access error mode

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM);

SoftwareSerial GPS(GPS_RX, GPS_TX); // initialize the pins for the GPS module

int ledMode = 0; // initialize the variable for the LED mode
int timePressed = 0; // initialize the variable for the time the button is pressed

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
        leds.setColorRGB(0, 255, 64, 0); // LED to orange
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
            leds.setColorRGB(0, 255, 255, 0); // LED to yellow
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
    case 9: // SD card FULL error mode
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
    case 11: // delay 2x mode 10
            ledState = !ledState;
            ledMode = 10;
        break;
    case 12: // delay 2x mode 8
            ledState = !ledState;
            ledMode = 8;
        break;
    default:
        leds.setColorRGB(0, 0, 0, 0); // LED to off
        break;
    }
}

void RbtnIntPressed()
{
    if (digitalRead(RBTN_PIN) == LOW)
        if (millis() - timePressed > 5000)
        {
            timePressed = 0;
            Serial.println("RbtnIntPressed 5+");
        }
    else
    {
        timePressed = 0;
    }
}

void GbtnIntPressed()
{
    if (digitalRead(GBTN_PIN) == LOW)
        if (millis() - timePressed > 5000)
        {
            timePressed = 0;
            Serial.println("GbtnIntPressed 5+");
        }
    else
    {
        timePressed = 0;
    }
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

    // Check if the green button is pressed at startup
    if (digitalRead(GBTN_PIN) == LOW)
    {
        leds.setColorRGB(0, 0, 255, 0);

        Serial.println("\nButton is pressed\n");

        delay(1000);
    }

    attachInterrupt(digitalPinToInterrupt(RBTN_PIN), RbtnIntPressed, CHANGE);
    attachInterrupt(digitalPinToInterrupt(GBTN_PIN), GbtnIntPressed, CHANGE); // attach interrupt to the green button

    SD.begin(SDPIN); // initialize the SD card
    
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
    // checkSensors(); // check the multi-sensor

    //checkSD(); // check the SD card

    // checkGPS(); // check the GPS module
}
