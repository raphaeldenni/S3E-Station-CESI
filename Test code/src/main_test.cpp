#include <Arduino.h>        // Arduino library
#include <Wire.h>           // I2C library
#include <SPI.h>            // Serial Peripheral Interface library for communication with the SD card and GPS module
#include <SD.h>             // Library for the SD card
#include <SoftwareSerial.h> // Library for the GPS module

#include <ChainableLED.h>       // Library for the LED strip
#include <Adafruit_Sensor.h>    // Library for the BME280 sensor
#include <Adafruit_I2CDevice.h> // Library for the BME280 sensor
#include <Adafruit_BME280.h>    // Library for the BME280 sensor
#include <TinyGPSPlus.h>        // Library for the GPS module
#include "DS1307.h"             // Library for the RTC module

#define GBTN_PIN 2 // define the pin for the green button
#define RBTN_PIN 3 // define the pin for the red button

#define LED_PIN 5       // define the alimentation pin for the LED
#define LED_DATA_PIN 6  // define the data pin for the LED
#define LEDS_NUM 1      // number of LEDs in the chain

#define LUM_DATA_PIN A0  // first luminosity sensor pin
#define SECOND_LUM_PIN A1 // second luminosity sensor pin

#define BME_ADDRESS 0x76 // define the BME280 sensor address
#define REFRESH_DELAY 1500 // define the delay between each refresh of the data
#define SEALEVELPRESSURE_HPA 1024.90 // define the sea level pressure

#define SD_PIN 4 // define the pin for the SD card

#define TX_TO_GPS 8 // define the TX pin for the GPS module
#define RX_TO_GPS 7 // define the RX pin for the GPS module
#define GPS_BAUD 9600 // define the baud rate for the GPS module

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM);

Adafruit_BME280 bme; // initialize I2C communication for the bme280 sensor

SoftwareSerial ss(RX_TO_GPS, TX_TO_GPS); // initialize the pins for the GPS module

TinyGPSPlus gps; // initialize the GPS module

DS1307 clock; // initialize the RTC module

ISR(TIMER1_COMPA_vect) // led state update interrupt
{
    //Serial.println("\nTick\n");

}

void redPressed()
{
    leds.setColorRGB(0, 255, 0, 0);
    Serial.println("Red button pressed");

}

void greenPressed()
{
    leds.setColorRGB(0, 0, 255, 0);
    Serial.println("Green button pressed");

}

void checkSensors()
{
    // Check Luminosity sensor pins
    Serial.println("Luminosity : " + String(analogRead(LUM_DATA_PIN)*100.0/1023.0) + "%");
    Serial.println("LUM1 : " + String(analogRead(SECOND_LUM_PIN)));

    Serial.println();

    delay(1000);

    // Check sensors pins
    // Affichage de la TEMPÉRATURE
    Serial.print(F("Temperature = "));
    Serial.print(bme.readTemperature());
    Serial.println(F(" °C"));

    // Affichage du TAUX D'HUMIDITÉ
    Serial.print(F("Humidity = "));
    Serial.print(bme.readHumidity());
    Serial.println(F(" %"));
  
    // Affichage de la PRESSION ATMOSPHÉRIQUE
    Serial.print(F("Atmos pressure = "));
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(F(" hPa"));

    // Affichage de l'estimation d'ALTITUDE
    Serial.print(F("Altitude = "));
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(F(" m"));

	delay(REFRESH_DELAY);

    Serial.println();

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

void checkGPS()
{
    // Check GPS module
    while (ss.available() > 0)
    {
        gps.encode(ss.read());

        Serial.print(F("\nLocation: ")); 

        if (gps.location.isValid())
        {
            Serial.print(gps.location.lat(), 6);
            Serial.print(F(","));
            Serial.print(gps.location.lng(), 6);
            break;

        } 
        else
        {
            Serial.print(F("INVALID"));

        };

        Serial.println();

    };

    delay(1000);

}

void checkRTC()
{
    // Check RTC module
    clock.getTime();

    Serial.print("Datetime: ");

    Serial.print(clock.hour, DEC);
    Serial.print(":");

    Serial.print(clock.minute, DEC);
    Serial.print(":");

    Serial.print(clock.second, DEC);
    Serial.print("  ");

    Serial.print(clock.month, DEC);
    Serial.print("/");

    Serial.print(clock.dayOfMonth, DEC);
    Serial.print("/");

    Serial.print(clock.year+2000, DEC);
    Serial.print(" ");

    Serial.print(clock.dayOfMonth);
    Serial.print("*");

    switch (clock.dayOfWeek)// Friendly printout the weekday
    {
        case MON:
        Serial.print("MON");
        break;
        case TUE:
        Serial.print("TUE");
        break;
        case WED:
        Serial.print("WED");
        break;
        case THU:
        Serial.print("THU");
        break;
        case FRI:
        Serial.print("FRI");
        break;
        case SAT:
        Serial.print("SAT");
        break;
        case SUN:
        Serial.print("SUN");
        break;
    }

    Serial.println("\n");

}

void setup()
{
    Serial.begin(9600); // initialize the serial communication

    Wire.begin(); // initialize the I2C communication

    SD.begin(SD_PIN); // initialize the SD card

    bme.begin(BME_ADDRESS); // initialize the BME280 sensor

    /*
    // initialize the BME280 sensor
    if (!bme.begin(BME280_ADDRESS)) 
    {  
        Serial.println("Could not find a valid BME280 sensor, check wiring!");

    }
    */

    ss.begin(GPS_BAUD); // initialize the serial communication with the GPS module

    clock.begin(); // initialize the RTC module

    // WARNING ZONE : the following code change datetime of the RTC clock module. If the module is already set, don't use it.

    /*
    clock.fillDayOfWeek(WED); // set the day of the week

    clock.fillByHMS(16,05,00); // delay of 18 seconds (for 16:00 click the upload button at 15:42)

    clock.setTime();
    */

   // END WARNING ZONE
    
    pinMode(GBTN_PIN, INPUT); // initialize the pin for the green button

    pinMode(LUM_DATA_PIN, OUTPUT);  // initialize the pin for the luminosity sensor
    pinMode(SECOND_LUM_PIN, OUTPUT); //

    // Check if the green button is pressed at startup
    if (digitalRead(GBTN_PIN) == LOW)
    {
        leds.setColorRGB(0, 0, 255, 0);

        Serial.println("\nButton is pressed\n");

        delay(1000);
    }

    attachInterrupt(digitalPinToInterrupt(RBTN_PIN), redPressed, CHANGE);
    attachInterrupt(digitalPinToInterrupt(GBTN_PIN), greenPressed, CHANGE); // attach interrupt to the green button
    
    // Timer configuration
    noInterrupts(); // disable all interrupts

    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0;  // initialize counter value to 0

    OCR1A = 64000;           // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12);  // CTC led
    TCCR1B |= (1 << CS12);   // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

    Serial.println("\nTest code is running\n");
    interrupts(); // enable all interrupts

}

void loop()
{
    checkSensors(); // check the multi-sensor
    checkSD();      // check the SD card
    //checkGPS();     // check the GPS module
    checkRTC();     // check the RTC module

}
