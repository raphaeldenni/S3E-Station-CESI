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

#define GBTN_PIN 2 // define the pin for the green button
#define RBTN_PIN 3 // define the pin for the red button

#define LED_PIN 5       // define the alimentation pin for the LED
#define LED_DATA_PIN 6  // define the data pin for the LED
#define LEDS_NUM 1      // number of LEDs in the chain

#define LUM_PIN A0  // first luminosity sensor pin
#define LUM_PIN1 A1 // second luminosity sensor pin

#define BME_ADDRESS 0x76 // define the BME280 sensor address
#define REFRESH_DELAY 1500 // define the delay between each refresh of the data
#define SEALEVELPRESSURE_HPA 1024.90 // define the sea level pressure

#define SD_PIN 4 // define the pin for the SD card

#define GPS_TX 7 // define the TX pin for the GPS module
#define GPS_RX 8 // define the RX pin for the GPS module

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM);

Adafruit_BME280 bme; // initialize I2C communication for the bme280 sensor

SoftwareSerial GPS(GPS_TX, GPS_RX); // initialize the pins for the GPS module

TinyGPSPlus gps; // initialize the GPS module

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
    Serial.println("LUM : " + String(analogRead(LUM_PIN)));
    Serial.println("LUM1 : " + String(analogRead(LUM_PIN1)));

    delay(1000);

    Serial.println();

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
    while (GPS.available() > 0)
    {
        gps.encode(GPS.read());
    };

    Serial.print("LAT=");
    Serial.println(gps.location.lat(), 6);
    Serial.print("LONG=");
    Serial.println(gps.location.lng(), 6);
    Serial.print("ALT=");
    Serial.println(gps.altitude.meters());
    Serial.println();

    delay(1000);
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

    GPS.begin(9600); // initialize the serial communication with the GPS module

    pinMode(GBTN_PIN, INPUT); // initialize the pin for the green button

    pinMode(LUM_PIN, OUTPUT);  // initialize the pin for the luminosity sensor
    pinMode(LUM_PIN1, OUTPUT); //

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
    // checkSD();      // check the SD card
    // checkGPS();     // check the GPS module

}
