#include <Arduino.h>               // Arduino library
#include <Wire.h>           // I2C library
#include <SPI.h>                   // Serial Peripheral Interface library for communication with the SD card and GPS module
#include <SD.h>                    // Library for the SD card
#include <SoftwareSerial.h> // Library for the GPS module

#include <ChainableLED.h>            // Library for the LED strip
#include <Adafruit_Sensor.h>         // Library for the BME280 sensor
#include <Adafruit_BME280.h>         // Library for the BME280 sensor
#include <TinyGPSPlus.h>             // Library for the GPS module

#define GBTN_PIN 2 // define the pin for the green button
#define RBTN_PIN 3 // define the pin for the red button

#define LED_PIN 5       // define the alimentation pin for the LED
#define LED_DATA_PIN 6  // define the data pin for the LED
#define LEDS_NUM 1      // number of LEDs in the chain

#define LUM_PIN A0  // first luminosity sensor pin
#define LUM_PIN1 A1 // second luminosity sensor pin

#define SEALEVELPRESSURE_HPA (1013.25) // define the sea level pressure

#define SD_PIN 4 // define the pin for the SD card

#define GPS_TX 7 // define the TX pin for the GPS module
#define GPS_RX 8 // define the RX pin for the GPS module

#define STANDARD 1                // define the value of the standard mode
#define CONFIGURATION 2           // define the value of the configuration mode
#define ECONOMY 3                 // define the value of the economy mode
#define MAINTENANCE 4             // define the value of the maintenance mode
#define ERROR_CLOCK_ACCESS 5      // define the value of the clock error
#define ERROR_GPS 6               // define the value of the GPS error mode
#define ERROR_CAPTOR_ACCESS 7     // define the value of the captor acess error mode
#define ERROR_DATA_INCOHERENCE 8  // define the value of the INCOHERENCE error mode
#define ERROR_SD_FULL 9           // define the value of the SD card FULL error mode
#define ERROR_SD_WRITE 10         // define the value of the BME280 access error mode

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM);

Adafruit_BME280 bme; // I2C

SoftwareSerial GPS(GPS_TX, GPS_RX); // initialize the pins for the GPS module

TinyGPSPlus gps; // initialize the GPS module

int ledMode = 0;    // initialize the variable for the LED mode
int actualMode = 0; // initialize the variable for the actual mode

ISR(TIMER1_COMPA_vect) // led state update interrupt
{
    static bool ledState = false;

    switch (ledMode)
    {
    case STANDARD:                      // standard mode
        leds.setColorRGB(0, 0, 255, 0); // LED to green
        break;

    case CONFIGURATION:                   // configuration mode
        leds.setColorRGB(0, 255, 255, 0); // LED to yellow
        break;

    case ECONOMY:                       // economy mode
        leds.setColorRGB(0, 0, 0, 255); // LED to blue
        break;

    case MAINTENANCE:                    // maintenance mode
        leds.setColorRGB(0, 255, 64, 0); // LED to orange
        break;

    case ERROR_CLOCK_ACCESS: // clock access error mode
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

    case ERROR_GPS: // GPS access error mode
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

    case ERROR_CAPTOR_ACCESS: // captor acess error mode
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

    case ERROR_DATA_INCOHERENCE: // Data incoherence mode
        if (ledState)
        {
            ledState = false;
            leds.setColorRGB(0, 255, 0, 0); // LED to red
        }
        else
        {
            leds.setColorRGB(0, 0, 255, 0); // LED to green
            ledMode = 12;
        }
        break;
    case ERROR_SD_FULL: // SD card FULL error mode
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

    case ERROR_SD_WRITE: // SD card access or edit error mode
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
        ledMode = ERROR_SD_WRITE;
        break;
    case 12: // delay 2x mode 8
        ledState = !ledState;
        ledMode = ERROR_DATA_INCOHERENCE;
        break;

    default:
        leds.setColorRGB(0, 0, 0, 0); // LED to off
        break;
    }
}

void maintenanceMode()
{
    int timePressed = millis();    // initialize the variable for the time the button is pressed
    while (digitalRead(RBTN_PIN) != HIGH) // if the red button is pressed
    {
        if (millis() - timePressed > 5000) // if the button is pressed for more than 1 second
        {
            if (ledMode != MAINTENANCE) // if the LED is not in maintenance mode
            {
                ledMode, actualMode = STANDARD; // set the LED to maintenance mode
            }
            else // if the LED is in maintenance mode
            {
                ledMode, actualMode = MAINTENANCE; // set the LED to the actual mode
            }
        }
    }
}

void ecoMode()
{
    Serial.println("ecoMode");
    // change config file
}

void checkSensors()
{
    // Check Luminosity sensor pins
    Serial.println("LUM : " + String(analogRead(LUM_PIN)));
    Serial.println("LUM1 : " + String(analogRead(LUM_PIN1)));

    delay(1000);

    Serial.println();

    // Check sensors pins
	Serial.print("Temperature in deg C = ");
	Serial.println(bme.readTemperature());

	Serial.print("Pressure in hPa = ");
	Serial.println(bme.readPressure() / 100.0F);

	Serial.print("Altitude in m = ");
	Serial.println(bme.readAltitude(SEALEVELPRESSURE_HPA));

	Serial.print("Humidity in %RH = ");
	Serial.println(bme.readHumidity());

	Serial.println();
	delay(5000);
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

    bme.begin(0x76); // initialize the BME280 sensor

    if (!bme.begin()) 
    {  
        Serial.println("Could not find a valid BME280 sensor, check wiring!");

    }

    GPS.begin(9600); // initialize the serial communication with the GPS module

    pinMode(GBTN_PIN, INPUT); // initialize the pin for the green button

    pinMode(LUM_PIN, OUTPUT);  // initialize the pin for the luminosity sensor
    pinMode(LUM_PIN1, OUTPUT); //

    //SD.begin(SD_PIN); // initialize the SD card

    // Check if the green button is pressed at startup
    if (digitalRead(GBTN_PIN) == LOW)
    {
        leds.setColorRGB(0, 0, 255, 0);

        Serial.println("\nButton is pressed\n");

        delay(1000);
    }

    attachInterrupt(digitalPinToInterrupt(RBTN_PIN), maintenanceMode, CHANGE);
    attachInterrupt(digitalPinToInterrupt(GBTN_PIN), ecoMode, LOW); // attach interrupt to the green button

    SD.begin(SD_PIN); // initialize the SD card
    
    // Timer configuration
    noInterrupts(); // disable all interrupts
    // initialize timer1
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
    // checkSD(); // check the SD card
    //// checkGPS(); // check the GPS module

}
