#include <Arduino.h>            // Arduino library
#include <SPI.h>                // SPI library
#include <Adafruit_I2CDevice.h> // I2C library
#include <Adafruit_Sensor.h>    // I2C library

#include <ChainableLED.h> // Library for the LED strip
#include <config.h>       // Configuration file

#define GBTN_PIN 2 // define the pin for the green button
#define RBTN_PIN 3 // define the pin for the red button

#define LED_PIN 5      // define the alimentation pin for the LED
#define LED_DATA_PIN 6 // define the data pin for the LED
#define LEDS_NUM 1     // number of LEDs in the chain

#define LED_UPDATE_INTERVAL 31250 // define the time between each LED update 62500 = 1s, Value is between 0 and 62500

#define STANDARD 1               // define the value of the standard mode
#define CONFIGURATION 2          // define the value of the configuration mode
#define ECONOMY 3                // define the value of the economy mode
#define MAINTENANCE 4            // define the value of the maintenance mode
#define ERROR_CLOCK_ACCESS 5     // define the value of the clock error
#define ERROR_GPS 6              // define the value of the GPS error mode
#define ERROR_CAPTOR_ACCESS 7    // define the value of the captor acess error mode
#define ERROR_DATA_INCOHERENCE 8 // define the value of the INCOHERENCE error mode
#define ERROR_SD_FULL 9          // define the value of the SD card FULL error mode
#define ERROR_SD_WRITE 10        // define the value of the BME280 access error mode

#define RED 0, 255, 0, 0
#define GREEN 0, 0, 255, 0
#define BLUE 0, 0, 0, 255
#define YELLOW 0, 255, 255, 0
#define ORANGE 0, 255, 50, 0
#define WHITE 0, 255, 255, 255

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM);

struct modeVar
{
    int actual = STANDARD;   // define the mode of the program
    int previous = STANDARD; // define the previous mode of the program
    int rBtntimePressed = 0; // define the pin for the red button
    int gBtntimePressed = 0; // define the pin for the green button
    int ledMode = STANDARD;
};

struct modeVar modeVar; // define the configuration structure
struct config config;   // define the configuration structure

ISR(TIMER1_COMPA_vect) // check if button is pressed
{
    static float state = 0;
    // LED update
    switch (modeVar.ledMode)
    {
    case ERROR_CAPTOR_ACCESS:
    case ERROR_GPS:
    case ERROR_CLOCK_ACCESS:
    case ERROR_SD_FULL:
        if (state <= 0)
        {
            leds.setColorRGB(RED);
            state += (62500 / LED_UPDATE_INTERVAL);
        }
        if (state >= 1)// 1s
        {
            if (modeVar.ledMode==ERROR_CAPTOR_ACCESS) leds.setColorRGB(GREEN);
            if (modeVar.ledMode==ERROR_GPS) leds.setColorRGB(YELLOW);
            if (modeVar.ledMode==ERROR_CLOCK_ACCESS) leds.setColorRGB(BLUE);
            if (modeVar.ledMode==ERROR_SD_FULL) leds.setColorRGB(WHITE);
            state += (LED_UPDATE_INTERVAL/62500);
        }
        if (state >= 2)
        {
            state = 0;
        }
        break;
    case ERROR_SD_WRITE:
    case ERROR_DATA_INCOHERENCE:
        if (state <= 0)
        {
            leds.setColorRGB(RED);
            state += (LED_UPDATE_INTERVAL/62500);
        }
        if (state >= 1)
        {
            if (modeVar.ledMode==ERROR_SD_WRITE) leds.setColorRGB(WHITE);
            if (modeVar.ledMode==ERROR_DATA_INCOHERENCE) leds.setColorRGB(GREEN);
            state += ((LED_UPDATE_INTERVAL / 2)/62500);
        }
        if (state >= 2)
        {
            state = 0;
        }
        break;
    default:
        state = 0;
        if (modeVar.ledMode == MAINTENANCE) leds.setColorRGB(ORANGE);
        if (modeVar.ledMode == ECONOMY) leds.setColorRGB(BLUE);
        if (modeVar.ledMode == STANDARD) leds.setColorRGB(GREEN);
        break;
    }
    // Button check
    if (digitalRead(RBTN_PIN) == LOW)
    {
        modeVar.rBtntimePressed++;
        if (modeVar.rBtntimePressed > (62500 * config.timeToSwitch / LED_UPDATE_INTERVAL))
        {
            modeVar.rBtntimePressed = 0;
            if (modeVar.actual != MAINTENANCE)
            {
                modeVar.previous = modeVar.actual;
                modeVar.actual = MAINTENANCE;
                Serial.println("ENTER MAINTENANCE MODE");
            }
            else
            {
                modeVar.actual = modeVar.previous;
                modeVar.previous = MAINTENANCE;
                Serial.println("EXIT MAINTENANCE MODE");
            }
        }
    }
    else if (digitalRead(GBTN_PIN) == LOW) // priority to the red button
    {
        modeVar.gBtntimePressed++;
        if (modeVar.gBtntimePressed > (62500 * config.timeToSwitch / LED_UPDATE_INTERVAL))
        {
            modeVar.gBtntimePressed = 0;
            if (modeVar.actual == STANDARD)
            {
                modeVar.previous = modeVar.actual;
                modeVar.actual = ECONOMY;
                Serial.println("ENTER ECONOMY MODE");
            }
            else if (modeVar.actual == ECONOMY)
            {
                modeVar.previous = modeVar.actual;
                modeVar.actual = STANDARD;
                Serial.println("ENTER STANDARD MODE");
            }
        }
    }
    else
    {
        modeVar.rBtntimePressed = 0;
        modeVar.gBtntimePressed = 0;
    }
}

void configMode()
{
    Serial.println("ENTER CONFIGURATION MODE");
    Serial.println("Enter help() to show the list of commands.");
    Serial.println("Enter exit to show the list of commands.");
}

void setup()
{
    pinMode(RBTN_PIN, INPUT); // define the red button pin as an input
    pinMode(GBTN_PIN, INPUT); // define the green button pin as an input
    Serial.begin(9600);       // initialize the serial communication
    if (digitalRead(GBTN_PIN) == LOW)
        configMode(); // if the green button is pressed at startup, enter configuration mode
    // Timer configuration
    noInterrupts(); // disable all interrupts
    // initialize timer1
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0;  // initialize counter value to 0

    OCR1A = LED_UPDATE_INTERVAL; // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12);      // CTC mode
    TCCR1B |= (1 << CS12);       // 256 prescaler
    TIMSK1 |= (1 << OCIE1A);     // enable timer compare interrupt

    interrupts(); // enable all interrupts
}

void loop()
{
    // Get DATA
    // Store DATA
    delay(5000);
    modeVar.ledMode++;
}