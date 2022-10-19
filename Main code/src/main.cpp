#include <Arduino.h>        // Arduino library

#include <ChainableLED.h>            // Library for the LED strip
#include <config.h>                  // Configuration file

#define GBTN_PIN 2 // define the pin for the green button
#define RBTN_PIN 3 // define the pin for the red button

#define LED_PIN 5      // define the alimentation pin for the LED
#define LED_DATA_PIN 6 // define the data pin for the LED
#define LEDS_NUM 1     // number of LEDs in the chain

#define LEDUPDATEINTERVAL 31250 // define the time between each LED update 62500 = 1s

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM);

struct modeVar modeVar; // define the configuration structure
struct config config;   // define the configuration structure

ISR(TIMER1_COMPA_vect) // check if button is pressed
{
    // LED update
    modeVar.ledMode = modeVar.actual;
    if (modeVar.ledMode == MAINTENANCE)
    {
        leds.setColorRGB(0, 255, 0, 0);
    }
    if (modeVar.ledMode == ECONOMY){
        leds.setColorRGB(0, 0, 0, 255);
    }
    if (modeVar.ledMode == STANDARD) leds.setColorRGB(0, 0, 255, 0);

    // Button update
    if (digitalRead(RBTN_PIN) == LOW)
    {
        modeVar.rBtntimePressed++;
        if (modeVar.rBtntimePressed > (62500*config.timeToSwitch/LEDUPDATEINTERVAL))
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
        if (modeVar.gBtntimePressed > (62500*config.timeToSwitch/LEDUPDATEINTERVAL))
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
    Serial.begin(9600); // initialize the serial communication
    if (digitalRead(GBTN_PIN) == LOW) configMode(); // if the green button is pressed at startup, enter configuration mode
    // Timer configuration
    noInterrupts(); // disable all interrupts
    // initialize timer1
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1 = 0;  // initialize counter value to 0

    OCR1A = LEDUPDATEINTERVAL;           // compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM12);  // CTC mode
    TCCR1B |= (1 << CS12);   // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

    interrupts(); // enable all interrupts
}

void loop()
{
    // Get DATA
    // Store DATA
    
}