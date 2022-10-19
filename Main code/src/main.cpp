// Meteorological data processing

#include <dependencies.h> // include all the libraries and the defines

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM); // initialize the LED strip

Adafruit_BME280 bme; // initialize I2C communication for the bme280 sensor

SoftwareSerial ss(RX_PIN_TO_GPS, TX_PIN_TO_GPS); // initialize the pins for the GPS module

TinyGPSPlus gps; // initialize the GPS module

DS1307 clock; // initialize the RTC module

struct config config; // initialize the config structure

struct modeVar modeVar; // initialize the mode structure

struct sensorsData sensorsData; // initialize the sensors data structure

struct gpsData gpsData; // initialize the GPS data structure

struct rtcData rtcData; // initialize the RTC data structure

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
            state += (LED_UPDATE_INTERVAL/62500);
        }
        else if (state >= 1)// 1s
        {
            if (modeVar.ledMode==ERROR_CAPTOR_ACCESS) leds.setColorRGB(GREEN);
            if (modeVar.ledMode==ERROR_GPS) leds.setColorRGB(YELLOW);
            if (modeVar.ledMode==ERROR_CLOCK_ACCESS) leds.setColorRGB(BLUE);
            if (modeVar.ledMode==ERROR_SD_FULL) leds.setColorRGB(WHITE);
            state += (LED_UPDATE_INTERVAL/62500);
        }
        else if (state >= 2)
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
        else if (state >= 1)
        {
            if (modeVar.ledMode==ERROR_SD_WRITE) leds.setColorRGB(WHITE);
            if (modeVar.ledMode==ERROR_DATA_INCOHERENCE) leds.setColorRGB(GREEN);
            state += ((LED_UPDATE_INTERVAL / 2)/62500);
        }
        else if (state >= 2)
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

void getData()
{
    // Sensors data
    if (bme.begin(BME_ADDRESS) || bme.begin(BME_ADDRESS_ALT))
    {
        sensorsData.luminosity = analogRead(LUM_DATA_PIN)*100.0/1023.0; // get the luminosity data
        sensorsData.temperature = bme.readTemperature();                // get the temperature data 
        sensorsData.pressure = bme.readPressure() / 100.0F;             // get the pressure data
        sensorsData.humidity = bme.readHumidity();                      // get the humidity data
        sensorsData.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);  // get the altitude data
    }
    else
    {
        modeVar.ledMode = ERROR_CAPTOR_ACCESS;

    }
    

    // GPS data
    /*
    while (ss.available() > 0)
    {
        if (gps.encode(ss.read()))
        {
            gpsData.latitude = gps.location.lat();    // get the latitude data
            gpsData.longitude = gps.location.lng();   // get the longitude data
            gpsData.altitude = gps.altitude.meters(); // get the altitude data
        }
    }
    */

    // RTC data
    rtcData.year = clock.year+2000; // get the year data
    rtcData.month = clock.month;    // get the month data 
    rtcData.day = clock.dayOfMonth; // get the day data
    rtcData.hour = clock.hour;      // get the hour data
    rtcData.minute = clock.minute;  // get the minute data
    rtcData.second = clock.second;  // get the second data

    // Check data coherence

    // Check temperature data
    if (sensorsData.temperature < -50 || sensorsData.temperature > 50)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Temperature data incoherence");
    };

    // Check pressure data
    if (sensorsData.pressure < 800 || sensorsData.pressure > 1200)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Pressure data incoherence");
    };

    // Check humidity data
    if (sensorsData.humidity < 0 || sensorsData.humidity > 100)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Humidity data incoherence");
    };

    // Check altitude data
    if (sensorsData.luminosity < 0 || sensorsData.luminosity > 100)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Luminosity data incoherence");
    };
    /*
    // Check altitude data
    if (gpsData.latitude < -90 || gpsData.latitude > 90)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Latitude data incoherence");
    };

    // Check longitude data
    if (gpsData.longitude < -180 || gpsData.longitude > 180)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Longitude data incoherence");
    };

    // Check altitude data
    if (gpsData.altitude < -1000 || gpsData.altitude > 10000)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Altitude data incoherence");
    };
    */
    // Check year data
    if (rtcData.year < 2000 || rtcData.year > 2100)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Year data incoherence");
    };

    // Check month data
    if (rtcData.month < 1 || rtcData.month > 12)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Month data incoherence");
    };

    // Check day data
    if (rtcData.day < 0 || rtcData.day > 31)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Day data incoherence");
    };

    return;

}

void storeData()
{
    // Store data in the SD card

    // Open the file
    File dataFile = SD.open("data.csv", FILE_WRITE);
    if (dataFile)
    {
        // Write data
        
    }
    else
    {
        modeVar.ledMode = ERROR_SD_WRITE;
        Serial.println("ERROR: Unable to open the file");
    };
 
}

void setup()
{
    Serial.begin(9600); // initialize the serial communication
    Wire.begin();       // initialize the I2C communication
    SD.begin(SD_PIN);   // initialize the SD card

    pinMode(RBTN_PIN, INPUT); // define the red button pin as an input
    pinMode(GBTN_PIN, INPUT); // define the green button pin as an input

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
    getData();

    // Store DATA
    storeData();

    delay(5000);
    modeVar.ledMode++;

}