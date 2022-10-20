// Meteorological data processing

#include <dependencies.h> // include all the libraries and the defines

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM); // initialize the LED strip

Adafruit_BME280 bme; // initialize I2C communication for the bme280 sensor

SoftwareSerial ss(RX_PIN_TO_GPS, TX_PIN_TO_GPS); // initialize the pins for the GPS module

TinyGPSPlus gps; // initialize the GPS module

DS1307 clock; // initialize the RTC module

struct config config; // initialize the config structure

struct modeVar modeVar; // initialize the mode structure

struct data data; // initialize the data structure

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
        else if (modeVar.ledMode == ECONOMY) leds.setColorRGB(BLUE);
        else if (modeVar.ledMode == STANDARD) leds.setColorRGB(GREEN);
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

            }
            else
            {
                modeVar.actual = modeVar.previous;
                modeVar.previous = MAINTENANCE;

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
            }
            else if (modeVar.actual == ECONOMY)
            {
                modeVar.previous = modeVar.actual;
                modeVar.actual = STANDARD;
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

String toString()
{
    String dataString = "";
    dataString += String(*data.rtc.year);
    dataString += "/";
    dataString += String(*data.rtc.month);
    dataString += "/";
    dataString += String(*data.rtc.day);
    dataString += " ";
    dataString += String(*data.rtc.hour);
    dataString += ":";
    dataString += String(*data.rtc.minute);
    dataString += ":";
    dataString += String(*data.rtc.second);
    dataString += " ";

    return dataString;
}

void getData()
{/*
    // Luminosity data
    static float luminosity = analogRead(LUM_DATA_PIN)*100.0/1023.0; // get the luminosity value
    Serial.println(luminosity);
    // Check luminosity data
    if (luminosity < 0 || luminosity > 100)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Luminosity data incoherence");
    };

    data.luminosity.luminosity = &luminosity; // store in struct the luminosity data
    Serial.println(*data.luminosity.luminosity);
    // Sensors data
    if (bme.begin(BME_ADDRESS) || bme.begin(BME_ADDRESS_ALT))
    {
        static float temperature = bme.readTemperature();                // get the temperature data 
        static float pressure = bme.readPressure()/100.0F;               // get the pressure data
        static float humidity = bme.readHumidity();                      // get the humidity data
        static float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);  // get the altitude data
        
        // Check temperature data
        if (temperature < -50 || temperature > 50)
        {
            modeVar.ledMode = ERROR_DATA_INCOHERENCE;
            Serial.println("ERROR: Temperature data incoherence");

        };

        // Check pressure data
        if (pressure < 800 || pressure > 1200)
        {
            modeVar.ledMode = ERROR_DATA_INCOHERENCE;
            Serial.println("ERROR: Pressure data incoherence");

        };

        // Check humidity data
        if (humidity < 0 || humidity > 100)
        {
            modeVar.ledMode = ERROR_DATA_INCOHERENCE;
            Serial.println("ERROR: Humidity data incoherence");

        };

        data.sensors.temperature = &temperature;                   // store in struct the temperature data
        data.sensors.pressure = &pressure;                         // store in struct the pressure data
        data.sensors.humidity = &humidity;                         // store in struct the humidity data
        data.sensors.altitude = &altitude;                         // store in struct the altitude data

    }
    else
    {
        modeVar.ledMode = ERROR_CAPTOR_ACCESS;

    }
    

    // GPS data

    while (ss.available() > 0)
    {
        if (gps.encode(ss.read()))
        {
            gpsData.latitude = gps.location.lat();    // get the latitude data
            gpsData.longitude = gps.location.lng();   // get the longitude data
            gpsData.altitude = gps.altitude.meters(); // get the altitude data

            // Check latitude data
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

        }
    }
    */
    // RTC data
    static int year = clock.year+2000; // get the year data
    static int month = clock.month;    // get the month data
    static int day = clock.dayOfMonth; // get the day data
    static int hour = clock.hour;      // get the hour data
    static int minute = clock.minute;  // get the minute data
    static int second = clock.second;  // get the second data

    // Check year data
    if (year < 2000 || year > 2099)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Year data incoherence");

    };

    // Check month data
    if (month < 1 || month > 12)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Month data incoherence");

    };

    // Check day data
    if (day < 1 || day > 31)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        Serial.println("ERROR: Day data incoherence");

    };

    data.rtc.year = &year;         // store in struct the year data
    data.rtc.month = &month;       // store in struct the month data
    data.rtc.day = &day;           // store in struct the day data
    data.rtc.hour = &hour;         // store in struct the hour data
    data.rtc.minute = &minute;     // store in struct the minute data
    data.rtc.second = &second;     // store in struct the second data
    
    return;

}

void storeData()
{
    // Store data in the SD card
    noInterrupts();

    String string = toString();

    Serial.println(string);

    // Open the file
    File dataFile = SD.open("data.txt", FILE_WRITE);
    if (dataFile)
    {
        // Write data
        //String string = toString();

        dataFile.print(string);

        Serial.println("Data stored in the SD card");

        // Close the file
        dataFile.close();

    }
    else
    {
        modeVar.ledMode = ERROR_SD_WRITE;
        Serial.println("ERROR: Unable to open the file");
    };

    SD.end();

    interrupts();

    return;
 
}

void setup()
{
    Serial.begin(MONITOR_BAUD); // initialize the serial communication
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
    if (modeVar.error == NO_ERROR)
    {
        modeVar.ledMode = modeVar.actual;
    }
    else
    {
        modeVar.ledMode = modeVar.error;
    };
    Serial.println(modeVar.actual);// new line
}