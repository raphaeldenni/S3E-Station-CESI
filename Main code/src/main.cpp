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

void configMode()
{
    Serial.println("ENTER CONFIGURATION MODE");
    Serial.println("Enter help() to show the list of commands.");
    Serial.println("Enter exit to show the list of commands.");
}

ISR(TIMER1_COMPA_vect) // check if button is pressed
{
    static int state = 0;
    // LED update
    switch (modeVar.ledMode)
    {
    case ERROR_CAPTOR_ACCESS:
    case ERROR_GPS:
    case ERROR_CLOCK_ACCESS:
    case ERROR_SD_FULL:
        if (state == 2)
        {
            leds.setColorRGB(RED);
            state++;
        }
        else if (state == 4) // 1s
        {
            if (modeVar.ledMode == ERROR_CAPTOR_ACCESS)
                leds.setColorRGB(GREEN);
            if (modeVar.ledMode == ERROR_GPS)
                leds.setColorRGB(YELLOW);
            if (modeVar.ledMode == ERROR_CLOCK_ACCESS)
                leds.setColorRGB(BLUE);
            if (modeVar.ledMode == ERROR_SD_FULL)
                leds.setColorRGB(WHITE);
            state = 0;
        }
        else
            state++;
        break;
    case ERROR_SD_WRITE:
    case ERROR_DATA_INCOHERENCE:
        if (state == 4)
        {
            leds.setColorRGB(RED);
            state++;
        }
        else if (state == 6)
        {
            if (modeVar.ledMode == ERROR_SD_WRITE)
                leds.setColorRGB(WHITE);
            if (modeVar.ledMode == ERROR_DATA_INCOHERENCE)
                leds.setColorRGB(GREEN);
            state = 0;
        }
        else
            state++;
        break;
    default:
        state = 0;
        if (modeVar.ledMode == MAINTENANCE)
            leds.setColorRGB(ORANGE);
        else if (modeVar.ledMode == ECONOMY)
            leds.setColorRGB(BLUE);
        else if (modeVar.ledMode == STANDARD)
            leds.setColorRGB(GREEN);
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
                modeVar.ledMode = MAINTENANCE;
            }
            else
            {
                modeVar.actual = modeVar.previous;
                modeVar.previous = MAINTENANCE;
                modeVar.ledMode = modeVar.actual;
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
                modeVar.ledMode = ECONOMY;
            }
            else if (modeVar.actual == ECONOMY)
            {
                modeVar.previous = modeVar.actual;
                modeVar.actual = STANDARD;
                modeVar.ledMode = STANDARD;
            }
        }
    }
    else
    {
        modeVar.rBtntimePressed = 0;
        modeVar.gBtntimePressed = 0;
    }
}

void getData()
{   
    // Luminosity data
    data.luminosity.luminosity = analogRead(LUM_DATA_PIN) * 100.0 / 1023.0; // get the luminosity value
    // Check luminosity data
    if (data.luminosity.luminosity < 0 || data.luminosity.luminosity > 100)
    {
        modeVar.ledMode = ERROR_DATA_INCOHERENCE;
        
    };
    
    // Sensors data
    if (bme.begin(BME_ADDRESS) || bme.begin(BME_ADDRESS_ALT))
    {
        data.sensors.temperature = bme.readTemperature();                // get the temperature data
        data.sensors.pressure = bme.readPressure()/100.0F;               // get the pressure data
        data.sensors.humidity = bme.readHumidity();                      // get the humidity data
        data.sensors.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);  // get the altitude data

        // Check temperature data
        if (data.sensors.temperature < -50 || data.sensors.temperature > 50)
        {
            modeVar.ledMode = ERROR_DATA_INCOHERENCE;
            Serial.println("ERROR: Temperature data incoherence");

        };

        // Check pressure data
        if (data.sensors.pressure < 800 || data.sensors.pressure > 1200)
        {
            modeVar.ledMode = ERROR_DATA_INCOHERENCE;
            Serial.println("ERROR: Pressure data incoherence");

        };

        // Check humidity data
        if (data.sensors.humidity < 0 || data.sensors.humidity > 100)
        {
            modeVar.ledMode = ERROR_DATA_INCOHERENCE;
            Serial.println("ERROR: Humidity data incoherence");

        };                       // store in struct the altitude data

    }
    else
    {
        modeVar.ledMode = ERROR_CAPTOR_ACCESS;

    }
/*
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
    clock.getTime();

    data.rtc.year = clock.year + 2000; // get the year data
    data.rtc.month = clock.month;      // get the month data
    data.rtc.day = clock.dayOfMonth;   // get the day data
    data.rtc.hour = clock.hour;        // get the hour data
    data.rtc.minute = clock.minute;    // get the minute data
    data.rtc.second = clock.second;    // get the second data

    // Check year data
    if (data.rtc.year < 2000 || data.rtc.year > 2099)
    {
        modeVar.error = ERROR_DATA_INCOHERENCE;
        
    };

    // Check month data
    if (data.rtc.month < 1 || data.rtc.month > 12)
    {
        modeVar.error = ERROR_DATA_INCOHERENCE;
        
    };

    // Check day data
    if (data.rtc.day < 1 || data.rtc.day > 31)
    {
        modeVar.error = ERROR_DATA_INCOHERENCE;
        
    };

    return;
}

void storeData()
{ 
    noInterrupts();
    // Store in SD card
    File dataFile = SD.open("data.csv", FILE_WRITE);

    if (dataFile)
    {
        dataFile.print(data.rtc.year);
        dataFile.print("-");
        dataFile.print(data.rtc.month);
        dataFile.print("-");
        dataFile.print(data.rtc.day);
        dataFile.print(" ");
        dataFile.print(data.rtc.hour);
        dataFile.print(":");
        dataFile.print(data.rtc.minute);
        dataFile.print(":");
        dataFile.print(data.rtc.second);
        dataFile.print(";");
        dataFile.print(data.sensors.temperature);
        dataFile.print(";");
        dataFile.print(data.sensors.pressure);
        dataFile.print(";");
        dataFile.print(data.sensors.humidity);
        dataFile.print(";");
        dataFile.print(data.sensors.altitude);
        dataFile.print(";");
        dataFile.print(data.luminosity.luminosity);
        dataFile.println();

        dataFile.close();
        Serial.println("\nSD card is working\n");
    }
    else
    {
        //modeVar.ledMode = ERROR_SD_ACCESS;
        Serial.println("\nError opening data.csv\n");
    }
    interrupts();

}

void printDataSerial()
{
    // Print data on the serial monitor
    Serial.print(data.rtc.year);
    Serial.print("-");
    Serial.print(data.rtc.month);
    Serial.print("-");
    Serial.print(data.rtc.day);
    Serial.print(" ");
    Serial.print(data.rtc.hour);
    Serial.print(":");
    Serial.print(data.rtc.minute);
    Serial.print(":");
    Serial.print(data.rtc.second);
    Serial.print(";");
    Serial.print(data.sensors.temperature);
    Serial.print(";");
    Serial.print(data.sensors.pressure);
    Serial.print(";");
    Serial.print(data.sensors.humidity);
    Serial.print(";");
    Serial.print(data.sensors.altitude);
    Serial.print(";");
    Serial.print(data.luminosity.luminosity);
    Serial.println();
    return;
}

void setup()
{
    Serial.begin(MONITOR_BAUD); // initialize the serial communication

    Wire.begin(); // initialize the I2C communication

    SD.begin(SD_PIN); // initialize the SD card

    clock.begin(); // initialize the RTC

    if (digitalRead(GBTN_PIN) == LOW)
        configMode(); // if the green button is pressed at startup, enter configuration mode

    pinMode(RBTN_PIN, INPUT); // define the red button pin as an input
    pinMode(GBTN_PIN, INPUT); // define the green button pin as an input

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
    if (digitalRead(RBTN_PIN) == HIGH && digitalRead(GBTN_PIN) == HIGH) // if red or green button is not pressed check if error, so we see if changing mode
    {
        if (modeVar.error == NO_ERROR)
            modeVar.ledMode = modeVar.actual;
        else
            modeVar.ledMode = modeVar.error;
    }
    else
        modeVar.ledMode = modeVar.actual;

    getData(); // get data from the sensors

    if (modeVar.actual == MAINTENANCE)
        printDataSerial();
    else
        storeData();

    delay(2000);

}