// Worldwide Weather Watcher Project
// 22/10/2022

// Group B
// DENNI Raphaël
// HOUILLE Lukas
// ROUAS Léo

#include <dependencies.h> // include all the libraries and the defines

ChainableLED leds(LED_PIN, LED_DATA_PIN, LEDS_NUM); // initialize the LED strip

Adafruit_BME280 bme; // initialize I2C communication for the bme280 sensor

SoftwareSerial ss(RX_PIN_TO_GPS, TX_PIN_TO_GPS); // initialize the pins for the GPS module

TinyGPSPlus gps; // initialize the GPS module

DS1307 clock; // initialize the RTC module

struct config config; // initialize the config structure

struct modeVar modeVar; // initialize the mode structure

struct data data; // initialize the data structure


void configLoad()
{
    // Read config file
    File dataFile = SD.open("config.txt", FILE_WRITE);

    if (dataFile && digitalRead(RBTN_PIN) != LOW)
    {
        // Attribute the value of the config file to the config structure
        leds.setColorRGB(YELLOW);

        int buffer[8];

        config.logInterval = dataFile.read(buffer, 2);
        config.timeout = dataFile.read(buffer, 2);

        config.year = dataFile.read(buffer, 4);
        config.month = dataFile.read(buffer, 2);
        config.day = dataFile.read(buffer, 2);

        config.hour = dataFile.read(buffer, 2);
        config.minute = dataFile.read(buffer, 2);
        config.second = dataFile.read(buffer, 2);

        config.lumin = dataFile.read();

        config.temp = dataFile.read();
        config.tempLow = dataFile.read(buffer, 3);
        config.tempHigh = dataFile.read(buffer, 3);

        config.press = dataFile.read();
        config.pressLow = dataFile.read(buffer, 4);
        config.pressHigh = dataFile.read(buffer, 4);

        config.hum = dataFile.read();
        
    }
    else
    {
        modeVar.ledMode = ERROR_SD_FULL;

    };

    dataFile.close();

    if (digitalRead(GBTN_PIN) == LOW)
    {
        leds.setColorRGB(YELLOW);

        clock.fillByYMD(config.year, config.month, config.day);
        clock.fillByHMS(config.hour, config.minute, config.second);

        clock.setTime();

    };

    if (config.logInterval < 600000 && DEBUG_INTERVAL == false)
        config.logInterval = 600000;

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
        if (modeVar.rBtntimePressed > (62500 * TIME_TO_SWITCH / LED_UPDATE_INTERVAL))
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
        if (modeVar.gBtntimePressed > (62500 * TIME_TO_SWITCH / LED_UPDATE_INTERVAL))
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
        DEBUG_SERIAL.println("ERROR: Year data incoherence");

    };

    // Check month data
    if (data.rtc.month < 1 || data.rtc.month > 12)
    {
        modeVar.error = ERROR_DATA_INCOHERENCE;
        DEBUG_SERIAL.println("ERROR: Month data incoherence");
        
    };

    // Check day data
    if (data.rtc.day < 1 || data.rtc.day > 31)
    {
        modeVar.error = ERROR_DATA_INCOHERENCE;
        DEBUG_SERIAL.println("ERROR: Day data incoherence");
        
    };

    // Luminosity data
    if (config.lumin == 1)
    {
        data.luminosity.luminosity = analogRead(LUM_DATA_PIN) * 100.0 / 1023.0; // get the luminosity value
        
        // Check luminosity data
        if (data.luminosity.luminosity < 0 || data.luminosity.luminosity > 100)
        {
        modeVar.error = ERROR_DATA_INCOHERENCE;
        DEBUG_SERIAL.println("ERROR: Luminosity data incoherence");
        
        };
    
    };
    

    // Sensors data
    if (bme.begin(BME_ADDRESS) || bme.begin(BME_ADDRESS_ALT))
    {
        // Temperature data
        if (config.temp == 1)
        {
            data.sensors.temperature = bme.readTemperature();                // get the temperature data
            
            // Check temperature data
            if (data.sensors.temperature < config.tempLow || data.sensors.temperature > config.tempHigh)
            {
                modeVar.error = ERROR_DATA_INCOHERENCE;
                DEBUG_SERIAL.println("ERROR: Temperature data incoherence");

            };

        };

        // Pressure data
        if (config.press == 1)
        {
            data.sensors.pressure = bme.readPressure()/100.0F;               // get the pressure data

            // Check pressure data
            if (data.sensors.pressure < config.pressLow || data.sensors.pressure > config.pressHigh)
            {
                modeVar.error = ERROR_DATA_INCOHERENCE;
                DEBUG_SERIAL.println("ERROR: Pressure data incoherence");

            };
            
        };

        // Humidity data
        if (config.hum == 1)
        {
            data.sensors.humidity = bme.readHumidity();                      // get the humidity data

            // Check humidity data
            if (data.sensors.humidity < 0 || data.sensors.humidity > 100)
            {
                modeVar.error = ERROR_DATA_INCOHERENCE;
                DEBUG_SERIAL.println("ERROR: Humidity data incoherence");

            };       
                
        };

    }
    else
    {
        modeVar.error = ERROR_CAPTOR_ACCESS;
        DEBUG_SERIAL.println("ERROR: BME280 access");

    }
    

    // GPS data
    while (ss.available() > 0)
    {
        if (gps.encode(ss.read()))
        {
            data.gps.latitude = gps.location.lat();    // get the latitude data
            data.gps.longitude = gps.location.lng();   // get the longitude data
            data.gps.altitude = gps.altitude.meters(); // get the altitude data

            // Check latitude data
            if (data.gps.latitude < -90 || data.gps.latitude > 90)
            {
                modeVar.error = ERROR_DATA_INCOHERENCE;
                DEBUG_SERIAL.println("ERROR: Latitude data incoherence");

            };

            // Check longitude data
            if (data.gps.longitude < -180 || data.gps.longitude > 180)
            {
                modeVar.error = ERROR_DATA_INCOHERENCE;
                DEBUG_SERIAL.println("ERROR: Longitude data incoherence");

            };

            // Check altitude data
            if (data.gps.altitude < -1000 || data.gps.altitude > 10000)
            {
                modeVar.error = ERROR_DATA_INCOHERENCE;
                DEBUG_SERIAL.println("ERROR: Altitude data incoherence");

            };

        }
        else
        {
            modeVar.ledMode = ERROR_GPS;

        };
    };
    
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
        dataFile.print(data.luminosity.luminosity);
        dataFile.print(";");
        dataFile.print(data.gps.latitude);
        dataFile.print(",");
        dataFile.print(data.gps.longitude);
        dataFile.print(";");
        dataFile.print(data.gps.altitude);
        dataFile.println();

        dataFile.close();

    }
    else
    {
        modeVar.error = ERROR_SD_FULL;

    }

    interrupts();

}


void printDataSerial()
{
    // Print data on the serial monitor
    Serial.println("===================");
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
    Serial.println(data.rtc.second);

    Serial.println(data.luminosity.luminosity);

    Serial.println(data.sensors.temperature);

    Serial.println(data.sensors.pressure);

    Serial.println(data.sensors.humidity);

    Serial.println(data.gps.latitude);

    Serial.println(data.gps.longitude);

    Serial.println(data.gps.altitude);

}


void setup()
{
    Serial.begin(MONITOR_BAUD); // initialize the serial communication

    Wire.begin(); // initialize the I2C communication

    SD.begin(SD_PIN); // initialize the SD card

    clock.begin(); // initialize the RTC

    //configLoad(); // load custom configuration

    delay(2500); // wait 2.5 second

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
    // Check if the red or green button is pressed and enter the corresponding mode
    // If a button is not pressed check for error
    if (digitalRead(RBTN_PIN) == HIGH || digitalRead(GBTN_PIN) == HIGH) 
    {
        if (modeVar.error == NO_ERROR)
            modeVar.ledMode = modeVar.actual;

        else
            modeVar.ledMode = modeVar.error;

    }
    else modeVar.ledMode = modeVar.actual;

    getData(); // get data from the sensors

    if (modeVar.actual == MAINTENANCE) 
    {
        printDataSerial();  // print data on the serial monitor
    
    }
    else
        storeData(); // store data in SD card

    if (DEBUG_INTERVAL == true)
        delay(config.logInterval); // wait for the next log interval (debug)

    else 
        delay(config.logInterval*1000*60); // wait for the next log interval
        

}