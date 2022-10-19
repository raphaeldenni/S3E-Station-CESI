// Structure file for the main program

struct modeVar
{
    int actual = STANDARD;   // define the mode of the program
    int previous = STANDARD; // define the previous mode of the program
    int rBtntimePressed = 0; // define the pin for the red button
    int gBtntimePressed = 0; // define the pin for the green button
    int ledMode = STANDARD;

};

struct sensorsData
{
    float luminosity = 0;  // define the luminosity value
    float temperature = 0; // define the temperature value
    float humidity = 0;    // define the humidity value
    float pressure = 0;    // define the pressure value
    float altitude = 0;    // define the altitude value

};

struct gpsData
{
    float latitude;  // define the latitude value
    float longitude; // define the longitude value
    float altitude;  // define the altitude value

};

struct rtcData
{
    int year = 2000; // define the year value
    int month = 1;   // define the month value
    int day = 1;     // define the day value
    int hour = 0;    // define the hour value
    int minute = 0;  // define the minute value
    int second = 0;  // define the second value

};


