// Structure file for the main program

struct config
{
    int logInterval = 1000; // define the interval between each log
    int timeout = 30;       // define the timeout for the GPS module

    int year = 2000; // define the year
    int month = 1;   // define the month
    int day = 1;     // define the day

    int hour = 0;   // define the hour
    int minute = 0; // define the minute
    int second = 0; // define the second

    int lumin = 1; // define the luminosity mode

    int temp = 1;      // define the temperature mode
    int tempLow = -50; // define the temperature minimum
    int tempHigh = 50; // define the temperature maximum

    int press = 1;        // define the pressure mode
    int pressLow = 800;   // define the pressure minimum
    int pressHigh = 1200; // define the pressure maximum

    int hum = 1; // define the humidity mode

};

struct modeVar
{
    int actual = STANDARD;   // define the mode of the program
    int previous = STANDARD; // define the previous mode of the program
    int error = NO_ERROR;    // define the error write in other module
    int rBtntimePressed = 0; // define the pin for the red button
    int gBtntimePressed = 0; // define the pin for the green button
    int ledMode = STANDARD;  // define the mode of the LEDs

};

struct luminosity
{
    float luminosity; // define the luminosity value

};

struct sensors
{
    float temperature; // define the temperature value
    float humidity;    // define the humidity value
    float pressure;    // define the pressure value

};

struct gps
{
    float latitude;  // define the latitude value
    float longitude; // define the longitude value
    float altitude;  // define the altitude value

};

struct rtc
{
    int year; // define the year value
    int month;   // define the month value
    int day;     // define the day value
    int hour;    // define the hour value
    int minute;  // define the minute value
    int second;  // define the second value

};

struct data
{
    struct luminosity luminosity;
    struct sensors sensors;
    struct gps gps;
    struct rtc rtc;
    
};
