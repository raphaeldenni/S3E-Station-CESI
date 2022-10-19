// Structure file for the main program

struct modeVar
{
    int actual = STANDARD;   // define the mode of the program
    int previous = STANDARD; // define the previous mode of the program
    int rBtntimePressed = 0; // define the pin for the red button
    int gBtntimePressed = 0; // define the pin for the green button
    int ledMode = STANDARD;

};

struct luminosity
{
    float *luminosity = NULL;  // define the luminosity value
};

struct sensors
{
    float *temperature = NULL; // define the temperature value
    float *humidity = NULL;    // define the humidity value
    float *pressure = NULL;    // define the pressure value
    float *altitude = NULL;    // define the altitude value

};

struct gps
{
    float *latitude = NULL;  // define the latitude value
    float *longitude = NULL; // define the longitude value
    float *altitude = NULL;  // define the altitude value

};

struct rtc
{
    int *year; // define the year value
    int *month = NULL;   // define the month value
    int *day = NULL;     // define the day value
    int *hour = NULL;    // define the hour value
    int *minute = NULL;  // define the minute value
    int *second = NULL;  // define the second value

};

struct data
{
    struct luminosity luminosity;
    struct sensors sensors;
    struct gps gps;
    struct rtc rtc;
    
};

