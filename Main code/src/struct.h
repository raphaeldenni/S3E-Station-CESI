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
    float luminosity = 0; // define the luminosity value
    float temperature = 0; // define the temperature value
    float humidity = 0;   // define the humidity value
    float pressure = 0;   // define the pressure value

};

struct gpsData
{
    float latitude;  // define the latitude value
    float longitude; // define the longitude value
    float altitude;  // define the altitude value

};

struct rtcData
{
    int year;   // define the year value
    int month;  // define the month value
    int day;    // define the day value
    int hour;   // define the hour value
    int minute; // define the minute value
    int second; // define the second value

};


