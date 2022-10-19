// Define file for the main program

#define GBTN_PIN 2 // define the pin for the green button
#define RBTN_PIN 3 // define the pin for the red button

#define LED_PIN 5                 // define the alimentation pin for the LED
#define LED_DATA_PIN 6            // define the data pin for the LED
#define LEDS_NUM 1                // number of LEDs in the chain
#define LED_UPDATE_INTERVAL 31250 // define the time between each LED update 62500 = 1s, Value is between 0 and 62500

#define LUM_DATA_PIN A0   // luminosity sensor data pin
#define SECOND_LUM_PIN A1 // second luminosity sensor pin

#define BME_ADDRESS 0x76             // define the BME280 sensor address
#define BME_ADDRESS_ALT 0x77 // define the BME280 sensor address
#define REFRESH_DELAY 1500           // define the delay between each refresh of the data
#define SEALEVELPRESSURE_HPA 1024.90 // define the sea level pressure

#define SD_PIN 4 // define the pin for the SD card

#define TX_PIN_TO_GPS 8   // define the TX pin for the GPS module
#define RX_PIN_TO_GPS 7   // define the RX pin for the GPS module
#define GPS_BAUD 9600     // define the baud rate for the GPS module

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

#define RED 0, 255, 0, 0       // define the value of the red color
#define GREEN 0, 0, 255, 0     // define the value of the green color
#define BLUE 0, 0, 0, 255      // define the value of the blue color
#define YELLOW 0, 255, 255, 0  // define the value of the yellow color
#define ORANGE 0, 255, 50, 0   // define the value of the orange color
#define WHITE 0, 255, 255, 255 // define the value of the white color