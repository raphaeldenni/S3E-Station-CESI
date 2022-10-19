// Dependencies file

#include <stdlib.h>

#include <Arduino.h>        // Arduino library
#include <Wire.h>           // I2C library
#include <SPI.h>            // Serial Peripheral Interface library for communication with the SD card and GPS module
#include <SD.h>             // Library for the SD card
#include <SoftwareSerial.h> // Library for the GPS module

#include <ChainableLED.h>       // Library for the LED strip
#include <Adafruit_Sensor.h>    // Library for the BME280 sensor
#include <Adafruit_I2CDevice.h> // Library for the BME280 sensor
#include <Adafruit_BME280.h>    // Library for the BME280 sensor
#include <TinyGPSPlus.h>        // Library for the GPS module
#include "DS1307.h"             // Library for the RTC module

#include <config.h> // Configuration file
#include <define.h> // Define file
#include <struct.h> // Structure file