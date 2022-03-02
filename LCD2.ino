// Adafruit_ImageReader test for Adafruit ST7789 320x240 TFT Breakout for Arduino.
// Demonstrates loading images to the screen, to RAM, and how to query
// image file dimensions.
// Requires three BMP files in root directory of SD card:
// parrot.bmp, miniwoof.bmp and wales.bmp.
// As written, this uses the microcontroller's SPI interface for the screen
// (not 'bitbang') and must be wired to specific pins (e.g. for Arduino Uno,
// MOSI = pin 11, MISO = 12, SCK = 13). Other pins are configurable below.

#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SdFat.h>                // SD card & FAT filesystem library
#include <Adafruit_ImageReader.h> // Image-reading functions

// TFT display and SD card share the hardware SPI interface, using
// 'select' pins for each to identify the active device on the bus.

#define SD_CS    4 // SD card select pin
#define TFT_CS  10 // TFT select pin
#define TFT_DC   8 // TFT display/command pin
#define TFT_RST  9 // Or set to -1 and connect to Arduino RESET pin


SdFat                SD;         // SD card filesystem
Adafruit_ImageReader reader(SD); // Image-reader object, pass in SD filesys

Adafruit_ST7789      tft    = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
int32_t              width  = 0, // BMP image dimensions
                     height = 0;

void setup(void) {
  tft.init(240, 240);           // Init ST7789 320x240
  SD.begin(SD_CS, SD_SCK_MHZ(10));
  reader.drawBMP("/loser.bmp", tft, 0, 0);
}

void loop() {
  
}
