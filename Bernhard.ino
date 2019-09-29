//https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51
//https://learn.adafruit.com/adafruit-music-maker-featherwing
//https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code

// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <SD.h>
#include "Adafruit_VS1053.h"

#include <Wire.h>
#include <Adafruit_Sensor.h> // Adafruit Unified Sensors
#include <Adafruit_BNO055.h> // Adafruit_BNO055
#include <utility/imumaths.h>
  
#include "Types.h"


// Define pins for Adafruit_VS1053 Musik Maker (Feather M4)
#define Adafruit_VS1053_RESET   -1     // VS1053 reset pin (not used!)
#define Adafruit_VS1053_CS       6     // VS1053 chip select pin (output)
#define Adafruit_VS1053_DCS     10     // VS1053 Data/command select pin (output)
#define Adafruit_CARDCS          5     // Card chip select pin
#define Adafruit_VS1053_DREQ     9     // VS1053 Data request, ideally an Interrupt pin   // DREQ should be an Int pin *if possible* (not possible on 32u4)

uint32 timerFactor = 100;
uint32 sleepyInterval = 5 * timerFactor;
uint32 previousMillis = 0;

Adafruit_VS1053_FilePlayer filePlayer = Adafruit_VS1053_FilePlayer(Adafruit_VS1053_RESET, Adafruit_VS1053_CS, Adafruit_VS1053_DCS, Adafruit_VS1053_DREQ, Adafruit_CARDCS);
Adafruit_BNO055 bno055 = Adafruit_BNO055(55);

// the setup function runs once when you press reset or power the board
void setup() 
{
  setupMusikMaker(); 
  setupBNO055(); 
}

void setupMusikMaker()
{
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);  
  Serial.begin(115200);

  if(!filePlayer.begin()) 
  { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     digitalWrite(13, HIGH);  
     while (1);
  }
  else
  {
    Serial.println(F("VS1053 found"));
  }
 
  filePlayer.sineTest(0x44, 500); // Make a tone to indicate VS1053 is working
  
  if(!SD.begin(Adafruit_CARDCS)) 
  {
    Serial.println(F("SD failed, or not present"));
    digitalWrite(13, HIGH);  
    while (1);  // don't do anything more
  }
  else
  {
    Serial.println("SD OK!");
  }
  
  // list files
  printDirectory(SD.open("/"), 0);
  
  // Set volume for left, right channels. lower numbers == louder volume!
  filePlayer.setVolume(1,1);
  
  // If DREQ is on an interrupt pin we can do background
  // audio playing
  filePlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
  
  // Play a file in the background, REQUIRES interrupts!
  Serial.println(F("Playing WakeupTrack"));
  filePlayer.playFullFile("/wakeup.mp3");

  Serial.println(F("Playing track 002"));
  filePlayer.startPlayingFile("/track002.mp3");  
}

void setupBNO055() 
{
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno055.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);    
  bno055.setExtCrystalUse(true);
}

// the loop function runs over and over again forever
void loop() 
{
  Serial.println(F("Loop"));
  /*digitalWrite(13, HIGH);   
  delay(100);               
  digitalWrite(13, LOW);    
  delay(100);
  digitalWrite(13, HIGH);   
  delay(100);               
  digitalWrite(13, LOW);
  delay(1000);  
  */
  /*if(millis() - previousMillis > sleepyInterval)
  {
    filePlayer.startPlayingFile("/owling.mp3");
    delay(10000);
    filePlayer.startPlayingFile("/hit.mp3");
    delay(10000);
    filePlayer.startPlayingFile("/vomit.mp3");
    delay(10000);
    filePlayer.startPlayingFile("/sick.mp3");
    delay(10000);
    filePlayer.startPlayingFile("/snore.mp3");
    delay(10000);
    filePlayer.startPlayingFile("/vomit.mp3");
    delay(10000);
    filePlayer.startPlayingFile("/sorrow.mp3");
    delay(10000);
    previousMillis = millis();
  }*/

  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno055.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");

  if(event.orientation.x > 200)
  {
    Serial.print("sorrow");
    filePlayer.startPlayingFile("/sorrow.mp3");    
  }

  if(event.orientation.y > 50)
  {
    Serial.print("hit");
    filePlayer.startPlayingFile("/hit.mp3");    
  }
  
  delay(100);


  /*Serial.print(".");
  // File is playing in the background
  if (musicPlayer.stopped()) {
    Serial.println("Done playing music");
    while (1) {
      delay(10);  // we're done! do nothing...
    }
  }
  if (Serial.available()) {
    char c = Serial.read();
    
    // if we get an 's' on the serial console, stop!
    if (c == 's') {
      musicPlayer.stopPlaying();
    }
    
    // if we get an 'p' on the serial console, pause/unpause!
    if (c == 'p') {
      if (! musicPlayer.paused()) {
        Serial.println("Paused");
        musicPlayer.pausePlaying(true);
      } else { 
        Serial.println("Resumed");
        musicPlayer.pausePlaying(false);
      }
    }
  }
  delay(100);
  */           
}

void printDirectory(File directory, uint8 coutOfTabs) 
{
   while(true) 
   {     
     File entry =  directory.openNextFile();
     if(!entry) 
     {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for(uint8 i=0; i<coutOfTabs; i++)
     {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if(entry.isDirectory()) 
     {
       Serial.println("/");
       printDirectory(entry, coutOfTabs+1);
     } else 
     {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}


/*
 * // Specifically for use with the Adafruit Feather, the pins are pre-set here!

// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>

// These are the pins used
#define VS1053_RESET   -1     // VS1053 reset pin (not used!)

// Feather ESP8266
#if defined(ESP8266)
  #define VS1053_CS      16     // VS1053 chip select pin (output)
  #define VS1053_DCS     15     // VS1053 Data/command select pin (output)
  #define CARDCS          2     // Card chip select pin
  #define VS1053_DREQ     0     // VS1053 Data request, ideally an Interrupt pin

// Feather ESP32
#elif defined(ESP32)
  #define VS1053_CS      32     // VS1053 chip select pin (output)
  #define VS1053_DCS     33     // VS1053 Data/command select pin (output)
  #define CARDCS         14     // Card chip select pin
  #define VS1053_DREQ    15     // VS1053 Data request, ideally an Interrupt pin

// Feather Teensy3
#elif defined(TEENSYDUINO)
  #define VS1053_CS       3     // VS1053 chip select pin (output)
  #define VS1053_DCS     10     // VS1053 Data/command select pin (output)
  #define CARDCS          8     // Card chip select pin
  #define VS1053_DREQ     4     // VS1053 Data request, ideally an Interrupt pin

// WICED feather
#elif defined(ARDUINO_STM32_FEATHER)
  #define VS1053_CS       PC7     // VS1053 chip select pin (output)
  #define VS1053_DCS      PB4     // VS1053 Data/command select pin (output)
  #define CARDCS          PC5     // Card chip select pin
  #define VS1053_DREQ     PA15    // VS1053 Data request, ideally an Interrupt pin

#elif defined(ARDUINO_NRF52832_FEATHER )
  #define VS1053_CS       30     // VS1053 chip select pin (output)
  #define VS1053_DCS      11     // VS1053 Data/command select pin (output)
  #define CARDCS          27     // Card chip select pin
  #define VS1053_DREQ     31     // VS1053 Data request, ideally an Interrupt pin

// Feather M4, M0, 328, nRF52840 or 32u4
#else
  #define VS1053_CS       6     // VS1053 chip select pin (output)
  #define VS1053_DCS     10     // VS1053 Data/command select pin (output)
  #define CARDCS          5     // Card chip select pin
  // DREQ should be an Int pin *if possible* (not possible on 32u4)
  #define VS1053_DREQ     9     // VS1053 Data request, ideally an Interrupt pin

#endif


Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);

void setup() {
  Serial.begin(115200);

  // if you're using Bluefruit or LoRa/RFM Feather, disable the radio module
  //pinMode(8, INPUT_PULLUP);

  // Wait for serial port to be opened, remove this line for 'standalone' operation
  while (!Serial) { delay(1); }
  delay(500);
  Serial.println("\n\nAdafruit VS1053 Feather Test");
  
  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }

  Serial.println(F("VS1053 found"));
 
  musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working
  
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1);  // don't do anything more
  }
  Serial.println("SD OK!");
  
  // list files
  printDirectory(SD.open("/"), 0);
  
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(10,10);
  
#if defined(__AVR_ATmega32U4__) 
  // Timer interrupts are not suggested, better to use DREQ interrupt!
  // but we don't have them on the 32u4 feather...
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT); // timer int
#else
  // If DREQ is on an interrupt pin we can do background
  // audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
#endif
  
  // Play a file in the background, REQUIRES interrupts!
  Serial.println(F("Playing full track 001"));
  musicPlayer.playFullFile("/track001.mp3");

  Serial.println(F("Playing track 002"));
  musicPlayer.startPlayingFile("/track002.mp3");
}

void loop() {
  Serial.print(".");
  // File is playing in the background
  if (musicPlayer.stopped()) {
    Serial.println("Done playing music");
    while (1) {
      delay(10);  // we're done! do nothing...
    }
  }
  if (Serial.available()) {
    char c = Serial.read();
    
    // if we get an 's' on the serial console, stop!
    if (c == 's') {
      musicPlayer.stopPlaying();
    }
    
    // if we get an 'p' on the serial console, pause/unpause!
    if (c == 'p') {
      if (! musicPlayer.paused()) {
        Serial.println("Paused");
        musicPlayer.pausePlaying(true);
      } else { 
        Serial.println("Resumed");
        musicPlayer.pausePlaying(false);
      }
    }
  }
  delay(100);
}



/// File listing helper
void printDirectory(File dir, int numTabs) {
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}
*/
