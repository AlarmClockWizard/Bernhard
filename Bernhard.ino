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

//settings
uint32 timerFactor = 100;
uint32 idleSoundBaseIntervall = 50 * timerFactor;
uint32 idleSoundRandomIntervall = 100  * timerFactor;
float32 maximalGravityVectorDifference = -50.0f;

uint32 timeOfLastIdleSound = millis();
uint32 idleSoundIntervall = idleSoundBaseIntervall;
bool wasHangingDown = false;
sensors_event_t initialGravityEvent;

Adafruit_VS1053_FilePlayer filePlayer = Adafruit_VS1053_FilePlayer(Adafruit_VS1053_RESET, Adafruit_VS1053_CS, Adafruit_VS1053_DCS, Adafruit_VS1053_DREQ, Adafruit_CARDCS);
Adafruit_BNO055 bno055 = Adafruit_BNO055(55);

// the setup function runs once when you press reset or power the board
void setup() 
{
  initialGravityEvent.acceleration.x = 12345;
  initialGravityEvent.acceleration.y = 12345;
  initialGravityEvent.acceleration.z = 12345;
  
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
}

void setupBNO055() 
{
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno055.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    filePlayer.sineTest(0x44, 500); 
    while(1);
  }
  
  delay(1000);    
  bno055.setExtCrystalUse(true);
}

float vectorLengthSquared(float vector[3])
{
    return vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2];
}

float vectorLength(float vec[3])
{
    return sqrt(vectorLengthSquared(vec));
}

float dotProduct(float vector0[3], float vector1[3])
{
    return vector0[0]*vector1[0] + vector0[1]*vector1[1] + vector0[2]*vector1[2];
}

bool isHangingDown()
{
  if(initialGravityEvent.acceleration.x == 0.0 || initialGravityEvent.acceleration.x == 12345)
  {
    Serial.print("taking gravity snapshot");
    bno055.getEvent(&initialGravityEvent, Adafruit_BNO055::VECTOR_GRAVITY);
  }
   
  sensors_event_t gravityEvent;
  bno055.getEvent(&gravityEvent, Adafruit_BNO055::VECTOR_GRAVITY);
  
  float gravity[3];
  gravity[0] = gravityEvent.acceleration.x;
  gravity[1] = gravityEvent.acceleration.y;
  gravity[2] = gravityEvent.acceleration.z;
  
  float initialGravity[3];
  initialGravity[0] = initialGravityEvent.acceleration.x;
  initialGravity[1] = initialGravityEvent.acceleration.y;
  initialGravity[2] = initialGravityEvent.acceleration.z;

  float dot = dotProduct(gravity, initialGravity); 
  bool hangingDown = dot < maximalGravityVectorDifference;

  Serial.print(" initialGx=");
  Serial.print(initialGravityEvent.acceleration.x);
  Serial.print(" initialGy=");
  Serial.print(initialGravityEvent.acceleration.y);
  Serial.print(" initialGz=");
  Serial.print(initialGravityEvent.acceleration.z);
  Serial.print(" gx=");
  Serial.print(gravityEvent.acceleration.x);
  Serial.print(" gy=");
  Serial.print(gravityEvent.acceleration.y);
  Serial.print(" gz=");
  Serial.print(gravityEvent.acceleration.z);
  Serial.print(" dot=");
  Serial.print(dot); 
  Serial.print("\n");

  return hangingDown;  
}

bool playIdleSound()
{
  if(millis() - timeOfLastIdleSound > idleSoundIntervall)
  {
    Serial.println(F("time to idle"));
    if(isHangingDown())
    {
      int32 randomIndex = random(0, 2);
      if(randomIndex == 0)
      {
        filePlayer.playFullFile("/sorrow.mp3");
      }
      if(randomIndex == 1)
      {
        filePlayer.playFullFile("/vomit.mp3");
      }
      if(randomIndex == 2)
      {
        filePlayer.playFullFile("/sick.mp3");
      }
    }
    else
    {
      int32 randomIndex = random(0, 2);
      if(randomIndex == 0)
      {
        filePlayer.playFullFile("/wakeup.mp3");
      }
      if(randomIndex == 1)
      {
        filePlayer.playFullFile("/owling.mp3");
      }
      if(randomIndex == 2)
      {
        filePlayer.playFullFile("/snore.mp3");
      }
    }
    idleSoundIntervall = idleSoundBaseIntervall + random(0, idleSoundRandomIntervall);
    timeOfLastIdleSound = millis();
  }
}

// the loop function runs over and over again forever
void loop() 
{
  if(isHangingDown())
  {
    if(wasHangingDown == false)
    {
      Serial.println(F("dumb bird down"));
      //filePlayer.playFullFile("/hit.mp3");   
      filePlayer.playFullFile("/shreek.mp3");  
      timeOfLastIdleSound = millis(); 
    }
  }
  else
  {
    if(wasHangingDown == true)
    {
      Serial.println(F("we are upright again"));
      filePlayer.playFullFile("/wakeup.mp3");
      timeOfLastIdleSound = millis(); 
    }
  }
  wasHangingDown = isHangingDown();
  playIdleSound();
    
  delay(500);    
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
