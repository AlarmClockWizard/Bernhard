//https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51/setup

// the setup function runs once when you press reset or power the board
void setup() 
{
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
}
 
// the loop function runs over and over again forever
void loop() 
{
  digitalWrite(13, HIGH);   
  delay(100);               
  digitalWrite(13, LOW);    
  delay(100);
  digitalWrite(13, HIGH);   
  delay(100);               
  digitalWrite(13, LOW);
  delay(1000);             
}
