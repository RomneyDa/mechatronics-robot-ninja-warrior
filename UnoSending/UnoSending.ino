#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup()  
{
  Serial.begin(9600); // Open serial port

  Serial.println("Hello Computer!"); // Send message to computer

  mySerial.begin(9600); // Initialize wireless serial object 

  mySerial.print("X"); // Send message to other Arduino

}

void loop()
{

  if(Serial.available())
      mySerial.write(Serial.read());
  
  delay(2);

  if(mySerial.available())
      Serial.write(mySerial.read());
  
  delay(2);
  
}
