#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

//////////////////////////
// INPUT GUIDE:         //
//   B: Blink LED       //
//////////////////////////

// Variables
int ledDelay = 250;
int ledPin = 13;
char var = 'A';

void setup()  
{
  Serial.begin(9600);
  mySerial.begin(9600); // Open serial port with xbee

  pinMode(ledPin, OUTPUT);
}

void loop()
{

// Read Serial Buffer

  if (mySerial.available()){
    var = char(mySerial.read()); // Set the state variable (will be last character)
    Serial.print(var);
    delay(3);
    if(!mySerial.available()){   // Print all the other characters
      Serial.println();
    }
  }

  if (var == 'B'){
    digitalWrite(ledPin, HIGH);
    delay(ledDelay);
    digitalWrite(ledPin, LOW);
    delay(ledDelay);
    
    digitalWrite(ledPin, HIGH);
    delay(ledDelay);
    digitalWrite(ledPin, LOW);
    delay(ledDelay);
    
    digitalWrite(ledPin, HIGH);
    delay(ledDelay);
    digitalWrite(ledPin, LOW);
  }

}
