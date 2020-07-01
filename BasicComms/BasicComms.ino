// This code is used to communicate between two arduinos with XBees

void setup()  
{
  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);

  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");

  // Open serial communications with the other Arduino board
  Serial3.begin(9600);

  // Send a message to the other Arduino board
  Serial3.write("Hello other Arduino!");

}

double t = 0;
String command = "";

void loop() // run over and over
{
    t = millis();

    if (Serial3.available()>0) { 
      Serial.write(Serial3.read()); 
    }

  // If data comes in from the computer, send it to the Xbee
    if (Serial.available()>0){
      Serial3.write(Serial.read());
    }

    //Serial.println(command);
    //delay(50);

    Serial3.println("hello");
    delay(100);
}
