  
  void GetWireless(){

  // If data comes in from XBee, send it out to serial monitor
    if (Serial2.available()) { 
      command = Serial2.readString();
    }

  // If data fomes in from the computer, send it to the Xbee
    if (Serial.available()){
      command = Serial.readString();
      Serial2.write(Serial.read());
    }
    
  }

