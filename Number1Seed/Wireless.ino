  
  void GetWireless(){

 // If data comes in from XBee, send it out to serial monitor
  
    if (Serial2.available()){
      command = Serial2.readString();   
    }
  }
