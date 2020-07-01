  
  void GetHall(){

    message = "Reading Hall Sensor";

    hall = analogRead(hallSensorPin);
    hallScaled = (hall - hallMid)*100.0/hallMid;    // Scale from -100 to 100 ish
    
    if (hall > hallMid*(1 + hallTol) || hall < hallMid*(1 - hallTol))
        hallOn = true;
    else 
        hallOn = false;
 
  }
