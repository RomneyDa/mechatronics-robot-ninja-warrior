
// THIS FUNCTION UPDATES THE REFLECTANCE ARRAY SENSOR VALUES
  void GetLine(){
    
    // Read raw reflectance sensor values. Range is 0-2500, 0 = white, 2500 = black
    qtrrc.read(sensorValues);
    
    num = denom = lowSum = lowCounter = 0;            // Reset sums & counters


    // This for loop and the following statement finds an average of values considered white
    for (i = 0; i < NUM_SENSORS; i++){
      if (sensorValues[i] < WHITE){                   // If it's reading white
        lowSum += sensorValues[i];                    // Sum all the white values
        lowCounter ++;                                // Count it
      }
    }
    lowAverage = lowSum/lowCounter;                   // Average the white values

    // This loop de-biases the values by subtracting that average
    for (i = 0; i < NUM_SENSORS; i++){                // For each sensor
      sensorCorrected = sensorValues[i] - lowAverage; // Remove the sensor bias
      
      num += i*sensorCorrected;                       // Weighted sum of all unbiased sensor values
      denom += sensorCorrected;                       // Sum of all unbiased sensor values
    }
  
    pos = num/denom;                                               // Centroid of values
    dist = (pos - (double(NUM_SENSORS) - 1)/2)*distBetweenSensors; // Convert to distance from center of array

  }


// THIS FUNCTION FOLLOWS A LINE!

  void LineFollow(){

      GetLine();                               // UPDATE SENSOR VALUES
      
      if (isnan(dist) || dist > 100) {         // If there's no line       
        Drive(0, 0);                           // Stop
        message = "(LF) Stopped - No Line!"; 
      }
        
      else {                                   // LINE FOLLOW MODE
        if (dist > thresholdDist) {            // If the robot is to the right of the line
          TurnLeft(MPow);           // Turn left
          message = "(LF) Turn Left";
        }
        else if (dist < -thresholdDist){       // If the robot is to the left of the line   
          TurnRight(MPow);          // Turn right
          message = "(LF) Turn Right";  
        }
        else {                                 // Otherwise
          Drive(MPow, MPow); // Go Straight
          message = "(LF) Drive Straight";
        }
      }
  }

  
