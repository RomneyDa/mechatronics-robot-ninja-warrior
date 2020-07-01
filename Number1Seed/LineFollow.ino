void LineFollow(){
  
      if (noLine)         // If there's no line       
        Drive(0, 0);                         // Stop
        
      else {                                 // LINE FOLLOW MODE
        if (dist > thresholdDist)            // If the robot is to the right of the line
          TurnLeft(MPow);                    // Turn left
        
        else if (dist < -thresholdDist)     // If the robot is to the left of the line   
          TurnRight(MPow);                   // Turn right
 
        else                                 // Otherwise
          Drive(MPow, MPow);                 // Go Straight
       
      }
}
void LineFollowPD(){
  
        LFerror = (dist - 0);
        LFvel = (LFerror - LFerror_old)/(t - t_oldLF);
        LFAdjust = KpLF*LFerror + KdLF*LFvel;
      
        if (noLine)                                     // If theres no line
        Drive(0, 0);                                     // Stop!
        
        else {
          if (LFerror < 0)                           // If robot is to the left of the line
            Drive(MPow, MPow + LFAdjust);                   // Turn Right!
        
          else if (LFerror > 0)                           // If robot is to the right of the line
            Drive(MPow - LFAdjust, MPow);                   // Turn Left!
        
          else                                            // Otherwise
            Drive(MPow, MPow);                              // Go Straight!
        }
        
        LFerror_old = LFerror;
        t_oldLF = t;
}

void UTState(){
 
  if (monkeyPos != railsUp)           // First, put the rails assembly up
    MoveRails(railsUp);

  if (t - tLDelay > 2.5){
    RunBelt(95);
    MPow = 60;   
  }
  
  if (noLine && t - tLDelay > 3){    // If no line, transition!
    tMDelay = t;
    pastLine = false;
    driveHard = false;
    stage = 4;
  } 
  else
    LineFollow();                     // Otherwise, follow the line
}

void GetLine(){
    
    // Read raw reflectance sensor values. Range is 0-2500, 0 = white, 2500 = black
    qtrrc.read(sensorValues);
    
    num = denom = lowSum = lowCounter = SUM = 0;            // Reset sums & counters
 

    for (i = 0; i < NUM_SENSORS - 1; i++){
        SUM += sensorValues[i];                    // Sum all the white values                               // Count it
    }
    sensorValues[7] = SUM / 7;
    
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

    noLine = (isnan(dist) || isinf(dist));

  }

