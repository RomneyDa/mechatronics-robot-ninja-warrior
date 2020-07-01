
// THIS FUNCTION UPDATES MOTOR POSITIONS, CONVERTED TO REVOLUTIONS
  void GetEncoders(){
    
    leftCount  =  LeftEnc.read()/(gearRatio*countsPerRev);
    rightCount = RightEnc.read()/(gearRatio*countsPerRev);
    
  }

// THIS FUNCTION UPDATES VELOCITIES
  void GetVelocity(){

    dt = t - tOldGV; // Change in time since last time this was run

 // Update velocity: velocity = change in position / change in time
    leftVel  = (leftCount  -  leftOld) / dt;
    rightVel = (rightCount - rightOld) / dt;

 // Update old values
    leftOld  = leftCount;
    rightOld = rightCount;
     
    tOldGV = t; // Update old time
  }

// THIS FUNCTION CONVERTS SPEED TO MOTOR POWER FOR THE LEFT MOTOR
// Q coefficients are derived from measurements and a cubic fit

  double Speed2Power(double Speed){ 
    return Q1*pow(Speed, 3) + Q2*pow(Speed, 2) + Q3*Speed + Q4;
  }

// UPDATES MOTOR POWERS TO AN APPROXIMATION OF CORRECT POWER. IT DOESN'T RUN THE MOTORS

  void SetSpeeds(double leftSpeed, double rightSpeed){
    
    if (leftSpeed < 0) j = -1;                   // Conversion functions only work for positive values (cubic)
    else               j =  1;
    
    leftPower = j*Speed2Power(abs(leftSpeed));   // Convert speed to power using measured relationship
    
    if (rightSpeed < 0) j = -1;
    else                j =  1;

    SL = Speed2Power(abs(rightSpeed));           // First, calculate right speed the same way as left
    rightPower = j*(R1*pow(SL, 2) + R2*SL + R3); // Convert left power to equivalent right power
                                                 // R coeffecients are derived from measurements
  }

// THIS FUNCTION DRIVES EACH WHEEL AT A CERTAIN SPEED BY CONSTANTLY CORRECTING an initial guess

  void DriveAtSpeed(double LVel, double RVel){
    
    //if ((t - tOldDS) > DSDelay){                  // Only updates after a small delay, to help prevent overcorrection

      SetSpeeds(LVel, RVel);                        // Set approximate motor speeds
    
        if ((leftVel - LVel) < -velThreshold)       // If the motor is going too slow (or fast if negative)
            leftAdjust++;                           // Increase left adjustment
            
        else if ((leftVel - LVel) >  velThreshold)  // If the motor is going too fast (or slow if negative)
            leftAdjust--;                           // Decrease left adjustment

        if ((rightVel - RVel) < -velThreshold)      // If the motor is going too slow (or fast if negative)
            rightAdjust++;                          // Increase right adjustment
            
        else if ((rightVel - RVel) >  velThreshold) // If the motor is going too fast (or slow if negative)
            rightAdjust--;                          // Decrease right adjustment

      leftPower += leftAdjust;
      rightPower += rightAdjust;

      leftPower = constrain(leftPower, -255, 255);
      rightPower = constrain(rightPower, -255, 255);
           
      // Drive each motor at its adjusted value
      Drive(leftPower, rightPower); 

      tOldDS = t; // Update old time
    //}
  }

  // DRIVE A CERTAIN DISTANCE. In this case, R is a straight distance
  void DriveDistance(double R){
    
    // This part initiates the distance values
    if (!startedDriving){
        Rev = R/(PI*wheelDiam) - distAdjust;      // Convert that distance to wheel revolutions
  
        origDistPos = leftCount;      // Record current position. Could also use rightCount (it's a straight line)
        leftAdjust = rightAdjust = 0; // Reset motor adjustments
        
        startedDriving = true;       // Update command so that it will get out of this loop and drive with given values
    }
  
    // This part drives until the given distance is reached
    else {               
      if (R == 0 || leftCount - origDistPos >= Rev)    // If distance is reached, stop  
          Stop();
      else 
          DriveAtSpeed(driveDistSpeed, driveDistSpeed); // Otherwise, keep going
    }
  }
