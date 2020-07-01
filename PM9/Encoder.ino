
// THIS FUNCTION UPDATES MOTOR POSITIONS, CONVERTED TO REVOLUTIONS
  void GetEncoders(){
    
    leftCount  =  LeftEnc.read()/(gearRatio*countsPerRev);
    rightCount = RightEnc.read()/(gearRatio*countsPerRev);
  //beltCount  = BeltEnc.read()/(gearRatio*countsPerRev);

  }

// THIS FUNCTION UPDATES VELOCITIES
  void GetVelocity(){

    dt = t - tOldGV; // Change in time since last time this was run

 // Update velocity: velocity = change in position / change in time
    leftVel  = (leftCount  -  leftOld) / dt;
    rightVel = (rightCount - rightOld) / dt;
    //beltVel  = (beltCount  -  beltOld) / dt;

 // Update old values
    leftOld  = leftCount;
    rightOld = rightCount;
    //beltOld  = beltCount;
     
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
    
    if ((t - tOldDS) > DSDelay){                   // Only updates after a small delay, to help prevent overcorrection

      SetSpeeds(LVel, RVel);                       // Set approximate motor speeds
    
      if (abs(leftPower + leftAdjust) <= 255){      // If adjusted left power is within range
        if      ((leftVel - LVel) < -velThreshold)  // If the motor is going too slow (or fast if negative)
           leftAdjust++;                            // Increase left adjustment
        else if ((leftVel - LVel) >  velThreshold)  // If the motor is going too fast (or slow if negative)
           leftAdjust--;                            // Decrease left adjustment
      }

      if (abs(rightPower + rightAdjust) <= 255){    // If adjusted right power is within range
        if      ((rightVel - RVel) < -velThreshold) // If the motor is going too slow (or fast if negative)
           rightAdjust++;                           // Increase right adjustment
        else if ((rightVel - RVel) >  velThreshold) // If the motor is going too fast (or slow if negative)
           rightAdjust--;                           // Decrease right adjustment
      }

      // Drive each motor at its adjusted value
      Drive(leftPower + leftAdjust, rightPower + rightAdjust); 

      tOldDS = t; // Update old time
    }
  }


// THIS FUNCTION DRIVES BY USING CORRECTION WITH NO INITIAL GUESS

  void DriveAtSpeed2(double LVel, double RVel){
    
    if ((t - tOldDS) > DSDelay){                   // Only updates after a small delay, to help prevent overcorrection
      
      if (abs(leftPower + leftAdjust) <= 255){      // If left power is within range
        if      ((leftVel - LVel) < -velThreshold)  // If the motor is going too slow (or fast if negative)
           leftPower++;                             // Increase power
        else if ((leftVel - LVel) >  velThreshold)  // If the motor is going too fast (or slow if negative)
           leftPower--;                             // Decrease power
      }

      if (abs(rightPower + rightAdjust) <= 255){    // If right power is within range
        if      ((rightVel - RVel) < -velThreshold) // If the motor is going too slow (or fast if negative)
           rightPower++;                            // Increase power
        else if ((rightVel - RVel) >  velThreshold) // If the motor is going too fast (or slow if negative)
           rightPower--;                            // Decrease power
      }

      // Drive each motor at its adjusted value
      Drive(leftPower, rightPower); 

      tOldDS = t; // Update old time
    }
  }

// THIS FUNCTION DRIVES THE ROBOT IN A CIRCLE, TO THE LEFT OR RIGHT
  void Circle(){
      
      if (circRight){
         if (rightCount - origPosR > Rev && leftCount - origPosL > RevOut){  // If wheel has traveled perimeter of circle (in rev), stop!
            circRight = false; circLeft = false;
            message = "Circle Complete! (R)";
         }
            
         message = "Radius Right!";

         // Set wheel speeds. If the wheels are x cm apart from eachother, Outer speed = ((x + R) / R) * Inner speed
         DriveAtSpeed(slowSpeed*(R + distBetweenWheels)/R, slowSpeed);
      }

      else if (circLeft){
         if (leftCount - origPosL > Rev && rightCount - origPosR > RevOut){  // If wheel has traveled perimeter of circle (in rev), stop!
            circLeft = false; circRight = false;
            message = "Circle Complete! (L)";
         }
         
         message = "Radius Left!";

         // Same as right circle, but speed values are flipped
         DriveAtSpeed(slowSpeed, slowSpeed*(R + distBetweenWheels)/R);
      }
      else 
         Stop();
      
  }
