
// FUNCTION TO RUN A MOTOR. PARAPHRASED FROM POLULU LIBRARY
// dPins are directional pins attached to driver (digital high/low) pPin is the PWM motor power pin
// Thus, each motor requires 3 pins: two digital pins to determine direction and one PWM pin for power

  void RunMotor(int dPin1, int dPin2, int pPin, int power){
    if (power == 0){             //
      digitalWrite(dPin1, LOW);  // TURN THE MOTORS OFF
      digitalWrite(dPin2, LOW);  //
    }                            //
    else if (power < 0){         //
      digitalWrite(dPin1, LOW);  // REVERSE THE MOTOR
      digitalWrite(dPin2, HIGH); //
      power = -power;            // MUST WRITE A POSITIVE VALUE
    }                            //
    else {                       //
      digitalWrite(dPin1, HIGH); // RUN THE MOTOR IN THE NORMAL DIRECTION
      digitalWrite(dPin2, LOW);  //
    }                            //
    analogWrite(pPin, power);    // WRITE THE POWER TO THE MOTOR
  }                              //

// FUNCTION TO DRIVE THE ROBOT. TIME SAVER

  void Drive(int M2Power, int M3Power){
    RunMotor(M2DPinA, M2DPinB, M2PPin, M2Power);
    RunMotor(M3DPinA, M3DPinB, M3PPin, M3Power); 
  }

  void RunBelt(int power){
    RunMotor(M1DPinA, M1DPinB, M1PPin, power);
  }


// SAME AS DRIVE FUNCTION BUT STOPS AFTER SOME TIME

  void DriveForTime(int LeftPower, int RightPower, int Time){
    Drive(LeftPower, RightPower);
    delay(Time);
    Drive(0, 0);
  }  

// THESE FUNCTIONS TURN THE ROBOT LEFT OR RIGHT
  
  void TurnRight(int power){
    Drive(power, -power);
  }
  
  void TurnLeft(int power){
    Drive(-power, power);
  }

// FUNCTION TO STOP THE ROBOT

  void Stop(){
    leftPower = 0; rightPower = 0;
    Drive(leftPower, rightPower);
  }

