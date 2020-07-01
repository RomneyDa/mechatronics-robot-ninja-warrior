//////////////////////////////////////////////////////////////////////////////////
// MECHATRONICS ROBOT NINJA WARRIOR II COMPETITION CODE                         //
// TEAM 8.1: Layne Christensen, Dallin Romney, Tyler Barrett and Anthony Amiker //
// SPRING 2018                                                                  //
//////////////////////////////////////////////////////////////////////////////////

// SET PINS
  int M2PPin = 11;
  int M3PPin = 12;
                     
  int M2DPinA = 30;
  int M2DPinB = 31;
  int M3DPinA = 32;
  int M3DPinB = 33;

// OTHER INITIAL VARIABLES
int MPow = 200;      // Motor power

// FUNCTION TO RUN A MOTOR. PARAPHRASED FROM POLULU LIBRARY
// dPins are directional pins attached to driver (digital high/low) pPin is the PWM motor power pin
// Thus, each motor requires 3 pins: two digital pins to determine direction and one PWM pin for power

  void RunMotor(int dPin1, int dPin2, int pPin, int power){
    if (power == 0){
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
  }

// FUNCTION TO DRIVE THE ROBOT. ANOTHER TIME SAVER
  void Drive(int M2Power, int M3Power){
    RunMotor(M2DPinA, M2DPinB, M2PPin, M2Power);
    RunMotor(M3DPinA, M3DPinB, M3PPin, M3Power); 
  }

// SAME AS THE DRIVE FUNCTION BUT STOPS AFTER A CERTAIN TIME
  void DriveForTime(int LeftPower, int RightPower, int Time){
    Drive(LeftPower, RightPower);
    delay(Time);
    Drive(0, 0);
  }
  
// SETUP
  void setup()
  {
    //INITIATE PINS
      pinMode(M2DPinA, OUTPUT);
      pinMode(M2DPinB, OUTPUT);
      pinMode(M2PPin,  OUTPUT);
      pinMode(M3DPinA, OUTPUT);
      pinMode(M3DPinB, OUTPUT);
      pinMode(M3PPin,  OUTPUT);

  }

void loop()
{
    DriveForTime(MPow, MPow, 2000);
    delay(500);
    DriveForTime(-MPow, -MPow, 2000);
    delay(500);
    DriveForTime(-MPow, MPow, 2000);
    delay(500);
    DriveForTime(MPow, -MPow, 2000);
    delay(500);

}
