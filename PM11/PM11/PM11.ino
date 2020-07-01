/////////////////////////////
//  MECHATRONICS TEAM 8.1  //
//  PM11 Code              //
//  3/30/18                //
/////////////////////////////

/*
 *  This code uses commands from the xBee to activate various functions
 *  Almost every variable is global, and declared at the top. 
 *  All functions are practically instantaneous, for robustness
 */

#include <Encoder.h>

/////////////   PINS   //////////////
 
  #define M2PPin 11             
  #define M3PPin 12
  
  #define REncP1 18  
  #define REncP2 19
  #define LEncP1 20
  #define LEncP2 21
                     
  #define M2DPinA 30
  #define M2DPinB 31
  #define M3DPinA 32
  #define M3DPinB 33

//////////   VARIABLES   ////////////

  // ENCODER
    Encoder RightEnc(REncP1, REncP2);   // RIGHT MOTOR
    Encoder LeftEnc (LEncP1, LEncP2);   // LEFT MOTOR
    //Encoder BeltEnc (BEncP1, BEncP2); // BELT 
  
    #define gearRatio    70.0         // Motor gear ratio
    #define countsPerRev 64.0         // Motor counts per revolution of ungeared motor
    
    double rightCount = 0, rightOld = 0, leftCount = 0, leftOld = 0, beltCount = 0, beltOld = 0;  // Motor position variables
    double rightVel = 0, leftVel = 0, beltVel = 0, t = 0, tOldGV = 0, dt = 0;                     // Velocity variables

    double motorSpeed = 0, motorPower = 0, Vel = 0;

// THIS FUNCTION UPDATES MOTOR POSITIONS, CONVERTED TO REVOLUTIONS
  void GetEncoders(){
    
    leftCount  = LeftEnc.read()*2*5*3.14159/(gearRatio*countsPerRev);
    rightCount = RightEnc.read()*2*5*3.14159/(gearRatio*countsPerRev);
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

// SAME AS DRIVE FUNCTION BUT STOPS AFTER SOME TIME

  void DriveForTime(int LeftPower, int RightPower, int Time){
    Drive(LeftPower, RightPower);
    delay(Time);
    Drive(0, 0);
  }

///////////////  SETUP  /////////////////

void setup() {

  // INITIATE PINS
    //pinMode(M1DPinA, OUTPUT); pinMode(M1DPinB, OUTPUT); pinMode(M1PPin,  OUTPUT);
    pinMode(M2DPinA, OUTPUT); pinMode(M2DPinB, OUTPUT); pinMode(M2PPin,  OUTPUT);
    pinMode(M3DPinA, OUTPUT); pinMode(M3DPinB, OUTPUT); pinMode(M3PPin,  OUTPUT);

    pinMode(REncP1, INPUT); pinMode(REncP2, INPUT);
    pinMode(LEncP1, INPUT); pinMode(LEncP2, INPUT);
    //pinMode(BEncP1, INPUT); pinMode(BEncP2, INPUT);

    // INITIATE SERIAL PORTS
    Serial.begin(19200);       // COMPUTER
    Serial.println("Hello");  // STARTING MESSAGE
}

////////////////  LOOP  /////////////////

void loop(){
  
    t = micros()/1000000.0;  // Get current time
     
    GetEncoders();           // Update encoder positions. See Encoder tab
    GetVelocity();           // Update motor velocities. See Encoder tab

    //Motor command
    if ((t > 0) && (t <= 1)) {
      motorSpeed = 0;                                 
    }
    else if ((t > 1) && (t <= 3)){
      motorSpeed = 0.75;
    }
    else if ((t > 3) && (t <= 5)){
      motorSpeed = 0;
    }
    else 
      Serial.end();

    Vel = (leftVel + rightVel) / 2;

    motorPower = motorSpeed*255/10.13;
 
    Serial.print(t, 3);
    Serial.print("\t");
    Serial.print(Vel, 5);
    Serial.print("\t");
    Serial.print(motorSpeed, 5);
  
    Serial.println();
  
    Drive(motorPower, motorPower);

} 

