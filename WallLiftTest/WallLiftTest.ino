////////////////////////////
//  MECHATRONICS TEAM 8.1 //
//  PM12 Code             //
//  4/2/18                //
////////////////////////////

/*
 *  This code uses commands from the xBee to activate various functions
 *  Almost every variable is global, and declared at the top. 
 *  All functions are practically instantaneous, for robustness
 */


#include <QTRSensors.h> 
#include <Encoder.h>
#include <Servo.h>

///////////   COMMANDS   ////////////
  #define CMD_DoNothing  "A"  // Do nothing
  #define CMD_Belt       'b'  // Run belt at a certain power, e.g., b150 = 150/255 power
  #define CMD_Dist       "D"  // Check Distance Sensors
  #define CMD_Hall       "H"  // Check Hall Sensor
  #define CMD_LineFollow "L"  // Line Follow
  #define CMD_Monkey     'm'  // Adjust Monkey Rails to position e.g. M180 = 180 degrees (all the way down)
  #define CMD_MonkeyFwd  "MF" // Move monkey rails to forward position
  #define CMD_MonkeyUp   "MU" // Move monkey rails to up position
  #define CMD_MonkeyBack "MB" // Move monkey rails to back position
  #define CMD_SerialOff  "p"  // Stop printing - will be much faster
  #define CMD_SerialOn   "P"  // Start printing - will delay significantly 
  #define CMD_RadiusL    'R'  // Drive a circle to the left,  e.g., R100 = 100 cm
  #define CMD_RadiusR    'r'  // Drive a circle to the right, e.g., r90  = 90 cm
  #define CMD_FollowingW "w"  // Following the wall
  #define CMD_WallFollow "W"  // Follow a wall
  #define CMD_DriveDist  'y'  // Drive a distance, e.g., y100 = drive 100 cm
  #define CMD_DriveSpeed 'z'  // Drive a speed, in tenths of an RPS e.g, z11 = 1.1 RPS

  // Placeholders - Don't send these commands, it will set them automatically
  #define CMD_MoveBelt   "B" //
  #define CMD_Circle     "C" // Doing circles
  #define CMD_MoveRails  "M" // Calls after monkey rails is called
  #define CMD_DrivingD   "Y" // Driving for a distance
  #define CMD_DrivingS   "Z" // Driving at a speed
  
/////////////   PINS   //////////////
 
  #define M2PPin 11             
  #define M3PPin 12
  #define M1PPin 13
  
  #define REncP1 18  
  #define REncP2 19
  #define LEncP1 20
  #define LEncP2 21
  
  //#define BEncP1 14  
  //#define BEncP2 15

  #define M1DPinA 28
  #define M1DPinB 29
  #define M2DPinA 30
  #define M2DPinB 31
  #define M3DPinA 32
  #define M3DPinB 33

  #define hallSensorPin  A11
  #define frontSensorPin A14
  #define sideSensorPin  A15

  #define railsServoLPin 44
  #define railsServoRPin 46

//////////   VARIABLES   ////////////

  // DISTANCE
    #define stopDistance 10         // Stop with a wall this far in front (cm)
    #define frontMax     30         // Maximum front sensor distance (cm)
    #define sideMax      15         // Maximum side sensor distance (cm)

    int Cfront = 2500, Bfront = -1; // Front distance sensor calibration constants (D = CV^B)
    int Cside  = 1000, Bside  = -1; // Side  distance sensor calibration constants (D = CV^B)
    
    int    Vfront, Vside;           // Global distance variables
    double Dfront, Dside;

  // LINE FOLLOW
    #define NUM_SENSORS 8     // Number of sensors used on reflectance array
    #define TIMEOUT     2500  // Waits for 2500 micro seconds
    #define EMITTER_PIN 2     // Emitter is controlled by digital 2 pin
    
    QTRSensorsRC qtrrc((unsigned char[]) { 3, 4, 5, 6, 7, 8, 9, 10 }, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
  
    double MPow = 60;           // Motor power out of 255 for Line Follow
    #define WHITE 1000.0       // Sensor reading for white. Function of distance from surface
    
    #define thresholdDist      0.4     // Distance from center of reflectance array to initiate turning, cm
    #define distBetweenSensors 0.9428  // Fixed distance between sensors on the array (measured)
    
    unsigned int sensorValues[NUM_SENSORS];
    double denom = 0, num = 0, pos = 0, lowSum = 0, lowAverage = 0, lowCounter = 0, sensorCorrected = 0, dist = 0; 

  // ENCODER
    Encoder RightEnc(REncP1, REncP2);   // RIGHT MOTOR
    Encoder LeftEnc (LEncP1, LEncP2);   // LEFT MOTOR
    //Encoder BeltEnc (BEncP1, BEncP2); // BELT 
  
    #define gearRatio    70.0         // Motor gear ratio
    #define countsPerRev 64.0         // Motor counts per revolution of ungeared motor

    double distBetweenWheels = 20.6;  // Distance between front wheels, cm
    double velThreshold = 0.005;       // Velocity correction accuracy, RPS
    double VMax = 2.1;                // Max wheel speed, RPS

    // Measured relationships for relationships between power and velocity
    // Q = coefficients of cubic fit of left motor power (0-255) to velocity (RPS)
    // R = coefficients of quadratic fit of right power to left power. 
    
      double Q1 = -37.61, Q2 = 123.4275, Q3 = -34.2377,  Q4 = 27.9337; // On ground
      double R1 = 0.0049, R2 = 0.4084,   R3 = 13.3867;                 // On ground
  
      // double Q1 =  40.34, Q2 = -93.09, Q3 = 92.96,  Q4 = 2.62;      // Free spinning
      // double R1 = 0.0042, R2 = 0.7560, R3 = 3.8558;                 // Free spinning

    #define slowSpeed 0.2      // Speed of inner wheel for circle turns (RPS)
    #define distAdjust 0.05    // To drive a certain distance, prevents overshoot (in rev)
    #define driveDistSpeed 0.3 // Speed for driving a straight distance (RPS)
    #define wheelDiam 10.0     // Wheel Diameter, cm

    #define adjust 1.05
    #define cycles 1.0
    
    //double DSDelay = 0.00;        // Delay between speed corrections
    
    double rightCount = 0, rightOld = 0, leftCount = 0, leftOld = 0, beltCount = 0, beltOld = 0;  // Motor position variables
    double rightVel = 0, leftVel = 0, beltVel = 0, t = 0, tOldGV = 0, dt = 0;                     // Velocity variables
    double R = 0, Rev = 0, RevOut = 0, origPosL = 0, origPosR = 0, origDistPos = 0;               // Encoder distance trackers
    bool circRight = false, circLeft = false;                                                     // Drive in circles bools
    int leftPower = 0, rightPower = 0, leftAdjust = 0, rightAdjust = 0;                           // DriveAtSpeed and motor power variables
    double tOldDS = 0, j = 0, k = 0, SL = 0;                                                      // SetSpeeds variables                        

  // SERVOS
    Servo railsServoL;
    Servo railsServoR;

    int railsUp = 109;          // Monkey Rails up position, degrees
    int railsBack = 120;        // Monkey Rails back position, degrees
    int railsForward = 0;      // Monkey Rails forward position, degrees

    int monkeyPos = railsForward;   // Default position
    
    double servoDelay = 0.010; // Time increment between servo updates, s

    double oldServoTime = 0;
    int pos0 = 0;

  // BELT
    int beltPower = 0;

  // HALL EFFECT SENSOR
    int hallMid = 0, hall = 0;
    bool hallOn = false;
    double hallScaled = 0;
    double hallTol = 0.2;  // Percent of range for the hall sensor to see to activate it, e.g., 0.2 = 20%

  // WALL FOLLOW

    double Derror = 0, Derror_old = 0, Dvel = 0, t_oldD = 0, WFAdjust = 0;

    #define KpDist  10
    #define KdDist  1
    
    #define WFDist  12    // Wall follow distance, cm
    #define WFSpeed 100   // Wall follow driving speed, rev/s
 
  // OTHER
    String message = "Tell me what to do!";   // Message to computer
    
    String command = "A";        // Command from Uno
    String oldCommand = command; // Previous Command
    String RadCMD  = " ";        // Placeholder for end of string in longer commands
    unsigned char i = 0;         // Generic for loop counter
    bool SerialOn = true;        // Activates serial monitor

///////////////  SETUP  /////////////////

void setup() {

  // INITIATE PINS
    pinMode(M1DPinA, OUTPUT); pinMode(M1DPinB, OUTPUT); pinMode(M1PPin,  OUTPUT);
    pinMode(M2DPinA, OUTPUT); pinMode(M2DPinB, OUTPUT); pinMode(M2PPin,  OUTPUT);
    pinMode(M3DPinA, OUTPUT); pinMode(M3DPinB, OUTPUT); pinMode(M3PPin,  OUTPUT);

    pinMode(REncP1, INPUT); pinMode(REncP2, INPUT);
    pinMode(LEncP1, INPUT); pinMode(LEncP2, INPUT);
    //pinMode(BEncP1, INPUT); pinMode(BEncP2, INPUT);

    pinMode(hallSensorPin,  INPUT); hallMid = analogRead(hallSensorPin);
    
    pinMode(frontSensorPin, INPUT);  pinMode(sideSensorPin, INPUT);
    pinMode(railsServoLPin, OUTPUT); pinMode(railsServoRPin, OUTPUT);

  // INITIATE SERIAL PORTS
    Serial.begin(9600);       // COMPUTER
    Serial.println(message);  // STARTING MESSAGE
    Serial2.begin(9600);      // XBEE
    Serial2.setTimeout(50);   // For reading strings!
    Serial.setTimeout(50);

  // SERVO
    MoveServos(monkeyPos);
  
    railsServoL.attach(railsServoLPin);
    railsServoR.attach(railsServoRPin);
}

bool hasntRun = true;
double tDelay = 0;

////////////////  LOOP  /////////////////

void loop(){

    t = micros()/1000000.0;
    
    GetHall();
    GetLine();

        
     if (hasntRun && hallOn){
        tDelay = t;
        Drive(200, 200);
        hasntRun = false;
     }
    
     if (!hasntRun){

       if (t - tDelay > 3)
          RunBelt(255);
       else
          RunBelt(-255);
       
       if (!(isnan(dist) || isinf(dist)) && t - tDelay > 1.45){
            LineFollow();
            MoveServos(railsUp);
       }
       else {
          if (t - tDelay < 11.5)
             MoveServos(railsUp - 10);
          else {
             MoveRails(railsUp + 10);
             MPow = 80;
          }
          
          if (t - tDelay > 4)
              Drive(200,200);
       } 
     }
} 

