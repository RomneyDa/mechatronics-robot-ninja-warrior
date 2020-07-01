////////////////////////////
//  MECHATRONICS TEAM 8.1 //
//  Competition Code      //
//  4/8/18                //
////////////////////////////

/*
 *  This code uses commands from the xBee to activate various functions
 *  Almost every variable is global, and declared at the top. 
 *  All functions are practically instantaneous, for robustness
 */

#include <QTRSensors.h> 
#include <Encoder.h>
#include <Servo.h>

//////////   COMMANDS   ////////////

  #define CMD_Hall    "W"
  #define CMD_Running "R"

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
    #define frontMax     30         // Maximum front sensor distance (cm)
    #define sideMax      15         // Maximum side sensor distance (cm)

    int Cfront = 2500, Bfront = -1; // Front distance sensor calibration constants (D = CV^B)
    int Cside  = 1000, Bside  = -1; // Side  distance sensor calibration constants (D = CV^B)
    
    int    Vfront, Vside;           // Global distance variables
    double Dfront, Dside;

  // ENCODER
    Encoder RightEnc(REncP1, REncP2);   // RIGHT MOTOR
    Encoder LeftEnc (LEncP1, LEncP2);   // LEFT MOTOR
  
    #define gearRatio    70.0         // Motor gear ratio
    #define countsPerRev 64.0         // Motor counts per revolution of ungeared motor

    double distBetweenWheels = 20.6;  // Distance between front wheels, cm
    double velThreshold = 0.01;       // Velocity correction accuracy, RPS
    double VMax = 2.1;                // Max wheel speed, RPS

    // Measured relationships for relationships between power and velocity
    // Q = coefficients of cubic fit of left motor power (0-255) to velocity (RPS)
    // R = coefficients of quadratic fit of right power to left power. 
    
    double Q1 = -37.61, Q2 = 123.4275, Q3 = -34.2377,  Q4 = 27.9337; // On ground
    double R1 = 0.0049, R2 = 0.4084,   R3 = 13.3867;                 // On ground

    #define distAdjust      0.05 // To drive a certain distance, prevents overshoot (in rev)
    #define driveDistSpeed  0.15 // Speed for driving a straight distance (RPS)
    #define wheelDiam       10.0 // Wheel Diameter, cm
    
    //double DSDelay = 0.00;     // Delay between speed corrections

    bool startedDriving = 0;     // For driving a certain distance
    double rightCount = 0, rightOld = 0, leftCount = 0, leftOld = 0, beltCount = 0, beltOld = 0;  // Motor position variables
    double rightVel = 0, leftVel = 0, beltVel = 0, t = 0, tOldGV = 0, dt = 0;                     // Velocity variables
    double R = 0, Rev = 0, RevOut = 0, origPosL = 0, origPosR = 0, origDistPos = 0;               // Encoder distance trackers
    int leftPower = 0, rightPower = 0, leftAdjust = 0, rightAdjust = 0;                           // DriveAtSpeed and motor power variables
    double tOldDS = 0, j = 0, k = 0, SL = 0;                                                      // SetSpeeds variables                        

  // SERVOS
  
    Servo railsServoL;
    Servo railsServoR;

    int railsUp = 114;          // Monkey Rails up position, degrees
    int railsBack = 149;        // Monkey Rails back position, degrees
    int railsForward = 16;      // Monkey Rails forward position, degrees
    int railsMR = 137;  //138

    int monkeyPos = railsUp;   // Default position
    
    double servoDelay = 0.010; // Time increment between servo updates, s

    double oldServoTime = 0;
    int pos0 = 0;

  // BELT
    int beltPower = 0;

 // HALL SENSOR
  
    double hallTol = 0.15;  // Percent of range for the hall sensor to see to activate it, e.g., 0.2 = 20%

    int hallMid = 0, hall = 0;
    bool hallOn = false;
    double hallScaled = 0;

 // WALL FOLLOW

    double tDelay = 0;
    double Derror = 0, Derror_old = 0, Dvel = 0, t_oldD = 0, WFAdjust = 0;
    bool followedWall = false;

    #define Dthresh 0.1

    #define KpDist  10
    #define KdDist  1.2
    
    #define WFDist  11.8 // Wall follow distance, cm
    #define WFSpeed 70   // Wall follow driving speed

 // LINE FOLLOW
 
    #define NUM_SENSORS 8     // Number of sensors used on reflectance array
    #define TIMEOUT     2500  // Waits for 2500 micro seconds
    #define EMITTER_PIN 2     // Emitter is controlled by digital 2 pin
    
    QTRSensorsRC qtrrc((unsigned char[]) { 3, 4, 5, 6, 7, 8, 9, 10 }, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
  
    int MPow = 60;            // Motor power out of 255 for Line Follow
    #define WHITE 1000.0       // Sensor reading for white. Function of distance from surface
    
    double thresholdDist = 0.7;     // Distance from center of reflectance array to initiate turning, cm
    #define distBetweenSensors 0.9428  // Fixed distance between sensors on the array (measured)
    
    unsigned int sensorValues[NUM_SENSORS];
    double denom = 0, num = 0, SUM = 0, pos = 0, lowSum = 0, lowAverage = 0, lowCounter = 0, sensorCorrected = 0, dist = 0; 
    bool noLine = true;
    double LFerror = 0, LFerror_old = 0, LFvel = 0, LFAdjust = 0, KpLF = 0, KdLF = 0, t_oldLF = 0;
    
    #define KpLF  55
    #define KdLF  5
    

  // TIME KEEPERS

    double tMDelay = 0;
    double tLDelay = 0;
    double tWW = 0;
    double tWF = 0;

  // MONKEY RAILS

    bool pastLine = false;
    bool driveHard = false;
    
  // OTHER

    int stage = 1;
    String command = "W"; // Waiting

    char cmdStr = 'W';
    int cmdNum = 0;
    
    String oldCommand = command; // Previous Command
    String RadCMD  = " ";        // Placeholder for end of string in longer commands
    unsigned char i = 0;         // Generic for loop counter    

void setup() {
  
  // INITIATE PINS
    pinMode(M1DPinA, OUTPUT); pinMode(M1DPinB, OUTPUT); pinMode(M1PPin,  OUTPUT);
    pinMode(M2DPinA, OUTPUT); pinMode(M2DPinB, OUTPUT); pinMode(M2PPin,  OUTPUT);
    pinMode(M3DPinA, OUTPUT); pinMode(M3DPinB, OUTPUT); pinMode(M3PPin,  OUTPUT);

    pinMode(REncP1, INPUT); pinMode(REncP2, INPUT);
    pinMode(LEncP1, INPUT); pinMode(LEncP2, INPUT);

    pinMode(hallSensorPin,  INPUT); hallMid = analogRead(hallSensorPin);
    
    pinMode(frontSensorPin, INPUT);  pinMode(sideSensorPin, INPUT);
    pinMode(railsServoLPin, OUTPUT); pinMode(railsServoRPin, OUTPUT);

  // INITIATE SERIAL PORTS
    Serial2.begin(9600);      // XBEE
    Serial2.setTimeout(50);   // For reading strings!

  // SERVO
    MoveServos(monkeyPos);
  
    railsServoL.attach(railsServoLPin);
    railsServoR.attach(railsServoRPin);

}

void loop() {

  t = micros()/1000000.0;
  
  GetWireless(); // CONSTANTLY CHECK FOR WIRELESS SIGNAL!
  GetLine();     // USED IN ALMOST EVERY TRANSITION!
  GetVelocity(); // Update motor velocities. See Encoder tab
  GetDistance(); // Read distance sensor(s)
    
  if(command == "1" || command == "2" || command == "3" || command == "4" || command == "5"){
     stage = command.toInt();
     command = CMD_Hall;
  }
  
  if(command == CMD_Hall)
     WaitForHall();


  else {

    if      (stage == 1)  // Wall Follow / Paddleboard
      WFState();

    else if (stage == 2)  // Wall Lift
      WLState();

    else if (stage == 3)  // U-turn
      UTState();
      
    else if (stage == 4)  // Monkey Rails
      MRState();

    else if (stage == 5)  // Warped Wall
      WWState();

    else
      Stop();

  }
   Serial2.println(stage);
}
