////////////////////////////
//  MECHATRONICS TEAM 8.1 //
//  PM9 Code              //
//  3/9/18                //
////////////////////////////

/*
 *  This code uses commands from the xBee to activate various functions
 *  Almost every variable is global, and declared at the top. 
 *  All functions are practically instantaneous, for robustness
 */


#include <QTRSensors.h> 
#include <Encoder.h>

///////////   COMMANDS   ////////////

  #define CMD_Dist       "D" // See Distance Sensors
  #define CMD_Encoder    "E" // Check Encoders
  #define CMD_LineFollow "L" // Line Follow
  #define CMD_SerialOff  "p" // Stop printing - will be much faster
  #define CMD_SerialOn   "P" // Start printing - will delay significantly 
  #define CMD_RadiusL    'R' // Drive a circle to the left,  e.g., R100 = 100 cm
  #define CMD_RadiusR    'r' // Drive a circle to the right, e.g., r90  = 90 cm
  #define CMD_DriveDist  'y' // Drive a distance, e.g., y100 = drive 100 cm
  #define CMD_DriveSpeed 'z' // Drive a speed, in tenths of an RPS e.g, z11 = 1.1 RPS

  // Placeholders - Don't send these commands, it will set them automatically
  #define CMD_Circle     "C" // Doing Circles
  #define CMD_DrivingD   "Y" // Driving for a distance
  #define CMD_DrivingS   "Z" // Driving at a speed
  
/////////////   PINS   //////////////
 
  #define M2PPin 11             
  #define M3PPin 12
  
  #define REncP1 18  
  #define REncP2 19
  #define LEncP1 20
  #define LEncP2 21
//#define BEncP1 14  
//#define BEncP2 15
                     
  #define M2DPinA 30
  #define M2DPinB 31
  #define M3DPinA 32
  #define M3DPinB 33

  #define frontSensorPin A14
  #define sideSensorPin A15

//////////   VARIABLES   ////////////

  // DISTANCE
    #define stopDistance 10         // Stop with a wall this far in front (cm)
    #define frontMax     30         // Maximum front sensor distance (cm)
    #define sideMax      20         // Maximum side sensor distance (cm)

    int Cfront = 2500, Bfront = -1; // Front distance sensor calibration constants (D = CV^B)
    int Cside  = 1000, Bside  = -1; // Side  distance sensor calibration constants (D = CV^B)
    
    int    Vfront, Vside;           // Global distance variables
    double Dfront, Dside;

  // LINE FOLLOW
    #define NUM_SENSORS 8     // Number of sensors used on reflectance array
    #define TIMEOUT     2500  // Waits for 2500 micro seconds
    #define EMITTER_PIN 2     // Emitter is controlled by digital 2 pin
    
    QTRSensorsRC qtrrc((unsigned char[]) { 3, 4, 5, 6, 7, 8, 9, 10 }, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
  
    #define MPow 30           // Motor power out of 255 for Line Follow
    #define WHITE 600.0       // Sensor reading for white. Function of distance from surface
    
    #define thresholdDist      1.3     // Distance from center of reflectance array to initiate turning, cm
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
    double velThreshold = 0.01;       // Velocity correction accuracy, RPS
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
    
    double DSDelay = 0.00;        // Delay between speed corrections
    
    double rightCount = 0, rightOld = 0, leftCount = 0, leftOld = 0, beltCount = 0, beltOld = 0;  // Motor position variables
    double rightVel = 0, leftVel = 0, beltVel = 0, t = 0, tOldGV = 0, dt = 0;                     // Velocity variables
    double R = 0, Rev = 0, RevOut = 0, origPosL = 0, origPosR = 0, origDistPos = 0;               // Encoder distance trackers
    bool circRight = false, circLeft = false;                                                     // Drive in circles bools
    int leftPower = 0, rightPower = 0, leftAdjust = 0, rightAdjust = 0;                           // DriveAtSpeed and motor power variables
    double tOldDS = 0, j = 0, k = 0, SL = 0;                                                             // SetSpeeds variables                        

  // OTHER
    String message = "Hello!";   // Message to computer
    String command = "A";        // Command from Uno
    String oldCommand = command; // Previous Command
    String RadCMD  = " ";        // Placeholder for end of string in longer commands
    unsigned char i = 0;         // Generic for loop counter
    bool SerialOn = true;        // Activates serial monitor

///////////////  SETUP  /////////////////

void setup() {

  // INITIATE PINS
    //pinMode(M1DPinA, OUTPUT); pinMode(M1DPinB, OUTPUT); pinMode(M1PPin,  OUTPUT);
    pinMode(M2DPinA, OUTPUT); pinMode(M2DPinB, OUTPUT); pinMode(M2PPin,  OUTPUT);
    pinMode(M3DPinA, OUTPUT); pinMode(M3DPinB, OUTPUT); pinMode(M3PPin,  OUTPUT);

    pinMode(REncP1, INPUT); pinMode(REncP2, INPUT);
    pinMode(LEncP1, INPUT); pinMode(LEncP2, INPUT);
    //pinMode(BEncP1, INPUT); pinMode(BEncP2, INPUT);

    pinMode(frontSensorPin, INPUT); pinMode(sideSensorPin, INPUT);

  // INITIATE SERIAL PORTS
    Serial.begin(9600);       // COMPUTER
    Serial.println(message);  // STARTING MESSAGE
    Serial2.begin(9600);      // XBEE
    Serial2.setTimeout(50);   // For reading strings!
}

////////////////  LOOP  /////////////////

void loop(){
  
    t = micros()/1000000.0;  // Get current time
     
    GetEncoders();           // Update encoder positions. See Encoder tab
    GetVelocity();           // Update motor velocities. See Encoder tab
    GetWireless();           // Check for commands from xBee. See Wireless tab
  
    if (command == CMD_Dist)         // Update distances - see Distance tab
      GetDistance();        
  
    if (command == CMD_LineFollow)   // Line follow - see LineFollow tab
      LineFollow();
  
    else if (command == CMD_Encoder) // Check encoders - see Encoders tab
      DriveAtSpeed(1.0, 1.0);


 // DRIVE IN A CIRCLE TO THE LEFT OR RIGHT
 
    else if (command.charAt(0) == CMD_RadiusR || command.charAt(0) == CMD_RadiusL){
      
        RadCMD = command.substring(1); // Take everthing but the first character of the string
        R = RadCMD.toInt();            // Convert to number
        
        Rev = adjust*cycles*2*R/wheelDiam;           // Convert to revolutions of the inner wheel
        RevOut = adjust*cycles*2*(R + distBetweenWheels)/wheelDiam;
  
        leftPower = rightPower = 0;    // Reset motor power adjustments
        leftAdjust = 0; rightAdjust = 0;

        origPosL = leftCount;          // Set distance references at wheel positions
        origPosR = rightCount;
  
        if (command.charAt(0) == CMD_RadiusR)
            circRight = true;                    // Activate right driving
        else                             
            circLeft = true;                     // Activate left driving
        
        command = CMD_Circle;  // Update command so that this loop will only run once. Next group will loop
    }

    // This part runs constantly for either direction - see Encoder tab
    else if (command == CMD_Circle)
          Circle();
  
  // DRIVE A CERTAIN SPEED. In this case, R is a speed (RAD/S)
  
    // This part recieves the message and initiates the speed 
    else if (command.charAt(0) == CMD_DriveSpeed){
        message = "Drive @ Speed!";
        RadCMD = command.substring(1);  // Get string other than first character (numerical part)
        R = double(RadCMD.toInt())/10;  // Convert from tenths of a RPS to RPS
  
        if (R > VMax)                   // Limit velocity to within max limits
          R = VMax;
        if (R < -VMax)
          R = -VMax;
  
        leftAdjust = rightAdjust = 0;   // Reset motor adjustments
        command = CMD_DrivingS;         // Update command so that this part will only run once. Next group will loop
    }
  
    // This part does the driving
    else if (command == CMD_DrivingS) { 
      if (R == 0)                       // If the distance is zero, stop
          Stop();
      else
          DriveAtSpeed2(R, R);           // Otherwise, drive at speed determined in initiation above
    }
  
  // DRIVE A CERTAIN DISTANCE. In this case, R is a straight distance
  
    // This part initiates the distance values
    else if (command.charAt(0) == CMD_DriveDist){
        message = "Drive @ Dist!";           
        
        RadCMD = command.substring(1);            // Get everything but the first letter in the string
        R = double(RadCMD.toInt());               // Convert it to a number
        Rev = R/(PI*wheelDiam) - distAdjust;      // Convert that distance to wheel revolutions
  
        origDistPos = leftCount;      // Record current position. Could also use rightCount (it's a straight line)
        leftAdjust = rightAdjust = 0; // Reset motor adjustments
        command = CMD_DrivingD;       // Update command so that it will get out of this loop and drive with given values
    }
  
    // This part drives until the given distance is reached
    else if (command == CMD_DrivingD) {               
      if (R == 0 || leftCount - origDistPos >= Rev){    // If distance is reached, stop
          message = "Distance Reached!";
  
          Stop();
      }
      else {
          DriveAtSpeed2(driveDistSpeed, driveDistSpeed); // Otherwise, keep going
      }
    }
  
  //  SERIAL MONITOR
  
    // This code simply allows the serial monitor to be turned off or on, without effecting anything else
    // Turning it off will speed up loop iteration significantly
  
    else if (command == CMD_SerialOn || command == CMD_SerialOff){
      SerialOn = (command == CMD_SerialOn);
      command = oldCommand;
    }
    
  // STOP
   
    else
      Stop();
    
    if (SerialOn){
      SerialPrint();
      DSDelay = 0.00;
    }
    else
      DSDelay = 0.05;
  
    oldCommand = command; // Update old command

} 

