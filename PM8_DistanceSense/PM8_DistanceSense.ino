//  MECHATRONICS TEAM 8.1
//  PM8 Code
//  3/4/18

#include <QTRSensors.h>

// Define Pins
  #define M2PPin 11
  #define M3PPin 12
                     
  #define M2DPinA 30
  #define M2DPinB 31
  #define M3DPinA 32
  #define M3DPinB 33

  #define frontSensorPin A14
  #define sideSensorPin A15

// Important variables
  int stopDistance = 10; // Will stop with a wall this far in front (cm)
  int wallDistMax = 10;  // Won't try to wall follow if no wall with this distance (cm)
  int wallDist = 6;      // Preferred distance for wall fallow (cm)
  int wallTol = 1.5;     // Distance tolerance for wall follow (cm)

  int MPow = 100;
  int driveTime = 25;   // ms
  int followTime = 150; // ms
  int delayTime = 50;   // ms
  double WHITE = 600;   // Sensor reading for white. Function of distance from surface
  
  double thresholdDist = 1.3;          // Distance from center of reflectance array to initiate turning, cm
  double distBetweenSensors = 0.9428;  // Fixed distance between sensors on the array (measured)

// Distance sensor calibration equation constants (experimentally derived)
  int Cfront = 2500, Bfront = -1; // Front distance sensor
  int Cside  = 1000, Bside  = -1; // Side distance sensor

// Reflectance sensors 0 through 7 are connected to pins 3 through 10
  #define NUM_SENSORS 8     //number of sensors used
  #define TIMEOUT     2500  //waits for 2500 micro seconds
  #define EMITTER_PIN 2     //emitter is controlled by digital 2 pin
  
  QTRSensorsRC qtrrc((unsigned char[]) { 3, 4, 5, 6, 7, 8, 9, 10 }, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

// Various initial variables
  unsigned int sensorValues[NUM_SENSORS];
  double denom = 0, num = 0, pos = 0;
  double lowSum = 0, lowAverage = 0, lowCounter = 0;
  double sensorCorrected = 0, dist = 0; 

  char var = 'A';

  int    Vfront, Vside;
  double Dfront, Dside;

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

  // It's useful to have turn functions so we don't have to keep track of which motor is which
  void TurnRight(int power, int Time){
    DriveForTime(power, -power, Time);
  }
  void TurnLeft(int power, int Time){
    DriveForTime(-power, power, Time);
  }

  void LineFollow(){
      if (dist > thresholdDist) {            // If the robot is to the right of the line
        Serial.print("(LF) Turn Left");      // Turn left
        TurnLeft(MPow, driveTime);
      }
      else if (dist < -thresholdDist){       // If the robot is to the left of the line
        Serial.print("(LF) Turn Right");     // Turn right
        TurnRight(MPow, driveTime);
      }
      else {                                 // Otherwise
        Serial.print("(LF) Drive Straight"); // Go straight
        DriveForTime(MPow, MPow, driveTime);
      }
  }
/*
  void WallFollow(){
    if(Dside > wallDist + wallTol) {      // WALL FOLLOW MODE
          Serial.print("(WF) Adjust Right");  // Turning the front of the robot right will move the distance sensor closer to the wall
          TurnRight(MPow, driveTime);
          delay(delayTime);
          DriveForTime(MPow, MPow, followTime);
          delay(delayTime);
          TurnLeft(MPow, driveTime);
    }
    else if(Dside < wallDist - wallTol){
          Serial.print("(WF) Adjust Left");   // and vice versa
          TurnLeft(MPow, driveTime);
          delay(delayTime);
          DriveForTime(MPow, MPow, followTime);
          delay(delayTime);
          TurnRight(MPow, driveTime);
    }
    else {
          Serial.print("(WF) Drive Straight"); // Go straight
          DriveForTime(MPow, MPow, followTime);[
*/

void setup() {

  // Initiate Pins
    pinMode(M2DPinA, OUTPUT);
    pinMode(M2DPinB, OUTPUT);
    pinMode(M2PPin,  OUTPUT);
    pinMode(M3DPinA, OUTPUT);
    pinMode(M3DPinB, OUTPUT);
    pinMode(M3PPin,  OUTPUT);

    pinMode(frontSensorPin, INPUT);
    pinMode(sideSensorPin,  INPUT);

  // Initialize Serial ports
    Serial.begin(9600); Serial.println("Hello Computer!");
    Serial2.begin(9600); // xBee on pins 17 (RX) and 16 (TX)
}

void loop() 
{
  
  if (Serial2.available()) { // If data comes in from XBee, send it out to serial monitor
    var = char(Serial2.read());
  }
  if (Serial.available()){
    Serial2.write(Serial.read());
  }
  // Read distance sensors and convert voltage to distance (cm)
    Vfront = analogRead(frontSensorPin);
    Dfront = Cfront*pow(Vfront, Bfront);
  
    Vside = analogRead(sideSensorPin);
    Dside = Cside*pow(Vside, Bside);
  
  // Read raw reflectance sensor values. Range is 0-2500, 0 = white, 2500 = black
    qtrrc.read(sensorValues);
  
  num = denom = lowSum = lowCounter = 0;
for (unsigned char i = 0; i < NUM_SENSORS; i++){
    if (sensorValues[i] < WHITE){                   // If it's reading white
      lowSum += sensorValues[i];                    // Sum all the white values
      lowCounter ++;                                // Count it
    }
  }

  lowAverage = lowSum/lowCounter;                   // Average the white values
  
  for (unsigned char i = 0; i < NUM_SENSORS; i++)   // For each sensor
  {
    sensorCorrected = sensorValues[i] - lowAverage; // Remove the sensor bias
    
    num += i*sensorCorrected;                       // Weighted sum of all unbiased sensor values
    denom += sensorCorrected;                       // Sum of all unbiased sensor values
    
    Serial.print(sensorValues[i]);                  // Print results
    Serial.print(' ');
  }

  pos = num/denom;    // Read: Numerator over denominator -> centroid of the values
  Serial.print(pos);
  Serial.print('\t');

  // Convert position on sensor array to distance from center of array
  dist = (pos - double(double(NUM_SENSORS) - 1)/2)*distBetweenSensors;
  
  Serial.print(dist);
  Serial.print('\t');

if (var != 'S'){
  if (Dfront < stopDistance) {              // If it sees a wall in front
    Serial.print("Stopped (Wall!)");        // Stop
    ;
    Drive(0, 0);
  }
  else if (isnan(dist) || dist > 100) {     // If there's no line
     
     // if(Dside < wallDistMax) WallFollow();

     Serial.print("Stopped (No Line!)");    // Stop
     Drive(100, 100);
  }
  else {                                    // LINE FOLLOW MODE
     LineFollow();
  }
}
else {
  Serial.print("Stopped (Wireless Command)");        // Stop
  Drive(0, 0);
}

  delay(delayTime);
  Serial.print('\t');

// Print distance sensor readings:
  Serial.print("Front: ");
  Serial.print(Vfront);
  Serial.print(" -> D = ");
  Serial.print(Dfront);

  Serial.print("   Side: ");
  Serial.print(Vside);
  Serial.print(" -> D = ");
  Serial.print(Dside);

  Serial.print(' ');
  Serial.println(var);
} 

