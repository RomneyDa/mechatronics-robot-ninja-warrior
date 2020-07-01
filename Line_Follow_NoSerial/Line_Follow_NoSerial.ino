#include <QTRSensors.h>

#define NUM_SENSORS 8     //number of sensors used
#define TIMEOUT     2500  //waits for 2500 micro seconds
#define EMITTER_PIN 2     //emitter is controlled by digital 2 pin

// Reflectance sensors 0 through 7 are connected to pins 3 through 10
  QTRSensorsRC qtrrc((unsigned char[]) { 3, 4, 5, 6, 7, 8, 9, 10 }, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

// Important variables
  int MPow = 100;
  int driveTime = 25; // ms
  int delayTime = 75; // ms
  double WHITE = 600; // Sensor reading for white. Function of distance from surface
  
  double thresholdDist = 1.3;          // Distance from center of reflectance array to initiate turning, cm
  double distBetweenSensors = 0.9428;  // Fixed distance between sensors on the array (measured)

// Define Pins
  #define M2PPin 11
  #define M3PPin 12
                     
  #define M2DPinA 30
  #define M2DPinB 31
  #define M3DPinA 32
  #define M3DPinB 33

// Various initial variables
  unsigned int sensorValues[NUM_SENSORS];
  double denom = 0, num = 0, pos = 0;
  double lowSum = 0, lowAverage = 0, lowCounter = 0;
  double sensorCorrected = 0, dist = 0; 

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

void setup() {

  // Initiate Pins
    pinMode(M2DPinA, OUTPUT);
    pinMode(M2DPinB, OUTPUT);
    pinMode(M2PPin,  OUTPUT);
    pinMode(M3DPinA, OUTPUT);
    pinMode(M3DPinB, OUTPUT);
    pinMode(M3PPin,  OUTPUT);
}

void loop() 
{
  // Read raw zreflectance sensor values. Range is 0-2500, 0 = white, 2500 = black
  qtrrc.read(sensorValues);
  
  num = denom = lowSum = lowCounter = 0;

  for (unsigned char i = 0; i < NUM_SENSORS; i++){  // For each sensor
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
  }

  // Read: Numerator over denominator -> centroid of the values
  pos = num/denom;    

  // Convert position on sensor array to distance from center of array
  dist = (pos - double(double(NUM_SENSORS) - 1)/2)*distBetweenSensors;

  if (dist > thresholdDist && dist < 100)         TurnLeft(MPow, driveTime);           // If to the right, turn left
  else if (dist < -thresholdDist && dist > -100)  TurnRight(MPow, driveTime);          // If to the left, turn right
  else                                            DriveForTime(MPow, MPow, driveTime); // Otherwise, go straight

  delay(delayTime);
  
} 

