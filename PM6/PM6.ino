//////////////////////////////////////////////////////////////////////////////////
// MECHATRONICS ROBOT NINJA WARRIOR II COMPETITION CODE                         //
// TEAM 8.1: Layne Christensen, Dallin Romney, Tyler Barrett and Anthony Amiker //
// SPRING 2018                                                                  //
//////////////////////////////////////////////////////////////////////////////////

/////////////   GUIDE   //////////////
// var =                            //
// A: Stop                          //
// W: Move wall servo               //
// U: Wall servo down               //
// u: Wall servo up                 //
// S: Move monkey rails assembly    //
// Q: Move monkey rails assembly up //
// q: Move mr assembly down         //
// B: Move belt                     //
// C: Move belt Other Direction     //
// D: Drive forward                 //
// d: Drive backward                //
// F: Drive forward, then backward  //
// T: Turn, then turn back          //
// L: Turn left                     //
// R: Turn right                    //
// Anything Else: Stop              //
//////////////////////////////////////

#include <Servo.h>          // FOR SERVOS
#include <SoftwareSerial.h> // FOR XBEES

// SET PINS
  int wallServoPin = 6;
  int M1PPin = 7;
  int M2PPin = 8;
  int M3PPin = 9;

  int xBeeRXPin = 10;
  int xBeeTXPin = 11;
  
  int M1DPinA = 28;
  int M1DPinB = 29;                      
  int M2DPinA = 30;
  int M2DPinB = 31;
  int M3DPinA = 32;
  int M3DPinB = 33;
  
  int railsServo1Pin = 44;
  int railsServo2Pin = 46;

// CREATE OBJECTS
  Servo railsServo1;
  Servo railsServo2;
  Servo wallServo;

  SoftwareSerial mySerial(xBeeRXPin, xBeeTXPin); // RX, TX

// OTHER INITIAL VARIABLES
char var = 'X';
int wallUp = 0;
int wallDown = 105;
int railsUp = 20;    // Rotate towards front limit
int railsDown = 110; // Rotate towards back limit
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

// FUNCTION TO RUN BELT. JUST SAVES TIME LATER
  void RunBelt(int power) {
    RunMotor(M1DPinA, M1DPinB, M1PPin, power);
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

// FUNCTION FOR SIMULTANEOUSLY MOVING THE MONKEY RAILS SERVOS
void MoveRails(int pos) {

  int pos0 = railsServo1.read();             // Get the current servo position
                                             //
  for(int i = 0; i <= abs(pos - pos0); i++){ // For each step between current and desired position,
      if (pos > pos0) {                      // If the desired position is greater
        railsServo1.write(pos0 + i);         // Move that direction
        railsServo2.write(180 - (pos0 + i)); // (Other servo is facing the opposite direction)
      }                                      //
      else {                                 // Otherwise,
        railsServo1.write(pos0 - i);         // Move the other way
        railsServo2.write(180 - (pos0 - i)); // (Again, other servo is facing the other way)
      }                                      //
      delay(25);                             // Small time increment makes the movements appear smooth
  }
}

// SETUP
  void setup()
  {
    //INITIATE PINS
      pinMode(railsServo1Pin, OUTPUT);
      pinMode(railsServo2Pin, OUTPUT);
      pinMode(wallServoPin,   OUTPUT);
    
      pinMode(M1DPinA, OUTPUT);
      pinMode(M1DPinB, OUTPUT);
      pinMode(M1PPin,  OUTPUT);
      pinMode(M2DPinA, OUTPUT);
      pinMode(M2DPinB, OUTPUT);
      pinMode(M2PPin,  OUTPUT);
      pinMode(M3DPinA, OUTPUT);
      pinMode(M3DPinB, OUTPUT);
      pinMode(M3PPin,  OUTPUT);

    // ATTACH SERVOS AND INITIATE OBJECTS

      railsServo1.write(railsServo1.read());
      railsServo2.write(railsServo2.read());
      wallServo.write(wallUp);
      
      railsServo1.attach(railsServo1Pin);
      railsServo2.attach(railsServo2Pin);
      wallServo.attach(wallServoPin);

      mySerial.begin(9600); // Open serial port with 
      //Serial.begin(9600);
      //Serial.println("starting...");
   
  }

void loop()
{

// READ SERIAL (XBEE)
  if (mySerial.available()){
    var = char(mySerial.read()); // Set the state variable (will be last character)
    //Serial.print(var);
    delay(3);
    if(!mySerial.available()){   // Print all the other characters
    //Serial.println();
    }
  }

// BELT
  if (var == 'B') {
    RunBelt(MPow);
  }
  else if (var == 'C') {
    RunBelt(-MPow);
  }
  else {
    RunBelt(0);
  }

// WALL LIFT ASSEMBLY
  if (var == 'W') {
    wallServo.write(wallDown);
    delay(1000);
    wallServo.write(wallUp);
    delay(1000);
  }
  if (var == 'U') {
    wallServo.write(wallUp);
  }
  if (var == 'u') {
    wallServo.write(wallDown);
  }

// MONKEY RAILS ASSEMBLY
  if (var == 'S') {
    MoveRails(railsUp);
    delay(1000);
    MoveRails(railsDown);
    delay(1000);
  }
  if (var == 'Q') {
    MoveRails(railsUp);
    delay(1000);
  }
  if (var == 'q') {
    MoveRails(railsDown);
    delay(1000);
  }
  if (var == 'Y'){
    
  }
// DRIVE
  if (var == 'F') {
    DriveForTime(MPow, MPow, 1000);
    delay(1000);
    DriveForTime(-MPow, -MPow, 1000);
    delay(1000);
  }
  if (var == 'D'){
    DriveForTime(MPow, MPow, 1000);
    delay(1000);
  }
  if (var == 'd'){
    DriveForTime(-MPow, -MPow, 1000);
    delay(1000);
  }
// TURN
  if (var == 'T') {
    DriveForTime(-MPow, MPow, 1000);
    delay(1000);
    DriveForTime(MPow, -MPow, 1000);
    delay(1000);
  }
  if (var == 'L') {
    DriveForTime(-MPow, MPow, 1000);
    delay(1000);
  }
  if (var == 'R') {
    DriveForTime(MPow, -MPow, 1000);
    delay(1000);
  }

/* LED TEST
  if (var == 'L') {
    for (int i = 0; i < 50; i++){ // BRIGHTEN IT
      analogWrite(13, i);
      delay(25);
    }
    for (int i = 50; i > 0; i--){ // DIM IT
      analogWrite(13, i);
      delay(25);
    }
  }
*/

/* HARD CODED TEST
  MoveRails(120);
  delay(2000);
  MoveRails(50);
  delay(2000);

  wallServo.write(90);
  delay(1000);
  wallServo.write(40);
  delay(1000);

  RunBelt(255);
  delay(1000);
  RunBelt(0);
  delay(1000);
  RunBelt(-255);
  delay(1000);
  RunBelt(0);

  Drive(100, 100);
  delay(2000);
  Drive(0, 0);
  delay(1000);
  Drive(-100, 100);
  delay(2000);
  Drive(0,0);
  delay(1000);
*/
}

///////////////////      EXTRAS       //////////////////////////

/* LED TEST

*/

/* SLOW WALL LIFT FUNCTION
void LiftWall(int pos) {

  int pos0 = wallServo.read();
  
  for(int i = 0; i <= abs(pos - pos0); i++){
      if (pos > pos0) {
        wallServo.write(pos0 + i);
      }
      else {
        wallServo.write(pos0 - i);
      }
      delay(25);
  }
*/

//}
