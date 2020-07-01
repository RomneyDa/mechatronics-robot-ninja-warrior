#include <SoftwareSerial.h>
//#include <DualVNH5019MotorShieldMod3.h>

SoftwareSerial mySerial(0, 1); // RX, TX

//////////////////////////
// INPUT GUIDE:         //
//   O: Output on       //
//   o: Output off      //
//   S: Wall lift Servo //
//   s: Monkey Servo    //
//   M: Motor           //
//////////////////////////

// Motor Variables
int motorDelay = 200;    // Time between motor power updates (ms)
int motorIncrement = 10; // Motor power increment. Must be a factor of 255 (1, 3, 5, 17, 51, or 255)

// Servo Variables
int wallServoDelay = 90;    // Time between servo movements (ms)
int wallServoIncrement = 5;  // How far to move servo, degrees. Must be a factor of 180!

int railsServoDelay = 90;   // Same for rails Servo
int railsServoIncrement = 5;

// Pins, servo/motor objects
int railsServoPin = 5;
int wallServoPin = 13;
//DualVNH5019MotorShieldMod3 motors;

// Initial variables
int motorPower = 0;     // Current motor power
int wallServoPos = 0;   // Current wall lift servo position in degrees
int wallServoUp = 1;    // Directional servo variable for wall lift
int railsServoPos = 0;  // Current rail assembly servo position in degrees
int railsServoUp = 1;   // Directional servo variable for rails
int motorCW = 1;        // Directional motor variable
int timeCurrent = 0;    // Time place-holder
int output = 1;         // Output toggle variable
char var = 'A';         // Initial input variable

void setup()  
{
  Serial.begin(9600);   // Open serial commuications
  mySerial.begin(9600); // Open serial port with 

  Serial.println("Hello Computer!");      // Send message to computer
  mySerial.print("Hello other Arduino!"); // Send message to Uno

}

void loop()
{

// Read Serial Buffer

  if (mySerial.available()){
    var = char(mySerial.read()); // Set the state variable (will be last character)
    Serial.print(var);
    delay(3);
    if(!mySerial.available()){   // Print all the other characters
      Serial.println();
    }
  }

// WALL LIFT SERVO

  if (var == 'S') {
    
    if (wallServoPos <= 0)   wallServoUp = 1; // Switch directions at 0 degrees
    if (wallServoPos >= 180) wallServoUp = 0; // Switch directions at 180 degrees
    
    if (wallServoUp) wallServoPos += wallServoIncrement; // Increase servo position
    else             wallServoPos -= wallServoIncrement; // Decrease servo position
    
    delay(wallServoDelay); // Wait
  }

// MONKEY RAILS ASSEMBLY SERVO

  if (var == 's') {
    
    if (railsServoPos <= 0)   railsServoUp = 1; // Switch directions at 0 degrees
    if (railsServoPos >= 80)  railsServoUp = 0; // Switch directions at 80 degrees
    
    if (railsServoUp) railsServoPos += railsServoIncrement; // Increase servo position
    else              railsServoPos -= railsServoIncrement; // Decrease servo position
    
    delay(railsServoDelay); // Wait
  }

// MOTOR

  if (var == 'M') {
    
      if (motorPower <= -400) motorCW = 1; // Switch directions at min power
      if (motorPower >= 400)  motorCW = 0; // Switch directions at max power
      
      if (motorCW) motorPower += motorIncrement; // Increase the motor power
      else         motorPower -= motorIncrement; // Decrease the motor power
  
      delay(motorDelay); // Wait
  }
  else motorPower = 0; // Turn the motor off

// OUTPUT

  if (var == 'O') output = 1; // Toggle output on
  if (var == 'o') {
    output = 0;               // Toggle output off
    Serial.println();         // Fill screen with blank lines
  }

  if(output) {
    Serial.print("rServo: ");
    Serial.print(railsServoPos);

    Serial.print("  wServo: ");
    Serial.print(wallServoPos);
    
    Serial.print("  mPower: ");
    Serial.println(motorPower);
  }      

}
