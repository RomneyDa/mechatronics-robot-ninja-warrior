#include <SoftwareSerial.h>
#include <DualVNH5019MotorShield.h>
#include <Servo.h>

SoftwareSerial mySerial(0, 1); // RX, TX

//////////////////////////
// INPUT GUIDE:         //
//   O: Output on       //
//   o: Output off      //
//   S: Wall lift Servo //
//   s: Monkey Servo    //
//   M: Motor           //
//////////////////////////

// Pins, servo/motor objects
int railsServoPin = 44;
int wallServoPin = 45;
Servo wallServo;
Servo railsServo;
DualVNH5019MotorShield motors;

void setup()  
{
  
  pinMode(wallServoPin, OUTPUT);
  pinMode(railsServoPin, OUTPUT);
  
  railsServo.attach(railsServoPin);
  railsServo.write(0);
  
  wallServo.attach(wallServoPin);
  wallServo.write(0);

  motors.init();
}

void loop()
{

  wallServo.write(90);
  delay(1000);
  wallServo.write(180);
  delay(1000);
  wallServo.write(90);
  delay(1000);
  wallServo.write(0);
  delay(1000);

  railsServo.write(90);
  delay(1000);
  railsServo.write(180);
  delay(1000);
  railsServo.write(90);
  delay(1000);
  railsServo.write(0);
  delay(1000);

  motors.setM1Speed(200);
  motors.setM2Speed(200);
  delay(2000);
  motors.setM1Speed(-200);
  motors.setM2Speed(-200);
  delay(2000);

  motors.setM1Speed(200);
  motors.setM2Speed(-200);
  delay(2000);
  motors.setM1Speed(-200);
  motors.setM2Speed(200);
  delay(2000);

  motors.setM1Speed(0);
  motors.setM2Speed(0);

}
