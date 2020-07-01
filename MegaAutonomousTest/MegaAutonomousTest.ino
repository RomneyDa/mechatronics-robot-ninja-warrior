#include "DualVNH5019MotorShield.h"
#include "Servo.h"

int railsServoPin = 44;
int wallServoPin = 45;

Servo wallServo;
Servo railsServo;
DualVNH5019MotorShield md;

void setup()
{
  pinMode(wallServoPin, OUTPUT);
  pinMode(railsServoPin, OUTPUT);

  railsServo.attach(railsServoPin);
  wallServo.attach(wallServoPin);

  railsServo.write(0);
  wallServo.write(0);
  
  Serial.begin(9600);
  Serial.println("Autonomous Test:");
  md.init();
  delay(5000);
}

void loop()
{

  Serial.println("Moving forward");
  md.setSpeeds(200, 200);
  delay(2000);

  Serial.println("Moving backwards");
  md.setSpeeds(-200, -200);
  delay(2000);

  Serial.println("Turning left");
  md.setSpeeds(200, -200);
  delay(2000);

  Serial.println("Turning right");
  md.setSpeeds(-200, 200);
  delay(2000);

  md.setSpeeds(0, 0);
  
  Serial.println("Moving wall lift servo");
  wallServo.write(90);
  delay(1000);
  wallServo.write(180);
  delay(1000);
  wallServo.write(0);
  delay(1000);

  Serial.println("Moving monkey rails servo");
  railsServo.write(90);
  delay(1000);
  railsServo.write(180);
  delay(1000);
  railsServo.write(0);

  Serial.println("Waiting 5 seconds");
  delay(5000);
}
