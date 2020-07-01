#include <Servo.h>    
int sensorPin = A3;
int wallServoPin = 6;
Servo wallServo;
  
int wallUp = 0;
int wallDown = 105;
int mid = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(wallServoPin,   OUTPUT);
  
  wallServo.write(wallUp);
  wallServo.attach(wallServoPin);
  
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  
  mid = analogRead(sensorPin);
  Serial.print("Initial Reading: ");
  Serial.println(mid);
}

void loop() {
  // put your main code here, to run repeatedly:

  int x = analogRead(sensorPin);
  Serial.println(analogRead(sensorPin));

  delay(100);
  
  if (x > mid + mid/10){
   wallServo.write(wallDown);
    delay(1000);
    wallServo.write(wallUp);
    delay(1000);
  }
  if (x < mid - mid/10){
    wallServo.write(wallDown);
    delay(1000);
    wallServo.write(wallUp);
    delay(1000);
}
    
}

