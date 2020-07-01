

int M1PPin = 7;
int M2PPin = 8;
int M3PPin = 9;

int M1DPinA = 28;
int M1DPinB = 29;                      
int M2DPinA = 30;
int M2DPinB = 31;
int M3DPinA = 32;
int M3DPinB = 33;


void RunMotor(int dPin1, int dPin2, int pPin, int power){
  if (power == 0){
    digitalWrite(dPin1, LOW); 
    digitalWrite(dPin2, LOW);
  }
  else if (power < 0){
    digitalWrite(dPin1, LOW); 
    digitalWrite(dPin2, HIGH);
    power = -power;
  }
  else {
    digitalWrite(dPin1, HIGH); 
    digitalWrite(dPin2, LOW);
  }
  analogWrite(pPin, power);
}

void RunBelt(int power) {
  RunMotor(M1DPinA, M1DPinB, M1PPin, power);
}

void Drive(int M2Power, int M3Power){
  RunMotor(M2DPinA, M2DPinB, M2PPin, M2Power);
  RunMotor(M3DPinA, M3DPinB, M3PPin, M3Power); 
}

void DriveForTime(int LeftPower, int RightPower, int Time){
  Drive(LeftPower, RightPower);
  delay(Time);
  Drive(0, 0);
}

void setup() {

  pinMode(M1DPinA, OUTPUT);
  pinMode(M1DPinB, OUTPUT);
  pinMode(M1PPin,  OUTPUT);
  pinMode(M2DPinA, OUTPUT);
  pinMode(M2DPinB, OUTPUT);
  pinMode(M2PPin,  OUTPUT);
  pinMode(M3DPinA, OUTPUT);
  pinMode(M3DPinB, OUTPUT);
  pinMode(M3PPin,  OUTPUT);
}

void loop() {
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

  //analogWrite(7, 100);

  delay(1000);

  for (int i = 0; i < 50; i++){
    analogWrite(13, i);
    delay(25);
  }
  for (int i = 50; i > 0; i--){
    analogWrite(13, i);
    delay(25);
  }
}
