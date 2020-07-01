
#define frontSensorPin A4
#define sideSensorPin A5

int Vfront, Vside;
double Dfront, Dside;

int Cfront = 2500;
int Bfront = -1;

int Cside = 1000;
int Bside = -1;

void setup() {
  
  pinMode(frontSensorPin, INPUT);
  pinMode(sideSensorPin,  INPUT);
  
  Serial.begin(9600);
  
}

void loop() {
  
  Vfront = analogRead(frontSensorPin);
  Dfront = Cfront*pow(Vfront, Bfront);

  Vside = analogRead(sideSensorPin);
  Dside = Cside*pow(Vside, Bside);

  Serial.print("Front: ");
  Serial.print(Vfront);
  Serial.print(" --> D = ");
  Serial.print(Dfront);
  
 // Serial.print('\t');

  Serial.print("   Side: ");
  Serial.print(Vside);
  Serial.print(" --> D = ");
  Serial.println(Dside);
  
}
