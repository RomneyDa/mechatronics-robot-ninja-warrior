
  void SerialPrint(){

    Serial.print("Command: ");
    Serial.print(command);

    Serial.print('\t');
    Serial.print(message);

    Serial.print('\t');

 // Distance Sensor Values
    if (command == CMD_Dist){
      Serial.print("DFront: ");
      Serial.print(Dfront);
    
      Serial.print("\tDSide: ");
      Serial.print(Dside);
    }

 // Encoder (steady speed)
    if (command == CMD_Encoder){
      Serial.print("(Encoders) Pos: L = ");
      Serial.print(leftCount);
      Serial.print(",\tR = ");
      Serial.print(rightCount);
      //Serial.print(",\tBelt = ");
      //Serial.print(beltCount);
    }

 // Reflectance array values for Line Follow
    if (command == CMD_LineFollow){
      Serial.print("Array: ");
      
      for (i = 0; i < NUM_SENSORS; i++){
        Serial.print(sensorValues[i]);
        Serial.print(' ');
      }
    }

 // Driving a certain speed, distance, or circle
    if (command == CMD_Circle){
        Serial.print("(Radius) R = ");
        Serial.print(R);
    }

    if (command == CMD_DrivingS){
        Serial.print("Speed = ");
        Serial.print(R);
     }
    if (command == CMD_DrivingD){
        Serial.print("Dist = ");
        Serial.print(R);
    }

 // For anything that involves driving, show what the motors are doing
    if (command == CMD_Encoder || command == CMD_Circle || command == CMD_DrivingS || command == CMD_DrivingD){

   // Velocities 
      Serial.print("\tVel: L = ");
      Serial.print(leftVel);
      Serial.print(",\tR = ");
      Serial.print(rightVel);
      //Serial.print(",\tBelt = ");
      //Serial.print(beltVel);

   // Powers
      Serial.print("\tPwr: L = ");
      Serial.print(leftPower);
      Serial.print(",\tR = ");
      Serial.print(rightPower);
      //Serial.print(",\tBelt = ");
      //Serial.print(beltVel);

   // Speed Adjustments
      Serial.print("\tAdj: L = ");
      Serial.print(leftAdjust);
      Serial.print(",\tR = ");
      Serial.print(rightAdjust);
      //Serial.print(",\tBelt = ");
      //Serial.print(beltVel);
    }
    
    Serial.println();
    
  }

