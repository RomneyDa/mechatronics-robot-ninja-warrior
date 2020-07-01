
  void SerialPrint(){

    Serial.print("[CMD = ");
    Serial.print(command);

    Serial.print("]\t");
    Serial.print(message);

    Serial.print('\t');

 // Distance Sensor Values
    if (command == CMD_Dist || command == CMD_FollowingW){
      Serial.print("DFront: ");
      Serial.print(Dfront);
    
      Serial.print("\tDSide: ");
      Serial.print(Dside);
    }

 // Wall Follow
    if (command == CMD_FollowingW){
      Serial.print("\tDerror: ");
      Serial.print(Derror);
    
      Serial.print("\tSpeed Adjustment: ");
      Serial.print(KpDist*Derror);
    }

// Hall Sensor Readout
    if (command == CMD_Hall){
      Serial.print("Mid Value: ");
      Serial.print(hallMid);
    
      Serial.print("\tValue: ");
      Serial.print(hall);

      Serial.print("\tScaled: ");
      Serial.print(hallScaled);

      Serial.print("\tReturn: ");
      Serial.print(hallOn);
    }

 // BeltMotor
    if (command == CMD_MoveBelt){
      Serial.print("Belt Power = ");
      Serial.print(beltPower);
      Serial.print(" (-255 to 255)");
    }

 // Reflectance array values for Line Follow
    if (command == CMD_LineFollow){
      Serial.print("Array: ");
      
      for (i = 0; i < NUM_SENSORS; i++){
        Serial.print(sensorValues[i]);
        Serial.print(' ');
      }

      Serial.print("\tDist = ");
      Serial.print(dist);
    }

 // Driving a certain speed, distance, or circle
    if (command == CMD_Circle){
        Serial.print("(Radius) R = ");
        Serial.print(R);
    }

    if (command == CMD_DrivingS){
        Serial.print(R);
     }
    if (command == CMD_DrivingD){
        Serial.print("Dist = ");
        Serial.print(R);
    }

 // Servos
    if (command == CMD_MoveRails){
        Serial.print("Desired Angle: ");
        Serial.print(pos0);

        Serial.print("\tCurrent Angle: ");
        Serial.print(monkeyPos);
    }

 // For anything that involves driving, show what the motors are doing
    if (command == CMD_Circle || command == CMD_DrivingS || command == CMD_DrivingD){

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

