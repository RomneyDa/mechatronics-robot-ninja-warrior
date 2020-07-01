void MoveRails(int Pos) {

  if(t - oldServoTime > servoDelay) {

    if (Pos > monkeyPos)
      monkeyPos++;
      
    else if (Pos < monkeyPos)
      monkeyPos--;

    MoveServos(monkeyPos);
    
    oldServoTime = t;
  }
}

void MoveServos(int pos){

    railsServoL.write(pos);
    railsServoR.write(180 - pos); // Facing opposite direction
  
}


