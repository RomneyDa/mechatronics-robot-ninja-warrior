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

    railsServoL.write(pos + 8);  // Adjusted by 10 degrees because hubs can't line up (~20 degrees / spoke), facing opposite
    railsServoR.write(180 - pos); // Facing opposite direction
  
}


