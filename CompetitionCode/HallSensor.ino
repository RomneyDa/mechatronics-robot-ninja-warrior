
bool GetHall(){

    hall = analogRead(hallSensorPin);
    hallScaled = (hall - hallMid)*100.0/hallMid;    // Scale from -100 to 100 ish
    
    if (hall > hallMid*(1 + hallTol) || hall < hallMid*(1 - hallTol))
        hallOn = true;
    else 
        hallOn = false;

    return hallOn;
}

void WaitForHall(){

  //cmdNum = 0;

  if (monkeyPos != railsUp)      // First, put the rails assembly down
      MoveRails(railsUp);

  if (GetHall()){
    command = CMD_Running;
   // Wall Follow / Paddleboard
      
    if (stage == 1)
      tWF = t;
      
    if (stage == 2)  // Wall Lift
      tDelay = t;
      

    else if (stage == 3)  // U-turn
      tLDelay = t;
      
      
    else if (stage == 4){  // Monkey Rails
      tMDelay = t;
      pastLine = false;
      driveHard = false;
    }

    else if (stage == 5)
      tWW = t;
  }
  
}

