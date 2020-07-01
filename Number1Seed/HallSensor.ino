
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

  GetHall();
  Stop();
  RunBelt(0);

  if ((stage == 1 || stage == 3 || stage == 4 || stage == 5) && monkeyPos != railsUp)
    MoveRails(railsUp);

  if (stage == 2 && monkeyPos != railsForward)
    MoveRails(railsForward);

  if (GetHall()){
    command = CMD_Running;
   // Wall Follow / Paddleboard
      
    if (stage == 1){
      tWF = t;
      thresholdDist = 0.8;
    }
      
    if (stage == 2)  // Wall Lift
      tDelay = t;
      

    else if (stage == 3){  // U-turn
      tLDelay = t;
      thresholdDist = 0.5;
  }
      
      
    else if (stage == 4){  // Monkey Rails
      tMDelay = t;
      pastLine = false;
      driveHard = false;
    }

    else if (stage == 5)
      tWW = t;
  }
  
}

