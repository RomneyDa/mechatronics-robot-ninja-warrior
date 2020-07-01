
  void MRState (){
    
    RunBelt(95);
    servoDelay = 0.02;

    if(driveHard){
        
      if (!noLine){              // If it sees a line, transition
        if (t - tMDelay > 4){
          stage = 5;
          tWW = t;
        }
        if(!pastLine) {
          MoveRails(railsMR);
          LineFollow();
        }
      }
      
      else if(t - tMDelay > 3.0) {
          Drive(35, 35);
      }
  
      if(noLine)
         pastLine = true;
  
      if (t - tMDelay < 2.5)
         MoveRails(railsUp - 20);
         
      else {
         MoveRails(railsMR);
         MPow = 60;
      }
    }

    else {
      Drive(60, 60);
      delay(500);
      driveHard = true;
    }
  }

