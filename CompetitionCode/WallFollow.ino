  void WallFollow(){
        
        Derror = Dfront - WFDist;
        Dvel = (Derror - Derror_old) / (t - t_oldD);

        WFAdjust = KpDist*Derror + KdDist*Dvel;

          if (Derror < 0)
            Drive( WFSpeed, WFSpeed + WFAdjust);
          else
            Drive( WFSpeed - WFAdjust, WFSpeed);

        Derror_old = Derror;
        t_oldD = t;

  }
  

void WFState(){

  if (monkeyPos != railsForward)         // First, put the rails assembly down
      MoveRails(railsForward);
      
   else if (Dside >= 12 && !noLine){ // If the side distance sensor sees nothing and it sees a line, transition to Wall Lift
     stage = 2;
     tDelay = t;
   }

  //else if (noLine && Dside > sideMax){
  //  stage = 2;
  //  tDelay = t;
  //}

  else if (Dfront < frontMax && t - tWF < 5.8){                                              // If the front distance sensor sees something, wall follow
    WallFollow();    
    MPow = 60;
  }
  
  else if (!noLine && t - tWF > 7){                      // Otherwise, if it sees a line, line follow (until side distance sees nothing!)
    LineFollow();
    MPow = 50;
  }
  else {                                 // Otherwise, drive straight forward
    Drive(60, 60);
  }   

}




  void WallFollow2(){

      if (Dfront >= frontMax)                // If there's no wall       
        Drive(0, 0);                         // Stop
        
      else {                                 // LINE FOLLOW MODE
        if (Dfront - WFDist > Dthresh)            // If the robot is to the right of the line
          TurnLeft(MPow);                    // Turn left
        
        else if (Dfront - WFDist < -Dthresh)     // If the robot is to the left of the line   
          TurnRight(MPow);                   // Turn right
 
        else                                 // Otherwise
          Drive(MPow, MPow);                 // Go Straight
       
      }


  }
