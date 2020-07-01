
  void WLState(){

    servoDelay = 0.01;
    
    if (t - tDelay < 2.5)        // In the first second, put the rails assembly down          
      MoveRails(railsForward); 
    else {
      RunBelt(-255);
      MoveServos(railsUp);     // After that, move the servos up!
    }

    if (Dside < sideMax && !noLine && t - tDelay > 2.5){    // If it sees the wall post to the side and it sees a line
      tLDelay = t;
      thresholdDist = 0.5;
      stage = 3;                        // Transition!
    }
         
    if (!noLine && t - tDelay > 3.3)   // If it sees a line and it's been 1.4 seconds,
        LineFollow();                   // Follow the line
    else if (t - tDelay > 2.5)
        Drive(200, 200);                // Otherwise, full power ahead!
    else 
        Drive(50, 50);
  }

