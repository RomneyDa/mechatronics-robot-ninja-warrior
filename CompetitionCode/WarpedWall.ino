
  void WWState() {

    MPow = 75;
    RunBelt(150);
    MoveRails(railsUp); //railsMR - 5

    if (!noLine) // If it sees a line
      LineFollow();                   // Line Follow

    // Else if hook not lifted, raise hook

    else if (t - tWW < 8)
      DriveAtSpeed(0.4, 0.4);    

    else
      Stop();
    
    // Else If wall is seen at the right distance, lower hook!

    // Once wall is lowered, transition

   // if (t - tWW > 7)
   //   RunBelt(0);

    // DONE!!!!

  }

