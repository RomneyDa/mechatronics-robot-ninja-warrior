
  void WallFollow(){

     if (monkeyPos == railsForward){

        message = "Following Wall";
        
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

     else
        MoveRails(railsForward);
  }

