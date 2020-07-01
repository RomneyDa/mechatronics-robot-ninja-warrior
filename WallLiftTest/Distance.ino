
  void GetDistance()
  {    
      
      // Read distance sensors and convert voltage to distance (cm)
      // Fit coefficients are experimentally derived

      // Front sensor
      Vfront = analogRead(frontSensorPin);
      Dfront = Cfront*pow(Vfront, Bfront);
      
      if (Dfront > frontMax)                // Limit sensor value to maximum
        Dfront = frontMax;

      // Side sensor
      Vside = analogRead(sideSensorPin);
      Dside = Cside*pow(Vside, Bside);
      
      if (Dside > sideMax)                  // Limit sensor value to maximum
        Dside = sideMax;
  }
