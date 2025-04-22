

int cntr = 500;
int cntrr = 0;
void loop(void)
{
        cntrr++;
        isMaxGforce();
        if(cntr == 0)
        {
          calculateLandingVelocity();
          cntr--;
        }

        if(MaxGForce>5)
          cntr--;
        
      SerialUSB.print("laningvelo: ");
      SerialUSB.println(LandingVelocity);
            SerialUSB.print("cntrr: ");
      SerialUSB.println(cntrr);
                  SerialUSB.print("accel: ");
      SerialUSB.println(Gforce*9.81);
      
      
}