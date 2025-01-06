#include <iostream>
#include <fstream>
#include <vector>
#include <valarray>
#include <Forge.h>

Forge *forge;
void setup()
{
  //Instantiate forge object and fill variables
  forge = new Forge();
  forge.getAltitude();
  forge.getAltitude();
  forge.getVelocity();

  //FSM instantiateion
  State currentState = PAD;
  State nextState = FLIGHT;
  enum State {PAD, FLIGHT, LAND, TRANSMIT, SHUTDOWN};
}



void loop()
{
  switch (currentState) 
  {
	
    case PAD:
    {
      if(forge.getChangeInAltitude() >= 10) //if altitude change is significant enough (not just moving rocket around but an actual liftoff) go to flight stage
      {
        currentState = nextState;
        nextState = LAND;
      }

      if(forge.getShutdownStatus())
      {
        currentState = SHUTDOWN;				
      }
    }

    case FLIGHT:
    {
      forge.isMaxAltitude();
      forge.isMaxGforce();
      forge.isMaxVelocity();

      if((forge.getChangeInAltitude() <= 10) && forge.currentAltitude() < 304.8) //checks for conditions signifying that landing has happened (304.8m = 1000ft)
      {
        forge.recordTime();
        forge.calculateLandingVelocity();
        currentState = nextState;
        nextState = TRANSMIT;
      }

      if(forge.getShutdownStatus())
      {
        currentState = SHUTDOWN;				
      }
    }

    case LAND:
    {
      forge.recordTemperature();
      forge.recordOrientation();
      forge.recordBatteryStatus();
      forge.calculateSurvivalChance();
      currentState = nextState;
      nextState = SHUTDOWN; //no shutdown check because will never stay in this state
    }

    case TRANSMIT:
    {
      if(forge.timer())
      {
        forge.transmitData();
      }
      else
      {
        currentState = nextState;
      }

      if(forge.getShutdownStatus())
      {
        currentState = SHUTDOWN;				
      }
    }

    case SHUTDOWN:
    {
      forge.shutdown();
    }

  }
}
