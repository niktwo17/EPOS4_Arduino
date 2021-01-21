#include <Arduino.h>
#include <epos.h>

epos EPOS4;
enum states{INIT, IDLE, AUTONOMOUS, MANUAL, ERROR} STATE, LastNoErrorSTATE;


void setup() 
{
  EPOS4.init(); 
  STATE = INIT;
}

void loop() 
{

  float RadIs = EPOS4.getCurrentPosition();
  if(RadIs == (-10)){STATE = ERROR;}

  // simple state machine to control the motors function depending on the current state
  switch (STATE)
  {
    case INIT:    
      if(EPOS4.doHoming() == true){STATE =MANUAL;}
      LastNoErrorSTATE = INIT;
      break;
    case IDLE:
      LastNoErrorSTATE = INIT;
      break;
    case AUTONOMOUS:
      LastNoErrorSTATE = INIT;
      break;
    case MANUAL:
      LastNoErrorSTATE = INIT;
      break;
    case ERROR:
      if (EPOS4.getCurrentPosition() != (-10)){STATE = LastNoErrorSTATE;}
    default:
      break;
  }

  // actually controls the motor
  EPOS4.steeringLoop(STATE, -0.5, 0.5);
  delay(50);
}