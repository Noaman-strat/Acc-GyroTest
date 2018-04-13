#include "Global.h"
#include "UI.h"
//#include "bcomdef.h"
//#include "OnBoard.h"

bool Flag_LED_LongOn = FALSE;

void Vibrate(unsigned char Vpulses)
{
  
}

void ChangeLED (uint8 LEDColor, uint8 Duration)
{
   if (Flag_LED_LongOn ==FALSE)
   {
      switch (LEDColor) 
      {
        case GreenLED:
          //turn red off, and green on
          PIN_setOutputValue(pinLEDGHandle, Board_GLED, 1);
          PIN_setOutputValue(pinLEDRHandle, Board_RLED, 0); 
        break;

        case RedLED:

          PIN_setOutputValue(pinLEDGHandle, Board_GLED, 0);
          PIN_setOutputValue(pinLEDRHandle, Board_RLED, 1); 
          break;

        case BothLEDsOff:

          PIN_setOutputValue(pinLEDGHandle, Board_GLED, 0);
          PIN_setOutputValue(pinLEDRHandle, Board_RLED, 0); 
          break;

      case BothLEDsON:
          PIN_setOutputValue(pinLEDGHandle, Board_GLED, 1);
          PIN_setOutputValue(pinLEDRHandle, Board_RLED, 1); 
        
        break;
        
        default:
          break;
      }
   }

  if ((Duration == DurationShort) && (LEDColor !=BothLEDsOff))
  {
    //for (m=0; m<40000; m++) asm("NOP"); 
    WAIT_70ms();
    //Task_sleep(5000);
    ChangeLED(BothLEDsOff,DurationShort);
  }
  else if ((Duration == Duration1sec) && (LEDColor !=BothLEDsOff))
  {
    Flag_LED_LongOn = TRUE;
    Start_LEDClock(1000);
  }
  else if ((Duration == Duration2sec) && (LEDColor !=BothLEDsOff))
  {
    Start_LEDClock(2000);
  }
  
  else if ((Duration == DurationInfinite) && (LEDColor !=BothLEDsOff))
  {
    //do nothing
    
  }
    
}
    
