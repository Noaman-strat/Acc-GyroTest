 
#ifndef GLOBAL_H
#define GLOBAL_H

#ifdef __cplusplus
extern "C" {
#endif


/*********************************************************************
 * INCLUDES
 */
  
#include <ti/drivers/PIN.h>
#include "PlaneRemote.h"
  
  
//********************************************************************
/*---------Timer Defines-----------------------
---------------------------------------------*/
#define WAIT_100us()      {for(unsigned short i=0;i<91;i++)asm("NOP"); }
#define WAIT_50us()       {for(unsigned short i=0;i<45;i++)asm("NOP"); }      
#define WAIT_10us()      {for(unsigned short i=0;i<9;i++)asm("NOP"); }
#define WAIT_1ms()      {for(unsigned short i=0;i<914;i++)asm("NOP"); }
#define WAIT_10ms()     {for(unsigned long i=0;i<9140;i++)asm("NOP"); }
#define WAIT_50ms()      {for(unsigned long i=0;i<45700;i++)asm("NOP"); }
#define WAIT_70ms()      {for(unsigned long i=0;i<64000;i++)asm("NOP"); } //measured with a scope
#define WAIT_100ms()      {for(unsigned long i=0;i<91400;i++)asm("NOP"); }
#define WAIT_500ms()      {for(unsigned long i=0;i<457142;i++)asm("NOP"); }
   
   
extern  PIN_State pinCSState;
extern  PIN_Handle pinCSHandle;

extern PIN_State pinLEDRState;
extern PIN_State pinLEDGState;

extern PIN_Handle pinLEDRHandle;
extern PIN_Handle pinLEDGHandle;
  
extern unsigned char checkForTilt;
extern long SettableAccThreshold;
extern long SettableTiltThreshold;
  


extern unsigned char TX_Data[9];
  #ifdef __cplusplus
}
#endif

#endif /* GLOBAL_H */