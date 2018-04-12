
#include "mySPI.h"
#include "hw_types.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <hal_defs.h>
//#include <ti/sysbios/knl/Clock.h>
//#include <ti/sysbios/family/arm/m3/Hwi.h>


#include <stdbool.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include "LSM6DSM.h"
#include "Global.h"
#include "myUART.h"
#include "util.h"
//#include <ti/drivers/pin/PINCC26XX.h>

#include "board.h"

#define SPIACC_TASK_STACK_SIZE     1000
#define SPIACC_TASK_PRIORITY       2
uint8_t spiACCTaskStack[SPIACC_TASK_STACK_SIZE];

Task_Struct spiACCTask;


SPI_Handle AccSPI = 0;
uint8_t         transmitBuffer[MSGSIZE] ={0};
uint8_t         receiveBuffer[MSGSIZE] = {0};
SPI_Transaction spiTransaction;
bool            transferOK = 0;


char IAmCalled = 0;

char RegInt = 0;

uint32_t standbyDurationMs = 100;
  
PIN_State pinCSState;
PIN_Handle pinCSHandle;
  
PIN_State  LSM_INT1_pin;
PIN_Handle LSM_INT1_pin_handle;

PIN_State  LSM_INT2_pin;
PIN_Handle LSM_INT2_pin_handle;

// Debounce timeout in milliseconds
#define INTPIN_DEBOUNCE_TIMEOUT  100
  // Value of INT Pin generated in LSM
static uint8_t LSM_INT;

// Key debounce clock
static Clock_Struct LSM_INT_pinClock;

// Pointer to application callback
LSMIntPinCB_t appLSM_INTHandler = NULL;

// Memory for the GPIO module to construct a Hwi
Hwi_Struct callbackPINKeys;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void LSM_INT_pinChangeHandler(UArg a0);
static void LSM_INTCallback(PIN_Handle hPin, PIN_Id pinId);


  // PIN configuration structure to set all KEY pins as inputs with pullups enabled
PIN_Config LSM_INT_PinsCfg[] =
{
    Board_LSM_INT1          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN |  PIN_PULLUP,
    Board_LSM_INT2          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN |  PIN_PULLUP,
    PIN_TERMINATE
};

//bool mySPI_Config(void)
static void spiThread(UArg s_arg0, UArg s_arg1)
{
  SPI_Params      spiParams;
  SPI_init();  // Initialize the SPI driver
  SPI_Params_init(&spiParams);  // Initialize SPI parameters
  spiParams.bitRate = 1000000;
  spiParams.frameFormat = SPI_POL1_PHA1;
  spiParams.mode = SPI_MASTER;
  spiParams.transferMode = SPI_MODE_CALLBACK;
  spiParams.transferCallbackFxn = mySPI_Callback;
 
  spiParams.dataSize = 8;       // 8-bit data size
  AccSPI = SPI_open(Board_SPI0, &spiParams);
  
  
  //Initialize LSM6DSM INT1 Interrupt
    // Initialize KEY pins. Enable int after callback registered
  LSM_INT1_pin_handle = PIN_open(&LSM_INT1_pin, LSM_INT_PinsCfg);
  PIN_registerIntCb(LSM_INT1_pin_handle, LSM_INTCallback);
  
  LSM_INT2_pin_handle = PIN_open(&LSM_INT2_pin, LSM_INT_PinsCfg);
  PIN_registerIntCb(LSM_INT2_pin_handle, LSM_INTCallback);
  
  PIN_setConfig(LSM_INT1_pin_handle, PIN_BM_IRQ, Board_LSM_INT1 | PIN_IRQ_NEGEDGE);
  PIN_setConfig(LSM_INT2_pin_handle, PIN_BM_IRQ, Board_LSM_INT2 | PIN_IRQ_NEGEDGE);
  
  // Setup callback for INT pins
  Util_constructClock(&LSM_INT_pinClock, LSM_INT_pinChangeHandler,
                     INTPIN_DEBOUNCE_TIMEOUT, 0, false, 0);

  // Set the application callback
  //appKeyChangeHandler = appKeyCB;
  
  if (AccSPI == NULL) {
      while (1);  // SPI_open() failed
  }
  // Fill in transmitBuffer

  pinCSHandle = PIN_open(&pinCSState, &BoardGpioInitTable[2]);
//  if (read_WHOAMI()== true)
//  {
    //Setup LSM6DSM
  read_WHOAMI();
    init_LSM6DSM();
    Task_sleep(standbyDurationMs*10);
  //  LSM_Wakeup();
    
    while(1)
    {
      //Wait for time LARGER than ACC ODR
      Task_sleep(standbyDurationMs*200);   
      
      RegInt = LSM6DSM_ReadReg(WRIST_TILT_IA);
      
      if ((RegInt && BV(7)) || (RegInt && BV(6)))
      {
        LSM_Clear_tiltInt();
        PIN_setOutputValue(pinLEDGHandle, Board_GLED, 1);
        PIN_setOutputValue(pinLEDRHandle, Board_RLED, 0);        
      }
      else
      {
        PIN_setOutputValue(pinLEDGHandle, Board_GLED, 0);
        PIN_setOutputValue(pinLEDRHandle, Board_RLED, 1);
      }
     /* 
      if (Acc_Moving()==true)
      {
        PIN_setOutputValue(pinLEDGHandle, Board_GLED, 1);
        PIN_setOutputValue(pinLEDRHandle, Board_RLED, 0);
      }
      else
      {
        PIN_setOutputValue(pinLEDGHandle, Board_GLED, 0);
        PIN_setOutputValue(pinLEDRHandle, Board_RLED, 1);
      }
      
      UART_write(DebugUart, &TX_Data, 8);
*/
    }
  //}
}

/*
bool read_WHOAMI(void){

  transmitBuffer[0] = 0x8F;
  transmitBuffer[1] = 0x00;
 //    receiveBuffer[0] = 0x8F;
 // receiveBuffer[1] = 0x00;
  spiTransaction.count = 2;
  spiTransaction.txBuf = transmitBuffer;
  spiTransaction.rxBuf = receiveBuffer;
        PIN_setOutputValue(pinCSHandle, Board_SPI0_CSN, 0);

  transferOK = SPI_transfer(AccSPI, &spiTransaction);
  if (!transferOK) 
  {
  }
 
  return true;
}
*/

static void mySPI_Callback (SPI_Handle sHandle, SPI_Transaction *spiTransaction)
{
  IAmCalled = spiTransaction->count; 
  PIN_setOutputValue(pinCSHandle, Board_SPI0_CSN, 1);
 // memcpy(receiveBuffer, spiTransaction->rxBuf,2);
  //receiveBuffer[0] = spiTransaction->rxBuf;

}

void spiACC_createTask(void)
{
  Task_Params spiACCtaskParams;

  // Configure task
  Task_Params_init(&spiACCtaskParams);
  spiACCtaskParams.stack = spiACCTaskStack;
  spiACCtaskParams.stackSize = SPIACC_TASK_STACK_SIZE;
  spiACCtaskParams.priority = SPIACC_TASK_PRIORITY;

  Task_construct(&spiACCTask, spiThread, &spiACCtaskParams, NULL);

}


/*********************************************************************
 * @fn      Board_keyCallback
 *
 * @brief   Interrupt handler for Keys
 *
 * @param   none
 *
 * @return  none
 */

static void LSM_INTCallback(PIN_Handle hPin, PIN_Id pinId)
{
  LSM_INT = 0;

  if ( PIN_getInputValue(Board_LSM_INT1) == 0 )
  {
    LSM_INT |= 0x0001;
  }

  if ( PIN_getInputValue(Board_LSM_INT2) == 0 )
  {
    LSM_Clear_tiltInt();
    LSM_INT |= 0x0002;
    
  }

  Util_startClock(&LSM_INT_pinClock);
}


static void LSM_INT_pinChangeHandler(UArg a0)
{
  if (appLSM_INTHandler != NULL)
  {
    // Notify the application
    (*appLSM_INTHandler)(LSM_INT);
  }
}
