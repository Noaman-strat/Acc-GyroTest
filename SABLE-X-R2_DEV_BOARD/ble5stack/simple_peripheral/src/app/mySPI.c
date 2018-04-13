
#include "mySPI.h"
#include "hw_types.h"

#include <hal_defs.h>
#include <stdlib.h>
//#include <ti/sysbios/knl/Clock.h>
//#include <ti/sysbios/family/arm/m3/Hwi.h>


#include <stdbool.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
//#include "LSM6DSM.h"
#include "Global.h"
#include "myUART.h"
#include "util.h"
#include "UI.h"
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
  //init_LSM6DSM_AccONLY();  
  Task_sleep(standbyDurationMs*10);
  //  LSM_Wakeup();
    
    while(1)
    {
      //Wait for time LARGER than ACC ODR
      Task_sleep(standbyDurationMs*200);   
      
      RegInt = LSM6DSM_ReadReg(WRIST_TILT_IA);
      
      if ((RegInt & BV(7)) || (RegInt & BV(6)))
      {
        LSM_Clear_tiltInt();
        ChangeLED(GreenLED, DurationShort);       
      }
      else
      {
        
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


void Process_Int(void)
{
  
  Gyro_FIFOStatus = LSM6DSM_ReadReg(LSM6DSM_FIFO_STATUS2);
  if (!(Gyro_FIFOStatus & FIFO_NOT_EMPTY) && (Gyro_FIFOStatus & FIFO_DATA_THRESHOLD_REACHED)) //if FIFO is not empty
  {
    CalculateAngle(ReadGyroData());
  }

  if (CalibrationCounter ==4)
  {
    Gyro_Offset_X = Gyro_X[5];
    Gyro_Offset_Y = Gyro_Y[5];
    Gyro_Offset_Z = Gyro_Z[5];
    CalibrationCounter++;
  }

  else if (CalibrationCounter <10)
  {
    CalibrationCounter++;
    //AngleX1 = 0;
    //AngleY1 = 0;
   // AngleZ1 = 0;
  }
  else if (CalibrationCounter ==10)
  {
    //LSM6DSM_WriteReg(LSM6DSM_INT1_CTRL,0x00); //Clear FIFO Int.
    
   
    //set the home position
    Gyro_Home_Position->AXIS_X = AngleX1;
    Gyro_Home_Position->AXIS_Y = AngleY1;
    Gyro_Home_Position->AXIS_Z = AngleZ1;
    
    //set the flags
    HomePositionSet = true;
    Motion_Calibration_Running = false;
       
    GetHomePositionOrientation();
    CalibrationCounter++;
    
    //start the ACC timer for regular operation
    Start_Acc1SecClock();

    ChangeLED(GreenLED, Duration2sec);
    
  }
    
}
//This routine performs motion callibration to:
//1- reset flags
//2- Reinitialize the ACC+GYRO
//3- Setup FIFO
//4- Enable Interrupts
// the ISR takes over from this point to get the home position
void StartMotionCalibration(void)
{

  uint8 value, valueL, valueH;
  
  
  Motion_Calibration_Running = true;
  HomePositionSet = false;
  
  AngleX1 = 0;
  AngleY1 = 0;
  AngleZ1 = 0;
  //1 - strat the Gyro and Accelerometer in Fifo Mode
  //set the interrupt 
  //we will run the FiFo cycle (1 second long at 12.5Hz) twice to get reliable home
  //position reading
  CalibrationCounter = 0;       //when this reaches 2, the callibration/home routine ends
  
  init_LSM6DSM();       //reinitialize ACC+GYRO
  init_Gryo_FIFO();     //setup FIFO
  
  //setup FIFO Interrupt on INT1
  LSM6DSM_WriteReg(LSM6DSM_DRDY_PULSE_CFG, 0x80); //pulse the interrupt
  LSM6DSM_WriteReg(LSM6DSM_INT1_CTRL,0x08); //Fifo threshold interrupt on Int_1
  
  //strat
  

}

//This routine determines the home position orientation by determining gravity direction
//This is called right after HOME position has been acquired.
//The idea is that the rotation will always be around a specific Axis during operation
//other Axes can be ignored
void GetHomePositionOrientation(void)
{
  int Avrg_gX = 0.0;
  int Avrg_gY = 0.0;
  int Avrg_gZ = 0.0; 
  
  /*
  //use the most recent Acc_X, Acc_y and Acc_Z values 
  char numOfRecords = 12; //LSM6DSM_FIFO_THRESHOLD/6; //3 Acc + 3 Gyro 
  
  for (char k=0; k< numOfRecords; k++)
  {
    Avrg_gX = Avrg_gX + abs(Acc_X[k]);
    Avrg_gY = Avrg_gY + abs(Acc_Y[k]);
    Avrg_gZ = Avrg_gZ + abs(Acc_Z[k]);
  }
  */
  //get the average
  Avrg_gX = abs(Acc_X[5]); // Avrg_gX / numOfRecords;
  Avrg_gY = abs(Acc_Y[5]); // Avrg_gY / numOfRecords;
  Avrg_gZ = abs(Acc_Z[5]); // Avrg_gZ / numOfRecords;
 
  //device is mounted on a flat horizontal surface (like ona table)
  //Rotation will be about X-axis
  if (Avrg_gZ > 14000)
    DeviceOrientation = Orientation_X;
  
  //device is mounted on the wall, horizontally.
  //Rotation will be about Z-axis
  else if (Avrg_gX > 14000)
    DeviceOrientation = Orientation_Z;
  
  //device is moutned on the wall, vertically.
  //Rotation will be about Z-axis
  else if (Avrg_gY > 14000)
    DeviceOrientation = Orientation_Z;

}

void CalculateAngle(char datasets)
{
  double AngleAcc;
  int AccMag;
  double diffAngleX = 0;
  double diffAngleY = 0;
  double diffAngleZ = 0;
   

    for (char gi=0; gi < datasets; gi++)      
    {     
      //Gyro Angle Calculation
      AngleX1 += ((double)Gyro_X[gi] * GyroDivisor500);
      AngleY1 -= ((double)Gyro_Y[gi] * GyroDivisor500); //-negative sign works
      AngleZ1 += ((double)Gyro_Z[gi] * GyroDivisor500);
      
      //Acc Angle Calculation
      //if Acc change is between 0.2 (3276) and 1.4g (19660) only then do the calculation.
      //otherwise it is just noisy data or rapid acceleration due to 
      AccMag = abs(Acc_X[gi]) +  abs(Acc_Y[gi]) + abs(Acc_Z[gi]);
      if ((AccMag> 8000) && (AccMag < 29000))
      {
        // Turning around the X axis results in a vector on the Y-axis
        AngleAcc = atan2((double)Acc_Y[gi], (double)Acc_Z[gi]) * 57.295;
        AngleX1 = (AngleX1 * 0.98) + (AngleAcc * 0.02);
      
        // Turning around the Y axis results in a vector on the X-axis
        AngleAcc = atan2((double)Acc_X[gi], (double)Acc_Z[gi]) * 57.295;
        AngleY1 = (AngleY1 * 0.98) + (AngleAcc * 0.02);

        // Turning around the Z axis results in a vector on the X-axis
        AngleAcc = atan2((double)Acc_X[gi], (double)Acc_Y[gi]) * 57.295;
        AngleZ1 = (AngleZ1 * 0.98) + (AngleAcc * 0.02);          
      }  
    }
    
    //only do this if we have already done calibration
    //this will prevent the Interrupt routine from waiting for the LED
    //potentially causing the Interrupt to be queued/missed
    if (HomePositionSet)
    {
      switch (DeviceOrientation)
      {
      case    Orientation_X:
        
        diffAngleX = abs(abs(AngleX1) - abs(Gyro_Home_Position->AXIS_X));
        diffAngleY = abs(abs(AngleY1) - abs(Gyro_Home_Position->AXIS_Y));
        if (diffAngleX< 20)
        {
          EventRecord = false;
        }
        else if ((diffAngleX> 30) && (diffAngleX < 90) && (!EventRecord))
        {
          //blink the LED
          ChangeLED(GreenLED, DurationShort);
        }
        else if ((diffAngleX > 90) && (!EventRecord))
        {
          EventRecord = true;
          //blink the LED
          if (!Flag_LED_LongOn)
          {
            ChangeLED(GreenLED, Duration2sec);
            
            /*
            MsgTypeField = JobEndState;
            update_GeotabPacket();
            Button1Feature1();
            JobIsValid=false; 
            */
          }
 
        }
     
        else if ((diffAngleY> 30) && (diffAngleY < 90) && (!EventRecord))
        {
          //blink the LED
          ChangeLED(GreenLED, DurationShort);
        }
        else if ((diffAngleY > 90) && (!EventRecord))
        {
          EventRecord = true;
          //blink the LED
          if (!Flag_LED_LongOn)
          {
            ChangeLED(GreenLED, Duration2sec);
            
            /*
            MsgTypeField = JobEndState;
            update_GeotabPacket();
            Button1Feature1();
            JobIsValid=false;
            */
          }
        }
        break;
        
      case Orientation_Z:    
        
        diffAngleZ = abs(abs(AngleZ1) - abs(Gyro_Home_Position->AXIS_Z));
        if (diffAngleZ< 20)
        {
          EventRecord = false;
        }
        else if ((diffAngleZ> 30) && (diffAngleZ < 90) && (!EventRecord))
        {
          //blink the LED
          ChangeLED(GreenLED, DurationShort);
        }
        else if ((diffAngleZ > 90) && (!EventRecord))
        {
          EventRecord = true;
          //blink the LED
          
          ChangeLED(GreenLED, Duration2sec);
          
          if (!Flag_LED_LongOn)
          {
            ChangeLED(GreenLED, Duration2sec);
            
            /*
            MsgTypeField = JobEndState;
            update_GeotabPacket();
            Button1Feature1();
            JobIsValid=false;
            */
            
          }
        }
        break;
        
      default:
        break;
      }
    }
  
  
}
