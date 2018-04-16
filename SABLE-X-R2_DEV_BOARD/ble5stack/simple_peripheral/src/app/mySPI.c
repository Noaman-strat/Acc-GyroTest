
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

#include "myUART.h"
#include "util.h"
#include "UI.h"


#include "board.h"

#define SPIACC_TASK_STACK_SIZE     2000
#define SPIACC_TASK_PRIORITY       2
uint8_t spiACCTaskStack[SPIACC_TASK_STACK_SIZE];

Task_Struct spiACCTask;


SPI_Handle AccSPI = 0;
uint8_t         transmitBuffer[MSGSIZE] ={0};
uint8_t         receiveBuffer[MSGSIZE] = {0};
SPI_Transaction spiTransaction;
bool            transferOK = 0;

double UpperTiltThreshold = 0.0;
double LowerTiltThreshold = 0.0;

bool Gyro_Started = true;
bool FirstTimeRun = true;
char Gyro_DataSetsToRead = 0;

signed short Gyro_X[20] = {0};
signed short Gyro_Y[20] = {0};
signed short Gyro_Z[20] = {0};
signed short Acc_X[20] = {0};
signed short Acc_Y[20] = {0};
signed short Acc_Z[20] = {0};

//float Angle_X[50] = {0};
//float Angle_Y[50] = {0};
//float Angle_Z[50] = {0};

//float AccAngle_X[50] = {0};
//float AccAngle_Y[50] = {0};
//float AccAngle_Z[50] = {0};

short Gyro_Offset_X = 0; //-40;
short Gyro_Offset_Y = 0; //380;
short Gyro_Offset_Z = 0; //400;


double AngleX1 = 0.0;
double AngleY1 = 0.0;
double AngleZ1 = 0.0;

unsigned int ACC_EVENT_PERIOD = 500;

bool Motion_Calibration_Running = false;

uint8 CurrentOrientation =0;
uint8 PreviousOrientation =0;



short CalculatedBeaconAngle = 0;

char CalibrationCounter = 0;
char HomePositionSet = 0;

enum devOrientation DeviceOrientation = NotIdentified;

bool EventRecord = true;

char IAmCalled = 0;

char RegInt = 0;

uint32_t standbyDurationMs = 100;

 signed short Gyro_Home_Position_X;
 signed short Gyro_Home_Position_Y;
 signed short Gyro_Home_Position_Z;
  
PIN_State pinCSState;
PIN_Handle pinCSHandle;
  





// Debounce timeout in milliseconds
#define INTPIN_DEBOUNCE_TIMEOUT  15


// Key debounce clock
static Clock_Struct LSM_INT_pinClock;

// Pointer to application callback
LSMIntPinCB_t appLSM_INTHandler = NULL;

// Memory for the GPIO module to construct a Hwi
Hwi_Struct callbackPINKeys;


/*********************************************************************
 * LOCAL FUNCTIONS
 */


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
  

  
  // Setup callback for INT pins
 // Util_constructClock(&LSM_INT_pinClock, LSM_INT_pinChangeHandler,
  //                   INTPIN_DEBOUNCE_TIMEOUT, 0, false, 0);

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
  
  Motion_Calibration_Running = true;
  HomePositionSet = false;
  
  AngleX1 = 0;
  AngleY1 = 0;
  AngleZ1 = 0;
  CalibrationCounter = 0;
  
  init_LSM6DSM();
  //init_LSM6DSM_AccONLY();  
  //Task_sleep(standbyDurationMs*10);
  //  LSM_Wakeup();
    
    while(1)
    {
      //Wait for time LARGER than ACC ODR
      Task_sleep(standbyDurationMs*100);  
      
      if ((CalibrationCounter >=10 ) && (!HomePositionSet))
      {
        //LSM6DSM_WriteReg(LSM6DSM_INT1_CTRL,0x00); //Clear FIFO Int.
        
       
        //set the home position
        Gyro_Home_Position_X = AngleX1;
       Gyro_Home_Position_Y = AngleY1;
       Gyro_Home_Position_Z = AngleZ1;

        //GetHomePositionOrientation();
        
        //start the ACC timer for regular operation
        Start_Acc1SecClock();

        ChangeLED(GreenLED, Duration2sec);
            //set the flags
        HomePositionSet = true;
        Motion_Calibration_Running = false;
        
      }
      /*
      if (HomePositionSet)
      {
        
        if (AngleX1 <0)
          TX_Data[0] = '-';
        myUart_covertChar((char) AngleX1,0);
        TX_Data[1] = tempchar1; TX_Data[2] = tempchar2; TX_Data[3] = tempchar3;
        
        if (AngleY1 <0)
          TX_Data[5] = '-';
        myUart_covertChar((char) AngleY1,0);
        TX_Data[6] = tempchar1; TX_Data[7] = tempchar2; TX_Data[8] = tempchar3;
        
        if (AngleZ1 <0)
          TX_Data[10] = '-';
        myUart_covertChar((char) AngleZ1,0);
        TX_Data[11] = tempchar1; TX_Data[12] = tempchar2; TX_Data[13] = tempchar3;
        

        UART_write(DebugUart, &TX_Data, 16);
      }
*/
      /*
      Gyro_FIFOStatus = LSM6DSM_ReadReg(LSM6DSM_FIFO_STATUS2);
      if (Gyro_FIFOStatus> 72)
      {
        Process_Int();
      }
      */
      /*
      RegInt = LSM6DSM_ReadReg(LSM6DSM_FIFO_STATUS2);
      
      if ((RegInt & BV(7)))
      {
        Process_Int();
        ChangeLED(GreenLED, DurationShort);       
      }
      else
      {
        
      }
      */
      /*
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
      */
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
/*
static void LSM_INTCallback(PIN_Handle hPin, PIN_Id pinId)
{
  LSM_INT = 0;

  if ( PIN_getInputValue(Board_LSM_INT1) == 1)
  {
    LSM_INT |= 0x0001;
    Process_Int();
  }

  if ( PIN_getInputValue(Board_LSM_INT2) == 1 )
  {
    Process_Int();
   // LSM_Clear_tiltInt();
    LSM_INT |= 0x0002;
    
  }

 // Util_startClock(&LSM_INT_pinClock);
}
*/

/*
static void LSM_INT_pinChangeHandler(UArg a0)
{
  if (appLSM_INTHandler != NULL)
  {
    // Notify the application
    (*appLSM_INTHandler)(LSM_INT);
  }
}
*/

void Process_Int(void)
{
  //CalibrationCounter++;
  //Gyro_FIFOStatus = LSM6DSM_ReadReg(LSM6DSM_FIFO_STATUS2);
  
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

  }
  
  
    CalibrationCounter++;
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
  int Avrg_gX = 0;
  int Avrg_gY = 0;
  int Avrg_gZ = 0; 
  
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

//checks to see if current asset position is in a different quadrant than the last time this routine was called
//if so, start the Gyro at 12.5 Hz to record and analyze the movement
//the initial Gyro reading is taken when the Device is Woken up after install.
void Check_for_Quadrant_Change(void)
{
  //checK if Accelerometer has detected movement
  
  if (!Gyro_Started)
  { 
    AngleX1=0;
    AngleY1=0;
    AngleZ1=0;
    LSM_Sleep_Gyro();
    Task_sleep(2000);
    init_Gryo_FIFO();
    Task_sleep(2000);
    LSM_Wakeup_Gyro();
  
   
  }
  
  //StartMotionCalibration();
  //If Gyro is already recording
  
  if (Gyro_Started)
  { 
    //check FIFO status
    Gyro_FIFOStatus = LSM6DSM_ReadReg(LSM6DSM_FIFO_STATUS2);
    if (!(Gyro_FIFOStatus & FIFO_NOT_EMPTY) && (Gyro_FIFOStatus & FIFO_DATA_THRESHOLD_REACHED)) //if FIFO is not empty
    {
      CalculateAngle(ReadGyroData());
    }
  }
  
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
      AngleX1 += ((double)Gyro_X[gi] * GyroDivisor125);
      AngleY1 -= ((double)Gyro_Y[gi] * GyroDivisor125); //-negative sign works
      AngleZ1 += ((double)Gyro_Z[gi] * GyroDivisor125);
      
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
        
        diffAngleX = abs(abs(AngleX1) - abs(Gyro_Home_Position_X));
        diffAngleY = abs(abs(AngleY1) - abs(Gyro_Home_Position_Y));
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
        
        diffAngleZ = abs(abs(AngleZ1) - abs(Gyro_Home_Position_Z));
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
