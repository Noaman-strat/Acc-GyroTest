/*
//set the weight of offset to 1
2-6g/LSB if the USR_OFF_W bit is set to 1.

1) Take a reading. Discard it
2) Wait and Read again
3) check X, Y and Z values
4) perform calibration if X and Y not close to 0
5) perform calibration if Z not close to 1g

0-32767 = 0 to 2g
32768-65535 = -2g to 0 (larger decimal value is smaller g value

signed scale goes from 0 to 127, -128 to -1
signed scale goes from 0 to 32767, -32767 to -1
32768-65535-0-32767
0 - 32767 = 0 to 2g
-32767 - 0 = 0 to -2g
*/

#include "LSM6DSM.h"
//#include <ti/drivers/PIN.h>n
//#include <ioCC2540.h>
#include "OnBoard.h"
#include "stdlib.h"
//#include "UART.h"
#include "Global.h"
#include "mySPI.h"
#include "hal_mcu.h"
//#include "simpleBLEPeripheral.h"
//#include "PacketUpdate.h"
#include "float.h"
#include "myUART.h"
#include <ti/sysbios/knl/Task.h>


// Accelerometer connected at (names in brackets are LIS3DH names)
// P0_4 = CS_N (CSB)
// P0_5 = SCK (SCx)
// P0_3 = MISO (SDO)
// P0_2 = MOSI (SDx)
// P1_7 = (INT1)

#define CS     P0_4
#define SCK    P0_5
#define MISO   P0_3
#define MOSI   P0_2

#define CS_DISABLED     P0 |= 0x10;
#define CS_ENABLED      P0 &= 0xef;


int16 AXIS_X_Previous=0;
int16 AXIS_Y_Previous=0;
int16  AXIS_Z_Previous=0;

AxesRaw_t accData;

int16 Delta_X=0;
int16 Delta_Y=0;
int16 Delta_Z=0;
u8_t old_position=0x00;
u8_t position=0x00;


double Z_Init_Angle = 0;
double Y_Init_Angle = 0;
double X_Init_Angle = 0;

double Z_Angle = 0;
double Y_Angle = 0;
double X_Angle = 0;

double Z_diff_Angle = 0; 
double Y_diff_Angle = 0;
double X_diff_Angle = 0;

double Z_Prev_Angle = 0;
double Y_Prev_Angle = 0;
double X_Prev_Angle = 0;

float Z_Diff = 0;
float Y_Diff = 0;
float X_Diff = 0;

float Z_gVal = 0;
float Y_gVal = 0;
float X_gVal = 0;

float Z_Init_gVal =0;
float Y_Init_gVal =0;
float X_Init_gVal =0;

float Z_diff_gVal =0;
float Y_diff_gVal =0;
float X_diff_gVal =0;

double X_diff1 = 0;
double Y_diff1 = 0;
double Z_diff1 = 0;

char X_positive = true;
char Y_positive = true;
char Z_positive = true;

double resultx1 = 0; double  resulty1 = 0; double  resultz1 =0;

char RegVal1 = 0;
uint8 Gyro_FIFOStatus = 0;


bool read_WHOAMI(void){
  char data[2];

  //data[0] = 0x8F ;
  //data[1] = 0x00 ;
  //read_accelerometer(0, &data, 2);
  LSM6DSM_ReadReg(0x8F);
 Task_sleep(1000);
  if (receiveBuffer[1] == 0x6A)
    return true;
  else
    return false;
}


void LSM_Wakeup()
{
  LSM6DSM_WriteReg(LSM6DSM_CTRL1_XL,(LSM6DSM_ODR_26HZ | LSM6DSM_FULLSCALE_2)); //free fall requires faster ODR, 12.5 Hz wont be enough  
}

void LSM_Sleep()
{
  LSM6DSM_WriteReg(LSM6DSM_CTRL1_XL,(LSM6DSM_ODR_PD | LSM6DSM_FULLSCALE_2)); //free fall requires faster ODR, 12.5 Hz wont be enough
    
}

void disable_LSM6DSM()
{
     //flush SPI
    // Peripheral function on SCK, MISO and MOSI (P0_3-5)
    // Configure CS as output
}
  
void init_LSM6DSM(void){
 
  
    uint8_t RegVal = 0;
    
    // *** Configure accelerometer ***
    
     //Reboot sequence:
    //1 - Set Gyro in power down
    //LSM6DSM_WriteReg(LSM6DSM_CTRL2_G,0x00); //Gyro powerdown
    //2 - Get Acc in high power mode
    //LSM6DSM_WriteReg(LSM6DSM_CTRL6_C, 0x90);
    //3 - set one to the boot pit
    //LSM6DSM_WriteReg(LSM6DSM_CTRL3_C,0x80); //reboot
    //4 - Wait 15 ms
    //Task_sleep(50000);
    
    //enable_Filtering();
    
    LSM6DSM_WriteReg(LSM6DSM_CTRL1_XL,(LSM6DSM_ODR_12_5HZ | LSM6DSM_FULLSCALE_2)); //free fall requires faster ODR, 12.5 Hz wont be enough
    
    //Set mode to Low power
    //we are reading the current value as to not overwrite the other bits. dont know the default value
    RegVal = LSM6DSM_ReadReg(LSM6DSM_CTRL6_C);
    LSM6DSM_WriteReg(LSM6DSM_CTRL6_C, (0x10 | 0x00));     //disable high power mode, set XL user offset bit to 1
    LSM6DSM_WriteReg(LSM6DSM_CTRL2_G, 0x00);     // disable Gyro
   
    //Disable FIFO
    LSM6DSM_WriteReg(LSM6DSM_FIFO_CTRL3, 0x00); //Gryo not in FIFO, Acc not in FIFO
    LSM6DSM_WriteReg(LSM6DSM_FIFO_CTRL5, 0x00); //FIFO ODR is 0. FIFO disabled
    
    LSM6DSM_WriteReg(LSM6DSM_CTRL3_C, 0x04);
    
    //Disable DEN for Acc
    LSM6DSM_WriteReg(LSM6DSM_CTRL4_C, 0x00);
    
    //setup FIFO Interrupt on INT1
   LSM6DSM_WriteReg(LSM6DSM_DRDY_PULSE_CFG, 0x00); //pulse the interrupt
    LSM6DSM_WriteReg(LSM6DSM_INT1_CTRL,0x00); //Fifo threshold interrupt on Int_1
    
    //init_Gryo_FIFO();
 
    // Wait 2ms for accelerometer to power up and settle
    Task_sleep(5000);

}


void calibrate_LSM6DSM()
{

  //XL user offset bit (CTRL6_C register) already set to 1 in initialization 
  //read data
  
  LSM6DSM_GetAccAxesRaw( &accData);
  //WAIT_100ms();
  //read again
  Task_sleep(100000);
  LSM6DSM_GetAccAxesRaw( &accData);
  
 
  
  X_gVal = CalcGValue(accData.AXIS_X);
  Y_gVal = CalcGValue(accData.AXIS_Y);
  Z_gVal = CalcGValue(accData.AXIS_Z);
  Z_Init_gVal = Z_gVal ;
  Y_Init_gVal = Y_gVal;
  X_Init_gVal = X_gVal;
  X_Init_Angle = CalcTiltAngle(X_gVal, Y_gVal, Z_gVal);
  Y_Init_Angle = CalcTiltAngle(Y_gVal, X_gVal, Z_gVal);
  Z_Init_Angle = CalcTiltAngle(Z_gVal, X_gVal, Y_gVal);
 
  
  //UpperTiltThreshold = SettableTiltThreshold + TiltThreshold;
  //LowerTiltThreshold = SettableTiltThreshold - TiltThreshold;
  /*
  //if less than 2 g, add a negative value
  if (accData.AXIS_X < 0x7FFF)
  {
    //this will give us a value from 0 to 127
   // temp_X16 = (float)127 + ((float) accData.AXIS_X * (float)LSM_offset_res);
  }
  else
  {
    //this will give us a value from 0 to -127
  //  temp_X16 = ((float)65535.000 - (float)accData.AXIS_X) * (float)LSM_offset_res;
      temp_X16 = (accData.AXIS_X - 0x7FFF) ;
      
      temp_X32 = (double)temp_X16 / (double)0x10D; // 2g/32767 * (2^-6) = 0.003906
  }
  TX_Data[9] = temp_X32;// accData.AXIS_X >>8;
  TX_Data[10] = temp_X16 >>8 ; //accData.AXIS_X;
  
  //convert to a 8 bit value
  temp_X = (char)temp_X16;

  //write the X offset register

  LSM6DSM_WriteReg(LSM6DSM_X_OFS_USR, 80);

  */
  
}

void init_Gryo_FIFO(void)
{
  
  //setup FIFO_CTRL1 - fifo level - 188 bytes (@ 12.5 Hhz, we can hold upto 5 seconds of data
  LSM6DSM_WriteReg(LSM6DSM_FIFO_CTRL1, LSM6DSM_FIFO_THRESHOLD); //12.5 x 3 x 5seconds
  LSM6DSM_WriteReg(LSM6DSM_FIFO_CTRL2, 0);

  //no fifo decimation for Gyro. Acceleration readings will not be stored in Fifo
  LSM6DSM_WriteReg(LSM6DSM_FIFO_CTRL3, (LSM6DSM_GYRO_FIFO_NO_DEC | LSM6DSM_ACC_FIFO_NO_DEC));

  LSM6DSM_WriteReg(LSM6DSM_FIFO_CTRL4, 0x80);

  //Fifo will record continuously at 12.5 Hz
  LSM6DSM_WriteReg(LSM6DSM_FIFO_CTRL5, (FIFO_MODE_CONTINUOUS | FIFO_ODR_12_5HZ));
   
  //Block Data Update - output registers not updated until MSB and LSB have been read
  //Auto increment register address
  LSM6DSM_WriteReg(LSM6DSM_CTRL3_C, (0x40 | 0x04));  
  
  //StartMotionCalibration(); //get home gyro position
}

u8_t LSM6DSM_ReadReg(u8_t reg){

      char data[2];

  data[0] = (reg | 0x80) ;
  data[1] = 0x00 ;
    return(read_accelerometer(reg, &data, 2));
}

u8_t LSM6DSM_WriteReg(u8_t reg, u8_t val){
    char dataW[2];
    dataW[0] = reg;
    dataW[1] = val;
    write_accelerometer(reg,  &dataW, 2);
    return 1;
}

//read a value from Acc
unsigned char read_accelerometer(unsigned char reg, void *dataByte, unsigned char numOfBytes)
{
  //unsigned char read = 0x00;
   // unsigned char data = 0x80 | reg;
  // char *dataByte0;
  // dataByte0 = dataByte;
  // dataByte0[0] |= 0x80;  //set the read bit
  // memcpy(dataByte, dataByte0,1);
  
  
  PIN_setOutputValue(pinCSHandle, Board_SPI0_CSN, 0);    //set CS

  //transmitBuffer = &dataByte;
  spiTransaction.count = numOfBytes;
  spiTransaction.txBuf = dataByte;
  spiTransaction.rxBuf = receiveBuffer;
  transferOK = SPI_transfer(AccSPI, &spiTransaction);
  if (!transferOK) 
  {
  }
  PIN_setOutputValue(pinCSHandle, Board_SPI0_CSN, 1);    //release CS
  
  return receiveBuffer[1];
}


//Description: This function will write to the accelerometer
void write_accelerometer(unsigned char reg, void *dataByte, unsigned char numOfBytes)
{
  
  //unsigned char dataByte[0] = 0x00 | reg;

  PIN_setOutputValue(pinCSHandle, Board_SPI0_CSN, 0);    //set CS
  
  //transmitBuffer = dataByte;
  spiTransaction.count = numOfBytes;
  spiTransaction.txBuf = dataByte;
  spiTransaction.rxBuf = 0;
  transferOK = SPI_transfer(AccSPI, &spiTransaction);
  if (!transferOK) 
  {
  }
 PIN_setOutputValue(pinCSHandle, Board_SPI0_CSN, 1);    //release CS

}

//Read all 3 axis from the ACC
void Read_Accelerometer_3Axis(void)
{
   //Get initial acc readings
    LIS3DH_GetAccAxesRaw( &accData);
    
      Delta_X= abs((accData.AXIS_X -AXIS_X_Previous));
      Delta_Y= abs((accData.AXIS_Y -AXIS_Y_Previous));
      Delta_Z= abs((accData.AXIS_Z -AXIS_Z_Previous));
  
    
    
   AXIS_X_Previous= (accData.AXIS_X );
   AXIS_Y_Previous= (accData.AXIS_Y);
   AXIS_Z_Previous= (accData.AXIS_Z);
}
/**************************************************************************//**
* @fn      Acc_Moving
* Written By:  Aleksandar Milanovic
 * Date:        OCT 5th 2015
* @brief    6D of motion check 
*
* @param    none
* @param    1 if moving  previous position not same as current 
******************************************************************************/
uint8_t Acc_Moving(void)
{
  uint8_t ret;
  
     //get 6D Position
    //    // Enable Interrupt Mode
/*
        LIS3DH_SetIntMode(LIS3DH_INT_MODE_6D_POSITION);
        LIS3DH_Get6DPosition(&position);
        if((old_position!=position)) ret=1;
        else ret =0;
        old_position = position;
*/
  // initialize Accelerometer and current position 
 //init_accelerometer();
         //Get initial acc readings
      LSM6DSM_GetAccAxesRaw( &accData);
      if (accData.AXIS_X >=0) //<=32767)
      {
        X_positive = 49;
      }
      else
      {
        X_positive = 48;
      }
      if (accData.AXIS_Y >=0) //<=32767)
      {
        Y_positive = 49;
      }
      else
      {
        Y_positive = 48;
      }
      
      if (accData.AXIS_Z >=0) //<=32767)
      {
        Z_positive = 49;
      }
      else
      {
        Z_positive = 48;
      }
      
      
      TX_Data[1] = X_positive;
      TX_Data[3] = Y_positive;
      TX_Data[5] = Z_positive;
      
      Delta_X= abs((accData.AXIS_X -AXIS_X_Previous));
      Delta_Y= abs((accData.AXIS_Y -AXIS_Y_Previous));
      Delta_Z= abs((accData.AXIS_Z -AXIS_Z_Previous));
     if( Delta_X < 600 && Delta_Y <600  && Delta_Z < 600) ret = FALSE;
     else ret =TRUE;
     
    
    
   AXIS_X_Previous= (accData.AXIS_X );
   AXIS_Y_Previous= (accData.AXIS_Y);
   AXIS_Z_Previous= (accData.AXIS_Z);
   
   // LIS3DH_SetMode( LIS3DH_POWER_DOWN );    // POWER DOWN acc
   
//  P0SEL = 0; // Configure Port 0 as GPIO
// P0DIR = 0x00; // FC Port 0 pins P0.0 and P0.1 as input (buttons),
//  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons) 

     
        
        return ret;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetAccAxesRaw
* Written By     : Noaman Makki
* Date           : Wednesday 12th August 2015
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
u8_t LSM6DSM_GetAccAxesRaw(AxesRaw_t* buff) {
  short int value;
  u8_t valueL;
  u8_t valueH;
  uint8_t Stat=0;
  
  
  //if there is no acceleration to read, then return 0
  Stat = LSM6DSM_ReadReg(LSM6DSM_STATUS_REG);
  if (( Stat & 0x01) ==0 )
     return 0;  
  
  //Task_sleep(2000); 
  valueL = LSM6DSM_ReadReg(LSM6DSM_OUTX_L_XL);
  valueH = LSM6DSM_ReadReg(LSM6DSM_OUTX_H_XL);

  value=0;
  value |= valueH;
  value<<=8;
  value|=valueL;

  buff->AXIS_X = value;
  
  valueL = LSM6DSM_ReadReg(LSM6DSM_OUTY_L_XL);
  valueH = LSM6DSM_ReadReg(LSM6DSM_OUTY_H_XL);

  value=0;
  value |= valueH;
  value<<=8;
  value|=valueL;

  buff->AXIS_Y = value;
  
  valueL = LSM6DSM_ReadReg(LSM6DSM_OUTZ_L_XL);
  valueH = LSM6DSM_ReadReg(LSM6DSM_OUTZ_H_XL);

  value=0;
  value |= valueH;
  value<<=8;
  value|=valueL;

  buff->AXIS_Z = value;


  return 1;
}


//converts acc Data into angles
u8_t LSM_check_tilt()
{
  LSM6DSM_GetAccAxesRaw( &accData);
  
  //convert into positive G
  //any tilt is bad tilt

  //0g - 0, 2g = 32767.
  //at 45 degree we should have 0.707g = 11538
  //at 35 degree we should have 0.573g = 9397

 // X_Val = accData.AXIS_X;
 // Y_Val = accData.AXIS_Y;
 // Z_Val = accData.AXIS_Z;
  
  X_gVal = CalcGValue(accData.AXIS_X);
  Y_gVal = CalcGValue(accData.AXIS_Y);
  Z_gVal = CalcGValue(accData.AXIS_Z);

  
  if (!checkForTilt)
  {
    X_diff_gVal = X_gVal - X_Init_gVal;
    Y_diff_gVal = Y_gVal - Y_Init_gVal;
    Z_diff_gVal = Z_gVal - Z_Init_gVal;
    
    if (X_diff_gVal <0.0) {X_diff_gVal = (double)(-1) * X_diff_gVal;}
    if (Y_diff_gVal <0.0) {Y_diff_gVal = (double)(-1) * Y_diff_gVal;}
    if (Z_diff_gVal <0.0) {Z_diff_gVal = (double)(-1) * Z_diff_gVal;}

    
    if ((X_diff_gVal > SettableAccThreshold) || (Y_diff_gVal > SettableAccThreshold) || (Z_diff_gVal > SettableAccThreshold) )
    {
      X_Init_gVal = X_gVal;
      Y_Init_gVal = Y_gVal;
      Z_Init_gVal = Z_gVal;
      return 0x02;   //yes there is acc
    }
  
  }
  else
  {
  
    X_Angle = CalcTiltAngle(X_gVal, Y_gVal, Z_gVal);
    Y_Angle = CalcTiltAngle(Y_gVal, X_gVal, Z_gVal);
    Z_Angle = CalcTiltAngle(Z_gVal, Y_gVal, X_gVal);
    
    X_diff_Angle = X_Angle - X_Init_Angle;
    Y_diff_Angle = Y_Angle - Y_Init_Angle;
    Z_diff_Angle = Z_Angle - Z_Init_Angle;
    
    if (X_diff_Angle<0.0) { X_diff_Angle = (double)(-1) * X_diff_Angle ;}
    if (Y_diff_Angle<0.0) { Y_diff_Angle = (double)(-1) * Y_diff_Angle ;}
    if (Z_diff_Angle<0.0) { Z_diff_Angle = (double)(-1) * Z_diff_Angle ;}
    
    if (X_Angle<0.0) { X_Angle = (double)(-1) * X_Angle ;}
    if (Y_Angle<0.0) { Y_Angle = (double)(-1) * Y_Angle ;}
    if (Z_Angle<0.0) { Z_Angle = (double)(-1) * Z_Angle ;}
    
    if (X_Prev_Angle<0.0) { X_Prev_Angle = (double)(-1) * X_Prev_Angle ;}
    if (Y_Prev_Angle<0.0) { Y_Prev_Angle = (double)(-1) * Y_Prev_Angle ;}
    if (Z_Prev_Angle<0.0) { Z_Prev_Angle = (double)(-1) * Z_Prev_Angle ;}
    
    X_diff1 = X_Angle - X_Prev_Angle;
    Y_diff1 = Y_Angle - Y_Prev_Angle;
    Z_diff1 = Z_Angle - Z_Prev_Angle;
    
    if ((X_diff1 >TiltThreshold) || (Y_diff1 >TiltThreshold)  || (Z_diff1 >TiltThreshold) )
    {

    //if any combination of x, y or x+y is greater than 35 degrees
    if ((X_diff_Angle > SettableTiltThreshold) || (Y_diff_Angle > SettableTiltThreshold) || (Z_diff_Angle > SettableTiltThreshold) )
    {
      
      X_Prev_Angle = X_Angle;
      Y_Prev_Angle = Y_Angle;
      Z_Prev_Angle = Z_Angle;
        return 0x01;   //yes there is tilt
    }
    }
  }

  
  return 0;
}

void configure_LSM_freefall()
{
  LSM6DSM_WriteReg(LSM6DSM_CTRL1_XL,(LSM6DSM_ODR_52HZ | LSM6DSM_FULLSCALE_2)); 
  
 
  
  LSM6DSM_WriteReg(LSM6DSM_TAP_CFG1, 0x81);      //enable free fall interrupt and set latch mode

  LSM6DSM_WriteReg(LSM6DSM_WAKE_UP_DUR, 0x00); //Set event duration (FF_DUR5 bit)
    
  LSM6DSM_WriteReg(LSM6DSM_FREE_FALL, 0x33);    //Threshold [0:2] = 010 = 250 mg, or 011 = 312 mg; Duration [7:3] = 010 = 2 (2/52 Hz) = 38.4 ms
      
  LSM6DSM_WriteReg(LSM6DSM_MD1_CFG, 0x10);  //free fall interrupt on INT 1 pin
}

void configure_LSM_sigmotion()
{
  LSM6DSM_WriteReg(LSM6DSM_FUNC_CFG_ACCESS, 0x80); //Enable access to Embedded functions registers
  LSM6DSM_WriteReg(LSM6DSM_SM_THS, 0x03);      //set significant motion threshold
  LSM6DSM_WriteReg(LSM6DSM_FUNC_CFG_ACCESS, 0x00); //Disable access to Embedded functions registers
  LSM6DSM_WriteReg(LSM6DSM_CTRL10_C, 0x05);     //Enable significant motion detection
  LSM6DSM_WriteReg(LSM6DSM_INT1_CTRL, 0x40);    //Significant motion interrupt on INT1
  
  
}

double CalcTiltAngle(float my_gVal1, float my_gVal2, float my_gVal3)
{
  double sqr1, sqr2, my_Diff1, my_Diff2, my_Diff3, myResult;
  double myAngle;
  
  my_Diff1 = my_gVal1;
  
  my_Diff2 = my_gVal2;
  
  my_Diff3 = my_gVal3;
  
  sqr1 = my_Diff2 * my_Diff2;
  sqr2 = my_Diff3 * my_Diff3;
  
  myResult = sqrt(sqr1 + sqr2);
  myResult = my_Diff1/myResult;
  
  myAngle = atan(myResult);
  myAngle = (myAngle/3.1416) * 180.00;
  
  return (myAngle);
  /*
  float X2, Y2, Z2;
  resultx1 = 0;
  resulty1 = 0;
  resultz1 = 0;
  X_Diff = X_gVal;//-X_gInit;
  Y_Diff = Y_gVal;//- Y_gInit;
  Z_Diff = Z_gVal;//- Z_gInit;
  
  X2 = X_Diff * X_Diff;
  Y2 = Y_Diff * Y_Diff;
  Z2 = Z_Diff * Z_Diff;
  
  resultx1 =sqrt(Y2+Z2);
  resultx1 =  X_Diff/resultx1; 
  X_Angle = atan(resultx1);
  X_Angle = (X_Angle/3.1416) * 180.00;

  resulty1 = sqrt(X2+Z2);
  resulty1 = Y_Diff/ resulty1;
  Y_Angle = atan(resulty1);  

  Y_Angle = (Y_Angle/3.1416) * 180.00;
  
  resultz1 = sqrt(Y2+X2);
  resultz1 = Z_Diff/ resultz1 ;
  Z_Angle = atan(resultz1);
  Z_Angle = (Z_Angle/3.1416) * 180.00;
  
  asm("NOP");

  */
  
}

/*
signed scale goes from 0 to 127, -128 to -1
signed scale goes from 0 to 32767, -32767 to -1

0-32767 = 0 to 2g
32768-65535 = -2g to 0 (larger decimal value is smaller g value

32768-65535-0-32767 = -2g - 0 - 0 - 2g
0 - 32767 = 0 to 2g
-32767 - 0 = 0 to -2g
*/
float CalcGValue(float myVal)
{
   return (myVal/16383.00);
  /*
  if (X_Val>0)
  {
    X_gVal = (float)X_Val/16383;
  }
  else
  {
    X_gVal = (float)X_Val/16383;
    //X_gVal = (float)(X_Val-32767)/16383;
  }
  
  if (Y_Val>0)
  {
    Y_gVal = (float)Y_Val/16383;
  }
  else
  {
    Y_gVal = (float)Y_Val/16383;
    //Y_gVal = (float)(Y_Val-32767)/16383;
  }
  
  if (Z_Val>0)
  {
    Z_gVal = (float)Z_Val/16383;
  }
  else
  {
    Z_gVal = (float)Z_Val/16383;
    //Z_gVal = (float)(Z_Val - 32767)/16383;
  }
  */
}


void LSM6DSM_process_Interrupt()
{

  unsigned char tempr = 0;
  //clear the LSM interrupt by reading WAKE_UP_SRC

  tempr = LSM6DSM_ReadReg(LSM6DSM_WAKE_UP_SRC);
  tempr &= 0x20;
  
    //if it is indeed Free fall interupt WAKE_UP_SRC bit 5
  if ((tempr >>5) ==1)
  {
      //free fall
    
  }
  
  tempr = 0;
  tempr = LSM6DSM_ReadReg(LSM6DSM_FUNC_SRC1);
  
  if(tempr & BV(6))
  {
      //significant motion
  }
    
  //blink the LED 

}

/*
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
  unsigned int m=0;
  
 // if (P1IFG & BV(7)) 
 // {
 //   LSM6DSM_process_Interrupt();
 // }
  

  //blink the LED
  
    //Clear the CPU interrupt flag for Port_1
  //  PxIFG has to be cleared before PxIF

  LSM6DSM_ReadReg(LSM6DSM_WAKE_UP_SRC);
  LSM6DSM_ReadReg(LSM6DSM_FUNC_SRC1);
   //clear the P1_7 interrupt flag
   //clear the MCU interrupt flag
}

*/