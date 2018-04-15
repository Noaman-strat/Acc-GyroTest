
#ifndef MYSPI_H
#define MYSPI_H

#ifdef __cplusplus
extern "C" {
#endif


/*********************************************************************
 * INCLUDES
 */
  
#include "PlaneRemote.h"
#include <./ti/drivers/SPI.h>
#include "LSM6DSM.h"
/*********************************************************************
*  EXTERNAL VARIABLES
*/
  
#define       MSGSIZE       20
extern uint8_t         transmitBuffer[MSGSIZE];
extern uint8_t         receiveBuffer[MSGSIZE];
extern SPI_Transaction spiTransaction;
extern bool            transferOK;

extern SPI_Handle AccSPI;

extern char IAmCalled;
/*********************************************************************
 * CONSTANTS
 */


extern bool Gyro_Started;

extern bool Motion_Calibration_Running;

extern char Gyro_Rotation_Axis; //0 for X, 1 for Y, 2 for Z
#define         X_AXIS  0
#define         Y_AXIS  1
#define         Z_AXIS  2



//extern float Angle_X[50];
//extern float Angle_Y[50];
//extern float Angle_Z[50];

//extern float AccAngle_X[50];
//extern float AccAngle_Y[50];
//extern float AccAngle_Z[50];

extern double AngleX1, AngleY1, AngleZ1;

extern unsigned int ACC_EVENT_PERIOD;

extern uint8_t CurrentOrientation;
extern uint8_t PreviousOrientation;

extern AxesRaw_t* Gyro_Home_Position;

extern char HomePositionSet;    //home position must be set by holding Button 1 for 5 seconds

//defines device mounting orientation
enum devOrientation
{
  Orientation_X, //0
  Orientation_Y,     //1
  Orientation_Z,  //2
  NotIdentified
};
extern enum devOrientation DeviceOrientation;
extern char CalibrationCounter;

//Angle calcualation from Acc + Gyro data
extern short CalculatedBeaconAngle;

#define         GyroK           0.98
#define         AccK            0.02

//Gyro provides the rate of change of angle (d-theta/d-time).
//to convert this rate of change into angle change, we need to divide by time between readings
#define         TimeDuration    0.08    //in seconds - time between success gyro readings

//sensitivity is specified in mdps/LSB. or 1/1000 dps/LSB
#define         GyroSensitivity_125     4.375   
#define         GyroSensitivity_500     17.5   

#define         GyroDivisor125          18.2857        //0.00047878      //dt / sensitivity = 0.08 / (4.375/1000)
#define         GyroDivisor500          4.5714       //0.0045714

#define         AccDivisor_2g           0.000061        //0.061 / 1000 (mg / 1000 = g)
#define         AccDivisor_4g           0.000122        //0.122 / 1000        
//at no change in angle, gyro still outputs a non-zero value
//we need to subtract this value from the reading to get accurate results
extern short Gyro_Offset_X;
extern short Gyro_Offset_Y;
extern short Gyro_Offset_Z;

//

extern bool EventRecord;
/*********************************************************************
 * TYPEDEFS
 */
typedef void (*LSMIntPinCB_t)(uint8_t INTPIN);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */


bool mySPI_Config(void);
void mySPI_send(char ds);
static void mySPI_Callback (SPI_Handle, SPI_Transaction *spiTransaction);

void Process_Int(void);

void StartMotionCalibration(void);
void EndMotionCalibration(void);

void CalculateAngle(char datasets);

void GetHomePositionOrientation(void);

void Process_Int(void);

void Check_for_Quadrant_Change(void);


//bool read_WHOAMI(void);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* UART_H */