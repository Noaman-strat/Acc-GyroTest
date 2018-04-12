/* 
 * File:   LSM6DSM.h
 * Author: Noaman Makki
 *
 * Created on August 12, 2015, 2:40 PM
 */

#include "hal_types.h"
#include "math.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
   
typedef uint8 u8_t;
typedef int8 i8_t;
typedef uint16 u16_t;
typedef int16 i16_t;

/******************************************************************************
 * MACROS
 */
// Wait 1 [ms]
//#define WAIT_1MS()      {for(unsigned short i=0;i<32000;i++)asm("NOP"); }

// Wait t [ms]
#define WAIT_MS(t)                      \
    do{                                 \
        for(uint8 i = 0; i<t; i++)      \
            WAIT_1ms();                 \
    }while(0)




#define BIT(x) ( (x) )
// Operating Mode
#define  LIS3DH_POWER_DOWN      0x00
#define  LIS3DH_LOW_POWER 	0x01
#define  LIS3DH_NORMAL		0x02

// LIS3DH FullScale

#define  LIS3DH_FULLSCALE_2     0x00
#define  LIS3DH_FULLSCALE_4     0x01
#define  LIS3DH_FULLSCALE_8     0x02
#define  LIS3DH_FULLSCALE_16    0x03

#define  LIS3DH_BLE_LSB		0x00
#define  LIS3DH_BLE_MSB		0x01


// State Temperature
#define   MEMS_ENABLE		0x01
#define   MEMS_DISABLE		0x00

#define MEMS_SET                0x01
#define MEMS_RESET              0x00

// LIS3DH Axis Enable
#define   LIS3DH_X_ENABLE       0x01
#define   LIS3DH_X_DISABLE      0x00
#define   LIS3DH_Y_ENABLE       0x02
#define   LIS3DH_Y_DISABLE      0x00
#define   LIS3DH_Z_ENABLE       0x04
#define   LIS3DH_Z_DISABLE      0x00
// Click Response
#define LIS3DH_DCLICK_Z_P       0x24
#define LIS3DH_DCLICK_Z_N       0x2C
#define LIS3DH_SCLICK_Z_P       0x14
#define LIS3DH_SCLICK_Z_N       0x1C
#define LIS3DH_DCLICK_Y_P       0x22
#define LIS3DH_DCLICK_Y_N       0x2A
#define LIS3DH_SCLICK_Y_P       0x12
#define LIS3DH_SCLICK_Y_N	0x1A
#define LIS3DH_DCLICK_X_P       0x21
#define LIS3DH_DCLICK_X_N       0x29
#define LIS3DH_SCLICK_X_P       0x11
#define LIS3DH_SCLICK_X_N       0x19
#define LIS3DH_NO_CLICK         0x00


#define  LIS3DH_INT1_6D_4D_DISABLE     0x00
#define  LIS3DH_INT1_6D_ENABLE         0x01
#define  LIS3DH_INT1_4D_ENABLE         0x02


#define  LIS3DH_UP_SX                  0x44
#define  LIS3DH_UP_DX                  0x42
#define  LIS3DH_DW_SX                  0x41
#define  LIS3DH_DW_DX                  0x48
#define  LIS3DH_TOP                    0x60
#define  LIS3DH_BOTTOM                 0x50

// Int Mode

#define  LIS3DH_INT_MODE_OR             0x00
#define  LIS3DH_INT_MODE_6D_MOVEMENT    0x01
#define  LIS3DH_INT_MODE_AND            0x02
#define  LIS3DH_INT_MODE_6D_POSITION    0x03
// Fifo Mode

#define  LIS3DH_FIFO_BYPASS_MODE        0x00
#define  LIS3DH_FIFO_MODE               0x01
#define  LIS3DH_FIFO_STREAM_MODE        0x02
#define  LIS3DH_FIFO_TRIGGER_MODE       0x03
#define  LIS3DH_FIFO_DISABLE            0x04

// Int trigger
#define   LIS3DH_TRIG_INT1              0x00
#define   LIS3DH_TRIG_INT2 		0x01
// HPF Mode
#define  LIS3DH_HPM_NORMAL_MODE_RES   0x00
#define  LIS3DH_HPM_REF_SIGNAL        0x01
#define  LIS3DH_HPM_NORMAL_MODE       0x02
#define  LIS3DH_HPM_AUTORESET_INT     0x03

// SPI Mode

#define  LIS3DH_SPI_4_WIRE            0x00
#define  LIS3DH_SPI_3_WIRE            0x01

// HPF CutOff
#define  LIS3DH_HPFCF_0               0x00
#define  LIS3DH_HPFCF_1               0x01
#define  LIS3DH_HPFCF_2               0x02
#define  LIS3DH_HPFCF_3               0x03


// LIS3DH ODR
#define   LIS3DH_ODR_1HZ	0x01
#define   LIS3DH_ODR_10HZ       0x02
#define   LIS3DH_ODR_25HZ	0x03
#define   LIS3DH_ODR_50HZ	0x04
#define   LIS3DH_ODR_100HZ	0x05
#define   LIS3DH_ODR_200HZ	0x06
#define   LIS3DH_ODR_400HZ	0x07
#define   LIS3DH_ODR_1620Hz_LP	0x08
#define   LIS3DH_ODR_1344Hz_NP_5367HZ_LP  0x09

//Register Definition
#define LIS3DH_WHO_AM_I				0x0F 

// CONTROL REGISTER 1
#define LIS3DH_CTRL_REG1				0x20
#define LIS3DH_ODR_BIT				        BIT(4)
#define LIS3DH_LPEN					BIT(3)
#define LIS3DH_ZEN					BIT(2)
#define LIS3DH_YEN					BIT(1)
#define LIS3DH_XEN					BIT(0)

//CONTROL REGISTER 2
#define LIS3DH_CTRL_REG2				0x21
#define LIS3DH_HPM     				BIT(6)
#define LIS3DH_HPCF					BIT(4)
#define LIS3DH_FDS					BIT(3)
#define LIS3DH_HPCLICK					BIT(2)
#define LIS3DH_HPIS2					BIT(1)
#define LIS3DH_HPIS1					BIT(0)

//CONTROL REGISTER 3
#define LIS3DH_CTRL_REG3				0x22
#define LIS3DH_I1_CLICK				BIT(7)
#define LIS3DH_I1_AOI1					BIT(6)
#define LIS3DH_I1_AOI2				        BIT(5)
#define LIS3DH_I1_DRDY1				BIT(4)
#define LIS3DH_I1_DRDY2				BIT(3)
#define LIS3DH_I1_WTM					BIT(2)
#define LIS3DH_I1_ORUN					BIT(1)

//CONTROL REGISTER 6
#define LIS3DH_CTRL_REG6				0x25
#define LIS3DH_I2_CLICK				BIT(7)
#define LIS3DH_I2_INT1					BIT(6)
#define LIS3DH_I2_BOOT         			BIT(4)
#define LIS3DH_H_LACTIVE				BIT(1)

//TEMPERATURE CONFIG REGISTER
#define LIS3DH_TEMP_CFG_REG				0x1F
#define LIS3DH_ADC_PD				        BIT(7)
#define LIS3DH_TEMP_EN					BIT(6)

//CONTROL REGISTER 4
#define LIS3DH_CTRL_REG4				0x23
#define LIS3DH_BDU					BIT(7)
#define LIS3DH_BLE					BIT(6)
#define LIS3DH_FS					BIT(4)
#define LIS3DH_HR					BIT(3)
#define LIS3DH_ST       				BIT(1)
#define LIS3DH_SIM					BIT(0)

//CONTROL REGISTER 5
#define LIS3DH_CTRL_REG5				0x24
#define LIS3DH_BOOT                                    BIT(7)
#define LIS3DH_FIFO_EN                                 BIT(6)
#define LIS3DH_LIR_INT1                                BIT(3)
#define LIS3DH_D4D_INT1                                BIT(2)

//REFERENCE/DATA_CAPTURE
#define LIS3DH_REFERENCE_REG		                0x26
#define LIS3DH_REF		                	BIT(0)

//STATUS_REG_AXIES
#define LIS3DH_STATUS_REG				0x27
#define LIS3DH_ZYXOR                                   BIT(7)
#define LIS3DH_ZOR                                     BIT(6)
#define LIS3DH_YOR                                     BIT(5)
#define LIS3DH_XOR                                     BIT(4)
#define LIS3DH_ZYXDA                                   BIT(3)
#define LIS3DH_ZDA                                     BIT(2)
#define LIS3DH_YDA                                     BIT(1)
#define LIS3DH_XDA                                     BIT(0)

//STATUS_REG_AUX
#define LIS3DH_STATUS_AUX				0x07

//INTERRUPT 1 CONFIGURATION
#define LIS3DH_INT1_CFG				0x30
#define LIS3DH_ANDOR                                   BIT(7)
#define LIS3DH_INT_6D                                  BIT(6)
#define LIS3DH_ZHIE                                    BIT(5)
#define LIS3DH_ZLIE                                    BIT(4)
#define LIS3DH_YHIE                                    BIT(3)
#define LIS3DH_YLIE                                    BIT(2)
#define LIS3DH_XHIE                                    BIT(1)
#define LIS3DH_XLIE                                    BIT(0)

//FIFO CONTROL REGISTER
#define LIS3DH_FIFO_CTRL_REG                           0x2E
#define LIS3DH_FM                                      BIT(6)
#define LIS3DH_TR                                      BIT(5)
#define LIS3DH_FTH                                     BIT(0)

//CONTROL REG3 bit mask
#define LIS3DH_CLICK_ON_PIN_INT1_ENABLE                0x80
#define LIS3DH_CLICK_ON_PIN_INT1_DISABLE               0x00
#define LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE              0x40
#define LIS3DH_I1_INT1_ON_PIN_INT1_DISABLE             0x00
#define LIS3DH_I1_INT2_ON_PIN_INT1_ENABLE              0x20
#define LIS3DH_I1_INT2_ON_PIN_INT1_DISABLE             0x00
#define LIS3DH_I1_DRDY1_ON_INT1_ENABLE                 0x10
#define LIS3DH_I1_DRDY1_ON_INT1_DISABLE                0x00
#define LIS3DH_I1_DRDY2_ON_INT1_ENABLE                 0x08
#define LIS3DH_I1_DRDY2_ON_INT1_DISABLE                0x00
#define LIS3DH_WTM_ON_INT1_ENABLE                      0x04
#define LIS3DH_WTM_ON_INT1_DISABLE                     0x00
#define LIS3DH_INT1_OVERRUN_ENABLE                     0x02
#define LIS3DH_INT1_OVERRUN_DISABLE                    0x00

//CONTROL REG6 bit mask
#define LIS3DH_CLICK_ON_PIN_INT2_ENABLE                0x80
#define LIS3DH_CLICK_ON_PIN_INT2_DISABLE               0x00
#define LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE              0x40
#define LIS3DH_I2_INT1_ON_PIN_INT2_DISABLE             0x00
#define LIS3DH_I2_INT2_ON_PIN_INT2_ENABLE              0x20
#define LIS3DH_I2_INT2_ON_PIN_INT2_DISABLE             0x00
#define LIS3DH_I2_BOOT_ON_INT2_ENABLE                  0x10
#define LIS3DH_I2_BOOT_ON_INT2_DISABLE                 0x00
#define LIS3DH_INT_ACTIVE_HIGH                         0x00
#define LIS3DH_INT_ACTIVE_LOW                          0x02

//INT1_CFG bit mask
#define LIS3DH_INT1_AND                                0x80
#define LIS3DH_INT1_OR                                 0x00
#define LIS3DH_INT1_ZHIE_ENABLE                        0x20
#define LIS3DH_INT1_ZHIE_DISABLE                       0x00
#define LIS3DH_INT1_ZLIE_ENABLE                        0x10
#define LIS3DH_INT1_ZLIE_DISABLE                       0x00
#define LIS3DH_INT1_YHIE_ENABLE                        0x08
#define LIS3DH_INT1_YHIE_DISABLE                       0x00
#define LIS3DH_INT1_YLIE_ENABLE                        0x04
#define LIS3DH_INT1_YLIE_DISABLE                       0x00
#define LIS3DH_INT1_XHIE_ENABLE                        0x02
#define LIS3DH_INT1_XHIE_DISABLE                       0x00
#define LIS3DH_INT1_XLIE_ENABLE                        0x01
#define LIS3DH_INT1_XLIE_DISABLE                       0x00

//INT1_SRC bit mask
#define LIS3DH_INT1_SRC_IA                             0x40
#define LIS3DH_INT1_SRC_ZH                             0x20
#define LIS3DH_INT1_SRC_ZL                             0x10
#define LIS3DH_INT1_SRC_YH                             0x08
#define LIS3DH_INT1_SRC_YL                             0x04
#define LIS3DH_INT1_SRC_XH                             0x02
#define LIS3DH_INT1_SRC_XL                             0x01

//INT1 REGISTERS
#define LIS3DH_INT1_THS                                0x32
#define LIS3DH_INT1_DURATION                           0x33

//INTERRUPT 1 SOURCE REGISTER
#define LIS3DH_INT1_SRC				0x31

//FIFO Source Register bit Mask
#define LIS3DH_FIFO_SRC_WTM                            0x80
#define LIS3DH_FIFO_SRC_OVRUN                          0x40
#define LIS3DH_FIFO_SRC_EMPTY                          0x20

//INTERRUPT CLICK REGISTER
#define LIS3DH_CLICK_CFG				0x38
//INTERRUPT CLICK CONFIGURATION bit mask
#define LIS3DH_ZD_ENABLE                               0x20
#define LIS3DH_ZD_DISABLE                              0x00
#define LIS3DH_ZS_ENABLE                               0x10
#define LIS3DH_ZS_DISABLE                              0x00
#define LIS3DH_YD_ENABLE                               0x08
#define LIS3DH_YD_DISABLE                              0x00
#define LIS3DH_YS_ENABLE                               0x04
#define LIS3DH_YS_DISABLE                              0x00
#define LIS3DH_XD_ENABLE                               0x02
#define LIS3DH_XD_DISABLE                              0x00
#define LIS3DH_XS_ENABLE                               0x01
#define LIS3DH_XS_DISABLE                              0x00

//INTERRUPT CLICK SOURCE REGISTER
#define LIS3DH_CLICK_SRC                               0x39
//INTERRUPT CLICK SOURCE REGISTER bit mask
#define LIS3DH_IA                                      0x40
#define LIS3DH_DCLICK                                  0x20
#define LIS3DH_SCLICK                                  0x10
#define LIS3DH_CLICK_SIGN                              0x08
#define LIS3DH_CLICK_Z                                 0x04
#define LIS3DH_CLICK_Y                                 0x02
#define LIS3DH_CLICK_X                                 0x01

//Click-click Register
#define LIS3DH_CLICK_THS                               0x3A
#define LIS3DH_TIME_LIMIT                              0x3B
#define LIS3DH_TIME_LATENCY                            0x3C
#define LIS3DH_TIME_WINDOW                             0x3D

//OUTPUT REGISTER
#define LIS3DH_OUT_X_L					0x28
#define LIS3DH_OUT_X_H					0x29
#define LIS3DH_OUT_Y_L					0x2A
#define LIS3DH_OUT_Y_H					0x2B
#define LIS3DH_OUT_Z_L					0x2C
#define LIS3DH_OUT_Z_H					0x2D

//AUX REGISTER
#define LIS3DH_OUT_1_L					0x08
#define LIS3DH_OUT_1_H					0x09
#define LIS3DH_OUT_2_L					0x0A
#define LIS3DH_OUT_2_H					0x0B
#define LIS3DH_OUT_3_L					0x0C
#define LIS3DH_OUT_3_H					0x0D

//STATUS REGISTER bit mask
#define LIS3DH_STATUS_REG_ZYXOR                        0x80    // 1	:	new data set has over written the previous one
							// 0	:	no overrun has occurred (default)
#define LIS3DH_STATUS_REG_ZOR                          0x40    // 0	:	no overrun has occurred (default)
							// 1	:	new Z-axis data has over written the previous one
#define LIS3DH_STATUS_REG_YOR                          0x20    // 0	:	no overrun has occurred (default)
							// 1	:	new Y-axis data has over written the previous one
#define LIS3DH_STATUS_REG_XOR                          0x10    // 0	:	no overrun has occurred (default)
							// 1	:	new X-axis data has over written the previous one
#define LIS3DH_STATUS_REG_ZYXDA                        0x08    // 0	:	a new set of data is not yet avvious one
                                                        // 1	:	a new set of data is available
#define LIS3DH_STATUS_REG_ZDA                          0x04    // 0	:	a new data for the Z-Axis is not availvious one
                                                        // 1	:	a new data for the Z-Axis is available
#define LIS3DH_STATUS_REG_YDA                          0x02    // 0	:	a new data for the Y-Axis is not available
                                                        // 1	:	a new data for the Y-Axis is available
#define LIS3DH_STATUS_REG_XDA                          0x01    // 0	:	a new data for the X-Axis is not available

#define LIS3DH_DATAREADY_BIT                           LIS3DH_STATUS_REG_ZYXDA


//STATUS AUX REGISTER bit mask
#define LIS3DH_STATUS_AUX_321OR                         0x80
#define LIS3DH_STATUS_AUX_3OR                           0x40
#define LIS3DH_STATUS_AUX_2OR                           0x20
#define LIS3DH_STATUS_AUX_1OR                           0x10
#define LIS3DH_STATUS_AUX_321DA                         0x08
#define LIS3DH_STATUS_AUX_3DA                           0x04
#define LIS3DH_STATUS_AUX_2DA                           0x02
#define LIS3DH_STATUS_AUX_1DA                           0x01

#define LIS3DH_MEMS_I2C_ADDRESS			        0x33

//FIFO REGISTERS
#define LIS3DH_FIFO_CTRL_REG			        0x2E
#define LIS3DH_FIFO_SRC_REG			        0x2F
      
   /*******************************************************************************
 * Watchdog Timer
 */

// WDCTL (0xC9) - Watchdog Timer Control
  #define WDCTL_CLR                         0xF0
  #define WDCTL_CLR0                        0x10
  #define WDCTL_CLR1                        0x20
  #define WDCTL_CLR2                        0x40
  #define WDCTL_CLR3                        0x80
  #define WDCTL_MODE                        (0x03 << 2)   // Selects mode, bit mask
  #define WDCTL_MODE_IDLE                   (0x00 << 2)   // Idle, when in Timer mode
  #define WDCTL_MODE_WD                     (0x02 << 2)   // Watchdog mode (when in watchdog mode writing to these bits have no effect.)
  #define WDCTL_MODE_TIMER                  (0x03 << 2)   // Timer mode
  #define WDCTL_INT                         (0x03)        // Interval select
  #define WDCTL_INT_1_SEC                   (0x00)
  #define WDCTL_INT_250_MSEC                (0x01)
  #define WDCTL_INT_15_MSEC                 (0x02)
  #define WDCTL_INT_2_MSEC                  (0x03)
   
      
      

      
      
      

typedef struct {
  signed short AXIS_X;
  signed short AXIS_Y;
  signed short AXIS_Z;
} AxesRaw_t;


void Start_WDT(void);
void ClearWDT(void);


/*
 *                  ACCELEROMETER - SPI Write / Read Setup 
 *
 * bit 0: RW bit. When 0, the data DI(7:0) is written into the device. When 1, the data DO(7:0)
 * from the device is read. In latter case, the chip drives SDO at the start of bit 8.
 *
 * bit 1: MS bit. When 0, the address remains unchanged in multiple read/write commands.
 * When 1, the address is auto incremented in multiple read/write commands.
 *
 * bit 2-7: address AD(5:0). This is the address field of the indexed register.
 * bit 8-15: data DI(7:0) (write mode). This is the data that is written into the device (MSb first).
 * bit 8-15: data DO(7:0) (read mode). This is the data that is read from the device (MSb first).
 */

void write_accelerometer(unsigned char reg, void *dataByte, unsigned char numOfBytes);
unsigned char read_accelerometer(unsigned char reg, void *dataByte, unsigned char numOfBytes);
void init_accelerometer(void);
void readSPI(uint8 *read, uint8 write) ; 
void writeSPI(uint8 write);
void Read_Accelerometer_3Axis(void);
uint8 Acc_Moving(void);

uint8 CheckQuadrant();
void LSM_Wakeup_Gyro(void);
void LSM_Sleep_Gyro(void);

void enable_Filtering(void);
char ReadGyroData(void);

bool read_WHOAMI(void);
uint8 read_LSM(void);

u8_t LSM6DSM_ReadReg(u8_t reg);
u8_t LSM6DSM_WriteReg(u8_t reg, u8_t val);
u8_t LSM6DSM_GetAccAxesRaw(AxesRaw_t* buff);
void init_LSM6DSM(void);
void init_Gryo_FIFO(void);
void LSM_Wakeup();
void LSM_Sleep();
void disable_LSM6DSM();
void calibrate_LSM6DSM();
u8_t LSM_check_tilt();
void configure_LSM_freefall();
void LSM6DSM_process_Interrupt();
void configure_LSM_sigmotion();
void configure_LSM_tiltInt();
void LSM_Clear_tiltInt();

double CalcTiltAngle(float my_gVal1, float my_gVal2, float my_gVal3);
float CalcGValue(float myVal);

void LSM6DSM_process_Gyro_Interrupt(void);

extern AxesRaw_t accData;

#define LSM6DSM_FIFO_THRESHOLD          36

#define LSM_offset_res  0.015625

#define  LSM6DSM_BLE_LSB		0x00
#define  LSM6DSM_BLE_MSB		0x01


// State Temperature
#define   MEMS_ENABLE		0x01
#define   MEMS_DISABLE		0x00

#define MEMS_SET                0x01
#define MEMS_RESET              0x00

// LSM6DSM Axis Enable

// Click Response
#define LSM6DSM_DCLICK_Z_P       0x24
#define LSM6DSM_DCLICK_Z_N       0x2C
#define LSM6DSM_SCLICK_Z_P       0x14
#define LSM6DSM_SCLICK_Z_N       0x1C
#define LSM6DSM_DCLICK_Y_P       0x22
#define LSM6DSM_DCLICK_Y_N       0x2A
#define LSM6DSM_SCLICK_Y_P       0x12
#define LSM6DSM_SCLICK_Y_N	0x1A
#define LSM6DSM_DCLICK_X_P       0x21
#define LSM6DSM_DCLICK_X_N       0x29
#define LSM6DSM_SCLICK_X_P       0x11
#define LSM6DSM_SCLICK_X_N       0x19
#define LSM6DSM_NO_CLICK         0x00


#define  LSM6DSM_INT1_6D_4D_DISABLE     0x00
#define  LSM6DSM_INT1_6D_ENABLE         0x01
#define  LSM6DSM_INT1_4D_ENABLE         0x02




// Int Mode

#define  LSM6DSM_INT_MODE_OR             0x00
#define  LSM6DSM_INT_MODE_6D_MOVEMENT    0x01
#define  LSM6DSM_INT_MODE_AND            0x02
#define  LSM6DSM_INT_MODE_6D_POSITION    0x03
// Fifo Mode

#define  LSM6DSM_FIFO_BYPASS_MODE        0x00
#define  LSM6DSM_FIFO_MODE               0x01
#define  LSM6DSM_FIFO_STREAM_MODE        0x02
#define  LSM6DSM_FIFO_TRIGGER_MODE       0x03
#define  LSM6DSM_FIFO_DISABLE            0x04

// Int trigger
#define   LSM6DSM_TRIG_INT1              0x00
#define   LSM6DSM_TRIG_INT2 		0x01


// SPI Mode

#define  LSM6DSM_SPI_4_WIRE            0x00
#define  LSM6DSM_SPI_3_WIRE            0x01

// HPF CutOff
#define  LSM6DSM_HPFCF_0               0x00
#define  LSM6DSM_HPFCF_1               0x01
#define  LSM6DSM_HPFCF_2               0x02
#define  LSM6DSM_HPFCF_3               0x03




//Register Definition
#define LSM6DSM_WHO_AM_I				0x0F 

/************** Device Register  *******************/
#define LSM6DSM_TEST_PAGE  			0X00
#define LSM6DSM_FUNC_CFG_ACCESS  		0X01
#define LSM6DSM_SENSOR_SYNC_TIME  		0X04
#define LSM6DSM_SENSOR_SYNC_EN  		0X05
#define LSM6DSM_FIFO_CTRL1  			0X06
#define LSM6DSM_FIFO_CTRL2  			0X07
#define LSM6DSM_FIFO_CTRL3  			0X08
#define LSM6DSM_FIFO_CTRL4  			0X09
#define LSM6DSM_FIFO_CTRL5  			0X0A
#define LSM6DSM_DRDY_PULSE_CFG 			0X0B
#define LSM6DSM_INT1_CTRL  			0X0D
#define LSM6DSM_INT2_CTRL  			0X0E
#define LSM6DSM_WHO_AM_I_REG  			0X0F
#define LSM6DSM_CTRL1_XL  			0X10
#define LSM6DSM_CTRL2_G  			0X11
#define LSM6DSM_CTRL3_C  			0X12
#define LSM6DSM_CTRL4_C  			0X13
#define LSM6DSM_CTRL5_C  			0X14
#define LSM6DSM_CTRL6_C  			0X15
#define LSM6DSM_CTRL7_G  			0X16
#define LSM6DSM_CTRL8_XL  			0X17
#define LSM6DSM_CTRL9_XL  			0X18
#define LSM6DSM_CTRL10_C  			0X19
#define LSM6DSM_MASTER_CONFIG  		        0X1A
#define LSM6DSM_WAKE_UP_SRC  			0X1B
#define LSM6DSM_TAP_SRC  			0X1C
#define LSM6DSM_D6D_SRC  			0X1D
#define LSM6DSM_STATUS_REG  			0X1E
#define LSM6DSM_OUT_TEMP_L  			0X20
#define LSM6DSM_OUT_TEMP_H  			0X21
#define LSM6DSM_OUTX_L_G  			0X22
#define LSM6DSM_OUTX_H_G  			0X23
#define LSM6DSM_OUTY_L_G  			0X24
#define LSM6DSM_OUTY_H_G  			0X25
#define LSM6DSM_OUTZ_L_G  			0X26
#define LSM6DSM_OUTZ_H_G  			0X27
#define LSM6DSM_OUTX_L_XL  			0X28
#define LSM6DSM_OUTX_H_XL  			0X29
#define LSM6DSM_OUTY_L_XL  			0X2A
#define LSM6DSM_OUTY_H_XL  			0X2B
#define LSM6DSM_OUTZ_L_XL  			0X2C
#define LSM6DSM_OUTZ_H_XL  			0X2D
#define LSM6DSM_SENSORHUB1_REG  		0X2E
#define LSM6DSM_SENSORHUB2_REG  		0X2F
#define LSM6DSM_SENSORHUB3_REG  		0X30
#define LSM6DSM_SENSORHUB4_REG  		0X31
#define LSM6DSM_SENSORHUB5_REG  		0X32
#define LSM6DSM_SENSORHUB6_REG  		0X33
#define LSM6DSM_SENSORHUB7_REG  		0X34
#define LSM6DSM_SENSORHUB8_REG  		0X35
#define LSM6DSM_SENSORHUB9_REG  		0X36
#define LSM6DSM_SENSORHUB10_REG  		0X37
#define LSM6DSM_SENSORHUB11_REG  		0X38
#define LSM6DSM_SENSORHUB12_REG  		0X39
#define LSM6DSM_FIFO_STATUS1  			0X3A
#define LSM6DSM_FIFO_STATUS2  			0X3B
#define LSM6DSM_FIFO_STATUS3  			0X3C
#define LSM6DSM_FIFO_STATUS4  			0X3D
#define LSM6DSM_FIFO_DATA_OUT_L  		0X3E
#define LSM6DSM_FIFO_DATA_OUT_H  		0X3F
#define LSM6DSM_TIMESTAMP0_REG  		0X40
#define LSM6DSM_TIMESTAMP1_REG  		0X41
#define LSM6DSM_TIMESTAMP2_REG  		0X42
#define LSM6DSM_STEP_COUNTER_L  		0X4B
#define LSM6DSM_STEP_COUNTER_H  		0X4C
#define LSM6DSM_FUNC_SRC1  			0X53
#define LSM6DSM_FUNC_SRC2  			0x54
#define WRIST_TILT_IA                           0x55
#define LSM6DSM_TAP_CFG1  			0X58
#define LSM6DSM_TAP_THS_6D  			0X59
#define LSM6DSM_INT_DUR2  			0X5A
#define LSM6DSM_WAKE_UP_THS  			0X5B
#define LSM6DSM_WAKE_UP_DUR  			0X5C
#define LSM6DSM_FREE_FALL  			0X5D
#define LSM6DSM_MD1_CFG  			0X5E
#define LSM6DSM_MD2_CFG  			0X5F

#define LSM6DSM_X_OFS_USR 			0X73
#define LSM6DSM_Y_OFS_USR  			0X74
#define LSM6DSM_Z_OFS_USR  			0X75




//Status Register
#define LSM6DSM_STATUS_TDA                   BIT(2)
#define LSM6DSM_STATUS_GDA                   BIT(1)
#define LSM6DSM_STATUS_XLDA                  BIT(0)

//ODR Setting
// LSM6DSM ODR
#define   LSM6DSM_ODR_PD	0x00
#define   LSM6DSM_ODR_1_6HZ	0xB0
#define   LSM6DSM_ODR_12_5HZ    0x10
#define   LSM6DSM_ODR_26HZ	0x20
#define   LSM6DSM_ODR_52HZ	0x30
#define   LSM6DSM_ODR_104HZ	0x40
#define   LSM6DSM_ODR_208HZ	0x50
#define   LSM6DSM_ODR_416HZ	0x60
#define   LSM6DSM_ODR_833HZ	0x70
#define   LSM6DSM_ODR_1660HZ    0x80

// Operating Mode
#define  LSM6DSM_HIGH_POWER      0x00
#define  LSM6DSM_LOW_POWER 	0x10

// LSM6DSM FullScale

#define  LSM6DSM_FULLSCALE_2     0x00
#define  LSM6DSM_FULLSCALE_4     0x04
#define  LSM6DSM_FULLSCALE_8     0x08
#define  LSM6DSM_FULLSCALE_16    0x0C

//FIFO_CTRL3 - Gyro decimation
#define LSM6DSM_GYRO_NOT_IN_FIFO       0x00
#define LSM6DSM_GYRO_FIFO_NO_DEC       0x08
#define LSM6DSM_GYRO_FIFO_DEC_2       0x18
#define LSM6DSM_GYRO_FIFO_DEC_3       0x18

//FIFO_CTRL3 - Acc decimation
#define LSM6DSM_ACC_NOT_IN_FIFO       0x00
#define LSM6DSM_ACC_FIFO_NO_DEC       0x01
#define LSM6DSM_ACC_FIFO_DEC_2       0x02
#define LSM6DSM_ACC_FIFO_DEC_3       0x03

//FIFO_CTRL5 - FIFO MODE
#define FIFO_MODE_DISABLED              0x00
#define FIFO_MODE_STOP_WHEN_FULL        0x00
#define FIFO_MODE_CONTINUOUS            0x06

//FIFO_CTRL5 - FIFO ODR
#define   FIFO_ODR_DIS	        0x00
#define   FIFO_ODR_12_5HZ       0x08
#define   FIFO_ODR_26HZ	        0x10
#define   FIFO_ODR_52HZ	        0x18
#define   FIFO_ODR_104HZ	0x20

//CTRL2_G - Gyri ODR
#define   GYRO_ODR_PD	0x00
#define   GYRO_ODR_1_6HZ	0xB0
#define   GYRO_ODR_12_5HZ    0x10
#define   GYRO_ODR_26HZ	0x20
#define   GYRO_ODR_52HZ	0x30
#define   GYRO_ODR_104HZ	0x40
#define   GYRO_ODR_208HZ	0x50
#define   GYRO_ODR_416HZ	0x60
#define   GYRO_ODR_833HZ	0x70
#define   GYRO_ODR_1660HZ    0x80

//CTRL2_G - Gyro FULL SCALE
#define  GYRO_FULLSCALE_DIS     0x00
#define  GYRO_FULLSCALE_125     0x02
#define  GYRO_FULLSCALE_245     0x00
#define  GYRO_FULLSCALE_500     0x04
#define  GYRO_FULLSCALE_1000    0x08
#define  GYRO_FULLSCALE_2000    0x0C


//CTRL7_G - Power mode
//----------------------
//----------------------

#define GYRO_POWER_MODE_HIGH            0x00
#define GYRO_POWER_MODE_LOW             0x80
#define GYRO_HIGHPASS_FILTER_EN         0x40
#define GYRO_HIGHPASS_FILTER_DIS        0x00

//FIFO STATUS2
#define FIFO_NOT_EMPTY  0x10
#define FIFO_DATA_THRESHOLD_REACHED     0x80
#define FIFO_DATA_OVERRUN               0x40

/************** Access Device RAM  *******************/
#define LSM6DSM_ADDR0_TO_RW_RAM         0x62
#define LSM6DSM_ADDR1_TO_RW_RAM         0x63
#define LSM6DSM_DATA_TO_WR_RAM          0x64
#define LSM6DSM_DATA_RD_FROM_RAM        0x65

#define LSM6DSM_RAM_SIZE                4096

/************** Embedded functions register mapping  *******************/
#define LSM6DSM_SLV0_ADD                     0x02
#define LSM6DSM_SLV0_SUBADD                  0x03
#define LSM6DSM_SLAVE0_CONFIG                0x04
#define LSM6DSM_SLV1_ADD                     0x05
#define LSM6DSM_SLV1_SUBADD                  0x06
#define LSM6DSM_SLAVE1_CONFIG                0x07
#define LSM6DSM_SLV2_ADD                     0x08
#define LSM6DSM_SLV2_SUBADD                  0x09
#define LSM6DSM_SLAVE2_CONFIG                0x0A
#define LSM6DSM_SLV3_ADD                     0x0B
#define LSM6DSM_SLV3_SUBADD                  0x0C
#define LSM6DSM_SLAVE3_CONFIG                0x0D
#define LSM6DSM_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define LSM6DSM_CONFIG_PEDO_THS_MIN          0x0F
#define LSM6DSM_CONFIG_TILT_IIR              0x10
#define LSM6DSM_CONFIG_TILT_ACOS             0x11
#define LSM6DSM_CONFIG_TILT_WTIME            0x12
#define LSM6DSM_SM_THS                          0x13
#define LSM6DSM_MAG_SI_XX                    0x24
#define LSM6DSM_MAG_SI_XY                    0x25
#define LSM6DSM_MAG_SI_XZ                    0x26
#define LSM6DSM_MAG_SI_YX                    0x27
#define LSM6DSM_MAG_SI_YY                    0x28
#define LSM6DSM_MAG_SI_YZ                    0x29
#define LSM6DSM_MAG_SI_ZX                    0x2A
#define LSM6DSM_MAG_SI_ZY                    0x2B
#define LSM6DSM_MAG_SI_ZZ                    0x2C
#define LSM6DSM_MAG_OFFX_L                   0x2D
#define LSM6DSM_MAG_OFFX_H                   0x2E
#define LSM6DSM_MAG_OFFY_L                   0x2F
#define LSM6DSM_MAG_OFFY_H                   0x30
#define LSM6DSM_MAG_OFFZ_L                   0x31
#define LSM6DSM_MAG_OFFZ_H                   0x32

#define A_WRIST_TILT_LAT                     0x50
#define A_WRIST_TILT_THS                     0x54
#define A_WRIST_TILT_Mask                    0x59

extern double Z_Init_Angle;
extern double Y_Init_Angle;
extern double X_Init_Angle;

extern double Z_Angle;
extern double Y_Angle;
extern double X_Angle;

extern double Z_diff_Angle;
extern double Y_diff_Angle;
extern double X_diff_Angle;

extern double Z_Prev_Angle;
extern double Y_Prev_Angle;
extern double X_Prev_Angle;

extern float Z_Diff;
extern float Y_Diff;
extern float X_Diff;

extern float Z_gVal;
extern float Y_gVal;
extern float X_gVal;

extern float Z_Init_gVal;
extern float Y_Init_gVal;
extern float X_Init_gVal;


extern float Z_diff_gVal;
extern float Y_diff_gVal;
extern float X_diff_gVal;

extern double X_diff1;
extern double Y_diff1;
extern double Z_diff1;

extern double resultx1,resulty1,resultz1;

extern double UpperTiltThreshold;
extern double LowerTiltThreshold;

extern char X_positive;
extern char Y_positive;
extern char Z_positive;

#define TiltThreshold 10.0

extern char RegVal1;
extern uint8 Gyro_FIFOStatus;