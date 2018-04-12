
#ifndef SPI_H
#define SPI_H

#ifdef __cplusplus
extern "C" {
#endif

  
  /*


To enable and initialize the SSI, perform the following steps:
1. Ensure the corresponding power domain is powered up properly. For details, see Chapter 6.
2. Enable the appropriate SSI module in PRCM by writing to the PRCM:SSICLKGR register, the
PRCM:SSICLKGS register, and the PRCM:SSICLKGDS register, or by using the DriverLib functions:
PRCMPeripheralRunEnable(uint32_t)
PRCMPeripheralSleepEnable(uint32_t)
PRCMPeripheralDeepSleepEnable(uint32_t)
and then loading the setting to clock controller by writing to PRCM:CLKLOADCTL
or by using the DriverLib function.
PRCMLoadSet().
3. Configure the IOC module to route the SSIn_RX, SSIn_TX, SSIn_FSS, and SSIn_CLK functionalities
from I/Os to the SSI module. IOCFGn.PORTID must be written to the correct PORTIDs.
For each of the frame formats, the SSI is configured using the following steps:
1. Ensure that the SSE bit in the SSI:CR1 register is clear before making any configuration changes.
2. Select whether the SSI is a master or slave:
(a) For master operations, set the SSI:CR1 register to 0x0000 0000.
(b) For slave mode (output enabled), set the SSI:CR1 register to 0x0000 0004.
(c) For slave mode (output disabled), set the SSI:CR1 register to 0x0000 000C.
3. Configure the clock prescale divisor by writing to the SSI:CPSR register.
4. Write the SSI:CR0 register with the following configuration:
• Serial clock rate (SCR)
• Desired clock phase and polarity, if using Motorola SPI mode (SPH and SPO)
• The protocol mode: Motorola SPI, TI SSF, MICROWIRE (FRF)
• The data size (DSS)
Initialization and Configuration www.ti.com
1496 SWCU117H–February 2015–Revised August 2017
Submit Documentation Feedback
Copyright © 2015–2017, Texas Instruments Incorporated
Synchronous Serial Interface (SSI)
5. Optionally, configure the µDMA channel (see Chapter 12) and enable the DMA options in the
SSI:DMACR register.
6. Enable the SSI by setting the SSE bit in the SSI:CR1 register.
As an example, assume that the SSI configuration is required to operate with the following parameters:
• Master operation
• Texas Instruments SSI mode
• 1-Mbps bit rate
• 8 data bits
Assuming the system clock is 48 MHz, the bit-rate calculation is shown in Equation 6.
SSIn_CLK = PERDMACLK / [CPSDVSR × (1 + SCR)] 1 × 106 = 20 × 106 / [CPSDVSR × (1 + SCR)] 1000000
bps = 48000000 Hz / [2 × (1 + 23)] (6)
In this case, if CPSDVSR = 0x2, SCR must be 0x18.
The configuration sequence is:
1. Ensure that the SSE bit in the SSI:CR1 register is clear.
2. Write the SSI:CR1 register with a value of 0x0000 0000.
3. Write the SSI:CPSR register with a value of 0x0000 0002.
4. Write the SSI:CR0 register with a value of 0x0000 1817.
5. The SSI is then enabled by setting the SSE bit in the SSI:CR1 register

*/
/*********************************************************************
 * INCLUDES
 */
  
#include "PlaneRemote.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      SPI_Init
 *
 * @brief   Setup SPI for Accelerometer
 *
 * @param   none
 *
 * @return  none
 */
bool SPI_Init(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SPI_H */