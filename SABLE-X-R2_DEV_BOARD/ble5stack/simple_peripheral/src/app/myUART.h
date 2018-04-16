
#ifndef MYUART_H
#define MYUART_H

#ifdef __cplusplus
extern "C" {
#endif

  
  /*

19.6 Initialization and Configuration
The UART module provides four I/O signals to be routed to the pads. The following signals are selected
through the IOCFGn registers in the IOC module.
The UART module provides four I/O functions to be routed to the pads:
• Inputs: RXD, CTS
• Outputs: TXD, RTS
CTS and RTS lines are active low.
NOTE: IOC must be configured before enabling UART, or unwanted transitions on input signals may
confuse UART on incoming transactions. When IOC is configured as UART-specific I/Os
(RXD, CTS, TXD, or RTS), IOC sets static output driver enable to the pad (output driver
enable = 1 for output TXD and RTS and output driver enable = 0 for inputs RXD and CTS).
To enable and initialize the UART, use the following steps:
1. Enable the serial power domain and enable the UART module in the PRCM module by writing to the
PRCM:UARTCLKGR register, the PRCM:UARTCLKGS register, and the PRCM:UARTCLKGDS
register, or by using the driver library functions:
PRCMPeripheralRunEnable(uint32_t), PRCMPeripheralSleepEnable(uint32_t),
PRCMPeripheralDeepSleepEnable(unit32_t)
and loading the setting to the clock controller by writing to the PRCM:CLKLOADCTL register or by
using the function
PRCMLoadSet().
2. Configure the IOC module to map UART signals to the correct GPIO pins. For more information on pin
connections, see Chapter 11.
19.7 Use of the UART Module
This section discusses the steps required to use a UART module. For this example, the UART clock is
assumed to be 24 MHz, and the desired UART configuration is the following:
• Baud rate: 115 200
• Data length of 8 bits
• One stop bit
• No parity
• FIFOs disabled
• No interrupts
The first thing to consider when programming the UART is the BRD because the UART:IBRD and
UART:FBRD registers must be written before the UART:LCRH register. The BRD can be calculated using
the equation described in Section 19.4.2.
BRD = 24 000 000 / (16 × 115 200) = 13.0208 (3)
The result of Equation 3 indicates that the UART:IBRD DIVINT field must be set to 13 decimal or 0xD.
Equation 4 calculates the value to be loaded into the UART:FBRD register.
UART:FBRD.DIVFRAC = integer (0.0208 × 64 + 0.5) = 1 (4)
With the BRD values available, the UART configuration is written to the module in the following order:
1. Disable the UART by clearing the UART:CTL UARTEN bit.
2. Write the integer portion of the BRD to the UART:IBRD register.
3. Write the fractional portion of the BRD to the UART:FBRD register.
4. Write the desired serial parameters to the UART:LCRH register

*/
/*********************************************************************
 * INCLUDES
 */
  
#include "PlaneRemote.h"
#include <./ti/drivers/UART.h>
/*********************************************************************
*  EXTERNAL VARIABLES
*/

extern char tempchar1, tempchar2, tempchar3;
 extern UART_Handle DebugUart;

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
 * @fn      UART_Init
 *
 * @brief   Setup UART for Accelerometer
 *
 * @param   none
 *
 * @return  none
 */
 char myUart_covertChar(char numtoConvert, char ind);
bool myUART_Config(void);
void myUART_send(char ds);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* UART_H */