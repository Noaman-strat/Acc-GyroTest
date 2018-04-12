
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
//bool read_WHOAMI(void);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* UART_H */