
#include "myUART.h"
#include <hw_uart.h>
#include <hw_memmap.h>
#include "hw_types.h"
#include <hw_prcm.h>

//#include <stdbool.h>
//#include <ti/sysbios/knl/Clock.h>
//#include <ti/sysbios/family/arm/m3/Hwi.h>


//#include <ti/drivers/pin/PINCC26XX.h>

#include "board.h"

char tempchar1, tempchar2, tempchar3;

unsigned char TX_Data[16] = {0,0,0,0,',',0,0,0,0,',',0,0,0,0,0x0D, 0x0A};
  UART_Handle DebugUart = 0;
bool myUART_Config(void)
{
  

  char        input;
 
  UART_Params uartParams;
  
  // Initialize the UART driver.
  UART_init();
  
  // Create a UART with data processing off.
  UART_Params_init(&uartParams);
  uartParams.writeDataMode = UART_DATA_BINARY;
  uartParams.readDataMode = UART_DATA_BINARY;
  uartParams.readReturnMode = UART_RETURN_FULL;
  uartParams.readEcho = UART_ECHO_OFF;
  uartParams.dataLength = UART_LEN_8;
  uartParams.baudRate = 115200;
  //uartParams.writeMode = ;
  
  // Open an instance of the UART drivers
  DebugUart = UART_open(Board_UART0, &uartParams);
  if (DebugUart == NULL) {
      // UART_open() failed
      while (1);
  }
  // Loop forever echoing
  //while (1) {
  //    UART_read(uart, &input, 1);
   //   UART_write(uart, &input, 1);
 // }
  input= 0x55;
  UART_write(DebugUart, &input, 1);
  UART_write(DebugUart, &input, 1);
  UART_write(DebugUart, &input, 1);  
  return true;
}

void myUART_send(char ds)
{

}

char myUart_covertChar(char numtoConvert, char ind)
{
  tempchar1 = (numtoConvert/100);
  tempchar2 =  ((numtoConvert - (tempchar1 * 100))/10);    //
  tempchar3 = ((numtoConvert - ((tempchar1 * 100) + (tempchar2 * 10))));
  tempchar1 +=48;
  tempchar2 +=48;
  tempchar3 +=48;
  
return 1;
}