#include "SPI.h"
#include <hw_ssi.h>
#include <hw_memmap.h>
#include "hw_types.h"
#include "hw_prcm.h"
//#include <stdbool.h>
//#include <ti/sysbios/knl/Clock.h>
//#include <ti/sysbios/family/arm/m3/Hwi.h>


//#include <ti/drivers/pin/PINCC26XX.h>

#include "board.h"


bool SPI_Init(void)
{
  //Enabled SSI0 clock
  HWREG(PRCM_BASE + PRCM_O_SSICLKGS) = PRCM_SSICLKGS_CLK_EN_SSI0;
  HWREG(PRCM_BASE + PRCM_O_SSICLKGR) = PRCM_SSICLKGR_CLK_EN_SSI0;
  HWREG(PRCM_BASE + PRCM_O_SSICLKGDS) = PRCM_SSICLKGS_CLK_EN_SSI0;
  //load the clock changes
  HWREG(PRCM_BASE + PRCM_O_CLKLOADCTL) = PRCM_CLKLOADCTL_LOAD;
  
  //wait for LOAD to be done
  //wait here
  //wait here
  
  IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_RX, IOC_STD_INPUT);
  IOCPortConfigureSet(IOID_9, IOC_PORT_MCU_SSI0_TX, IOC_STD_OUTPUT); 
  IOCPortConfigureSet(IOID_10, IOC_PORT_MCU_SSI0_CLK, IOC_STD_OUTPUT);   
  IOCPortConfigureSet(IOID_11, IOC_PORT_MCU_SSI0_FSS, IOC_STD_OUTPUT); 
  
  HWREG(SSI0_BASE + SSI_O_CR0) = ((0x18 << SSI_CR0_SCR_S) | SSI_CR0_DSS_8_BIT ) ;
  //CR1 - default settings are ok

  //Data is read and written to DR register
  //SR is status register that is used check if module is busy and/or in which state

  //prescalar divider is 2, 48 MHz clock, 1MBPs datarate
  HWREG(SSI0_BASE + SSI_O_CPSR) = 0x02;   
  
  //use TX and RX interrupts
  HWREG(SSI0_BASE + SSI_O_IMSC) = (SSI_IMSC_TXIM | SSI_IMSC_RXIM);
  
  //interrupt status can be read from SSI_0_RIS register
  
  
  return true;
}