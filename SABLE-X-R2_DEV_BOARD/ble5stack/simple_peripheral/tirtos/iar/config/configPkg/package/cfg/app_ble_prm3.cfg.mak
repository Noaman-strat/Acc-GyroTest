# invoke SourceDir generated makefile for app_ble.prm3
app_ble.prm3: .libraries,app_ble.prm3
.libraries,app_ble.prm3: package/cfg/app_ble_prm3.xdl
	$(MAKE) -f D:\mie\Work\ti\simplelink_cc2640r2_sdk_1_40_00_45\examples\rtos\SABLE-X-R2_DEV_BOARD\ble5stack\simple_peripheral\tirtos\iar\config/src/makefile.libs

clean::
	$(MAKE) -f D:\mie\Work\ti\simplelink_cc2640r2_sdk_1_40_00_45\examples\rtos\SABLE-X-R2_DEV_BOARD\ble5stack\simple_peripheral\tirtos\iar\config/src/makefile.libs clean

