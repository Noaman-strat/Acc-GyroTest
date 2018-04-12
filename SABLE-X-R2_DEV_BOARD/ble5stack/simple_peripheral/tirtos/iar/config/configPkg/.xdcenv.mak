#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = D:/mie/Work/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages;D:/mie/Work/ti/simplelink_cc2640r2_sdk_1_40_00_45/source;D:/mie/Work/ti/simplelink_cc2640r2_sdk_1_40_00_45/source/ti/ble5stack
override XDCROOT = D:/mie/Work/ti/xdctools_3_50_02_20_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = D:/mie/Work/ti/simplelink_cc2640r2_sdk_1_40_00_45/kernel/tirtos/packages;D:/mie/Work/ti/simplelink_cc2640r2_sdk_1_40_00_45/source;D:/mie/Work/ti/simplelink_cc2640r2_sdk_1_40_00_45/source/ti/ble5stack;D:/mie/Work/ti/xdctools_3_50_02_20_core/packages;..
HOSTOS = Windows
endif
