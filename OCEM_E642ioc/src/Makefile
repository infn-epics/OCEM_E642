TOP=../..

include $(TOP)/configure/CONFIG

PROD_IOC = OCEM_E642Ioc

# dbd dello IOC
DBD += OCEM_E642Ioc.dbd

#OCEM_E642Ioc_DBD += base.dbd
OCEM_E642Ioc_DBD += OCEM_E642Support.dbd

# entry point
OCEM_E642Ioc_SRCS += OCEM_E642Ioc_registerRecordDeviceDriver.cpp
OCEM_E642Ioc_SRCS += OCEM_E642Main.cpp

# Librerie da linkare
OCEM_E642Ioc_LIBS += OCEM_E642     # <-- la tua libreria driver
OCEM_E642Ioc_LIBS += asyn stream   # quello che usi
OCEM_E642Ioc_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
