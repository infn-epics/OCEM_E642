TOP=../..

include $(TOP)/configure/CONFIG

# Usa typed rset
USR_CPPFLAGS += -DUSE_TYPED_RSET

# Nome della libreria del driver
LIBRARY += OCEM_E642

# Sorgenti del driver
OCEM_E642_SRCS += drvOCEM.c
OCEM_E642_SRCS += devOCEM.c
# OCEM_E642_SRCS += OCEM_E642Version.cpp

# # Auto-genera header di versione
# GENVERSION = OCEM_E642Version.h
# GENVERSIONMACRO = OCEM_E642VERSION

# File DBD della libreria
DBD += OCEM_E642Support.dbd
 
OCEM_E642Support_DBD += base.dbd
OCEM_E642Support_DBD += asyn.dbd
OCEM_E642Support_DBD += drvAsynIPPort.dbd
OCEM_E642Support_DBD += stream.dbd
OCEM_E642Support_DBD += ocemCdc.dbd
# Questa libreria linka con asyn + base

OCEM_E642_LIBS += asyn stream
OCEM_E642_LIBS += $(EPICS_BASE_IOC_LIBS)

PROD_IOC = OCEM_E642Ioc
OCEM_E642Ioc_DBD += OCEM_E642Support.dbd

# entry point
OCEM_E642Ioc_SRCS += OCEM_E642Ioc_registerRecordDeviceDriver.cpp
OCEM_E642Ioc_SRCS += OCEM_E642Main.cpp

# Librerie da linkare
OCEM_E642Ioc_LIBS += OCEM_E642     # <-- la tua libreria driver
OCEM_E642Ioc_LIBS += asyn stream   # quello che usi
OCEM_E642Ioc_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

# Dipendenza esplicita per la generazione header
#devOCEM_E642Version$(DEP): $(COMMON_DIR)/$(GENVERSION)
