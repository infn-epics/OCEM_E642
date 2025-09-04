TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Use typed rset structure (see 3.16.1 release notes)
USR_CPPFLAGS += -DUSE_TYPED_RSET

# Build an IOC support library
# LIBRARY_IOC += OCEMPowersupplySupport

# Link locally-provided code into the support library,
# rather than directly into the IOC application, that
# causes problems on Windows DLL builds
#OCEMPowersupplySupport_SRCS += dbSubExample.c
#OCEMPowersupplySupport_SRCS += devOCEMPowersupplyVersion.c
#OCEMPowersupplySupport_SRCS += OCEMPowersupplyHello.c
#OCEMPowersupplySupport_SRCS += initTrace.c

OCEM_E642Support_LIBS += $(EPICS_BASE_IOC_LIBS)

# Auto-generate a header file containing a version string.
# Version comes from the VCS if available, else date+time.
GENVERSION = OCEM_E642Version.h
# Macro name
GENVERSIONMACRO = OCEM_E642VERSION

# Build the IOC application
PROD_IOC = OCEM_E642

# OCEMPowersupply.dbd will be created and installed
DBD += OCEM_E642.dbd

# OCEMPowersupply.dbd will include these files:
OCEM_E642_DBD += base.dbd
OCEM_E642_DBD += asyn.dbd
OCEM_E642_DBD += drvAsynIPPort.dbd
OCEM_E642_DBD += stream.dbd


OCEM_E642_SRCS += OCEM_E642_registerRecordDeviceDriver.cpp
OCEM_E642_SRCS += ocemCdc.c
OCEM_E642_DBD += $(TOP)/OCEM_E642App/Db/ocemCdc.dbd

#USR_SRCS += ocemCdc.c

# Build the main IOC entry point where needed
OCEM_E642_SRCS_DEFAULT += OCEM_E642Main.cpp
OCEM_E642_SRCS_vxWorks += -nil-

# Link in the code from our support library
OCEM_E642_LIBS += asyn
OCEM_E642_LIBS += stream
# To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file

# Link QSRV (pvAccess Server) if available
ifdef EPICS_QSRV_MAJOR_VERSION
    OCEM_E642_LIBS += qsrv
    OCEM_E642_LIBS += $(EPICS_BASE_PVA_CORE_LIBS)
    OCEM_E642_DBD += PVAServerRegister.dbd
    OCEM_E642_DBD += qsrv.dbd
endif

# Finally link IOC to the EPICS Base libraries
OCEM_E642_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE

# Explicit dependency needed for generated header file
devOCEM_E642Version$(DEP): $(COMMON_DIR)/$(GENVERSION)
