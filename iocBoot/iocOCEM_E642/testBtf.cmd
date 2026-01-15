#!../../bin/linux-x86_64/OCEM_E642ioc
# https://epics-modbus.readthedocs.io/en/latest/overview.htm

epicsEnvSet("TOP","../../")

## Register all support components
dbLoadDatabase "../../dbd/OCEM_E642Support.dbd"
#dbLoadDatabase("../../OCEM_E642App/Db/OCEM_E642Support.dbd")
OCEM_E642Ioc_registerRecordDeviceDriver(pdbbase)


# Configure Serial communication

#QUATM05
drvAsynIPPortConfigure("OCEM_PORT", "192.168.192.40:4014") 
ocemInit "OCEM_PORT", "0,2,1,3"

# Configure polling rates:
#   idlePeriod    - polling interval when no commands active (seconds)
#   activePeriod  - polling interval when commands are being processed (seconds)
#   activeTimeout - how long to stay in active mode after last command (seconds)
# Default: idle=1.0s, active=0.1s, timeout=5.0s
ocemSetPollingRates 0.05, 0.01, 10

# Set debug level (can also be changed at runtime via IOC shell):
#   0 = Errors only (default)
#   1 = Basic info (commands sent, polling cycles)
#   2 = Protocol details (ENQ/ACK/NAK, raw bytes)
#   3 = Full trace (every function call, all data)
ocemSetDebug 2

#QUATM08
#drvAsynIPPortConfigure("OCEM_PORT", "192.168.192.30:4004") 
#ocemInit "OCEM_PORT", "7"


# Load database records ## ports name are already define in db
# Configuration parameters: SET_TOLERANCE, ZERO_TOLERANCE, SET_TIMEOUT_S
# Threshold parameters: CURRENT_THRESHOLD (A), VOLTAGE_THRESHOLD (V) for unsolicited msg generation
# Note: -1 means auto (5% of IMAX/VMAX), set to specific value to override
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATB102, ADDR=0, IMAX=100,VMAX=25,SET_TOLERANCE=1.9,ZERO_TOLERANCE=1.8,SET_TIMEOUT_S=20,CURRENT_THRESHOLD=1,VOLTAGE_THRESHOLD=2, FORCE_STATE_QUERY_S=0, FORCE_OPERATING_QUERY_S=0")
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATM001, ADDR=2, IMAX=100,VMAX=25,SET_TOLERANCE=1.9,ZERO_TOLERANCE=1.8,SET_TIMEOUT_S=20,CURRENT_THRESHOLD=5,VOLTAGE_THRESHOLD=1.25")
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATM004, ADDR=1, IMAX=100,VMAX=25,SET_TOLERANCE=1.9,ZERO_TOLERANCE=1.8,SET_TIMEOUT_S=20,CURRENT_THRESHOLD=5,VOLTAGE_THRESHOLD=1.25")
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATB101, ADDR=3, IMAX=100,VMAX=25,SET_TOLERANCE=1.9,ZERO_TOLERANCE=1.8,SET_TIMEOUT_S=20,CURRENT_THRESHOLD=5,VOLTAGE_THRESHOLD=1.25")


iocInit()
