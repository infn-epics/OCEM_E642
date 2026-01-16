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

# Configure per-slave initialization parameters:
#   timeoutS   - timeout for each PS init before retry (default 5s)
#   maxRetries - max retries before marking PS failed (default 3)
# Each PS must receive VER, PRG, STA, COR, TEN data from RMT response
ocemSetInitParams 20, 3

# Configure polling rates:
#   idlePeriod    - polling interval when no commands active (seconds)
#   activePeriod  - polling interval when commands are being processed (seconds)
#   activeTimeout - how long to stay in active mode after last command (seconds)
# Default: idle=1.0s, active=0.1s, timeout=5.0s
ocemSetPollingRates 0.3, 0.1, 10

# Set debug level (can also be changed at runtime via IOC shell):
#   0 = Errors only (default)
#   1 = Basic info (commands sent, polling cycles)
#   2 = Protocol details (ENQ/ACK/NAK, raw bytes)
#   3 = Full trace (every function call, all data)
ocemSetDebug 0

#QUATM08
#drvAsynIPPortConfigure("OCEM_PORT", "192.168.192.30:4004") 
#ocemInit "OCEM_PORT", "7"


# Load database records ## ports name are already define in db
# Configuration parameters: SET_TOLERANCE, ZERO_TOLERANCE, SET_TIMEOUT_S
# Threshold parameters: CURRENT_THRESHOLD (A), VOLTAGE_THRESHOLD (V) for unsolicited msg generation
# Note: -1 means auto (5% of IMAX/VMAX), set to specific value to override
# Polling parameters: POLL_STATE=1 / POLL_ANALOG=1 to enable periodic SL/SA queries
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATB102, ADDR=0, IMAX=100,VMAX=25,SET_TOLERANCE=1.9,ZERO_TOLERANCE=1.8,SET_TIMEOUT_S=60,CURRENT_THRESHOLD=100,VOLTAGE_THRESHOLD=25, POLL_STATE=1, POLL_ANALOG=1")
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATM001, ADDR=2, IMAX=100,VMAX=25,SET_TOLERANCE=1.9,ZERO_TOLERANCE=1.8,SET_TIMEOUT_S=60,CURRENT_THRESHOLD=100,VOLTAGE_THRESHOLD=25, POLL_STATE=1, POLL_ANALOG=1")
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATM004, ADDR=1, IMAX=100,VMAX=25,SET_TOLERANCE=1.9,ZERO_TOLERANCE=1.8,SET_TIMEOUT_S=60,CURRENT_THRESHOLD=100,VOLTAGE_THRESHOLD=25, POLL_STATE=1, POLL_ANALOG=1")
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATB101, ADDR=3, IMAX=100,VMAX=25,SET_TOLERANCE=1.9,ZERO_TOLERANCE=1.8,SET_TIMEOUT_S=60,CURRENT_THRESHOLD=100,VOLTAGE_THRESHOLD=25, POLL_STATE=1, POLL_ANALOG=1")


iocInit()
