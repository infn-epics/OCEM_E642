#!../../bin/linux-x86_64/OCEM_E642
# https://epics-modbus.readthedocs.io/en/latest/overview.htm

< envPaths

## Register all support components
dbLoadDatabase "../../dbd/OCEM_E642.dbd"
OCEM_E642_registerRecordDeviceDriver(pdbbase)


epicsEnvSet("STREAM_PROTOCOL_PATH", "../../db")
#puts "STREAM_PROTOCOL_PATH=$(STREAM_PROTOCOL_PATH)"

# Configure Serial communication

drvAsynIPPortConfigure("OCEM_PORT", "192.168.192.20:4001") 
ocemInit "OCEM_PORT",2,"10,11"

 
#asynSetTraceIOMask("readback_all", 0, 4)
#asynSetTraceMask("readback_all", 0,8)

# Load database records ## ports name are already define in db
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEM,R=SLAVE10, PORT=OCEM_PORT, ADDR=10, IMAX=280,VMAX=40")
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEM,R=SLAVE11, PORT=OCEM_PORT, ADDR=11, IMAX=280,VMAX=40")
dbLoadRecords("$(TOP)/db/unimag-ocem.db", "P=BTF:MAG:OCEM,R=SLAVE11")
dbLoadRecords("$(TOP)/db/unimag-ocem.db", "P=BTF:MAG:OCEM,R=SLAVE10")

iocInit()
