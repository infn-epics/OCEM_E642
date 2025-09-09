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


 



#asynSetTraceIOMask("readback_all", 0, 4)
#asynSetTraceMask("readback_all", 0,8)

#asynSetTraceIOMask("command_port", 0, 4)
#asynSetTraceMask("command_port", 0,9)






# Load database records ## ports name are already define in db
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEM,R=SERIAL0, PORT=OCEM_PORT, ADDR='k'")
#dbLoadRecords("$(TOP)/db/unimag-ocem.db", "P=SPARC:MAG:OCEM,R=PLXDPL01")

iocInit()
epicsThreadSleep(1.0)

# inizializzo il campo InputAddress
dbpf BTF:MAG:OCEM:SERIAL0:InputAddress 'j'
