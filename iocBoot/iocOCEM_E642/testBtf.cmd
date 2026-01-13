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
ocemInit "OCEM_PORT",4,"0,2,1,3"

#QUATM08
#drvAsynIPPortConfigure("OCEM_PORT", "192.168.192.30:4004") 
#ocemInit "OCEM_PORT",1,"7"


# Load database records ## ports name are already define in db
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATB102, ADDR=0, IMAX=100,VMAX=25")
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATM001, ADDR=2, IMAX=100,VMAX=25")
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATM004, ADDR=1, IMAX=100,VMAX=25")
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEME642,R=QUATB101, ADDR=3, IMAX=100,VMAX=25")
dbLoadRecords("$(TOP)/db/unimag-ocemE642.db", "P=BTF:MAG:OCEME642,R=QUATB102")
dbLoadRecords("$(TOP)/db/unimag-ocemE642.db", "P=BTF:MAG:OCEME642,R=QUATM001")
dbLoadRecords("$(TOP)/db/unimag-ocemE642.db", "P=BTF:MAG:OCEME642,R=QUATM004")
dbLoadRecords("$(TOP)/db/unimag-ocemE642.db", "P=BTF:MAG:OCEME642,R=QUATB101")

iocInit()
