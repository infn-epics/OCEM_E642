#!../../bin/linux-x86_64/OCEM_E642
# https://epics-modbus.readthedocs.io/en/latest/overview.htm

< envPaths

## Register all support components
dbLoadDatabase "../../dbd/OCEM_E642.dbd"
OCEM_E642_registerRecordDeviceDriver(pdbbase)


# Configure Serial communication

#QUATM05
drvAsynIPPortConfigure("OCEM_PORT", "192.168.192.30:4002") 
ocemInit "OCEM_PORT",1,"4"

#QUATM08
#drvAsynIPPortConfigure("OCEM_PORT", "192.168.192.30:4004") 
#ocemInit "OCEM_PORT",1,"7"


# Load database records ## ports name are already define in db
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=DAFNE:MAG:OCEM,R=QUATM05, ADDR=4, IMAX=280,VMAX=40")
#dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=BTF:MAG:OCEM,R=SLAVE11, ADDR=11, IMAX=280,VMAX=40")
dbLoadRecords("$(TOP)/db/unimag-ocem.db", "P=DAFNE:MAG:OCEM,R=QUATM05")
#dbLoadRecords("$(TOP)/db/unimag-ocem.db", "P=BTF:MAG:OCEM,R=SLAVE10")

iocInit()
