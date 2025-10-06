#ifndef DRVOCEM_H
#define DRVOCEM_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <dbScan.h>
#include <iocsh.h>
#include <asynDriver.h>
#include <asynOctet.h>
#include <errlog.h>
#define MAX_SLAVE 32

typedef struct {
    char name[32];      // es. "STA", "COR", "VOLTAGE"
    char value[64];     // ultimo valore letto
    IOSCANPVT ioscan;   // canale di interrupt per questa variabile
} OCEM_Var;

typedef struct {
    int addr;           // indirizzo slave
    char lastSelCommand[32];
    char status[40];
    char current[40];
    char voltage[40];
    char polarity[40];
    char alarms[40];
    //IOSCANPVT per notificare record
    IOSCANPVT ioscanStatus;
    IOSCANPVT ioscanCurrent;
    IOSCANPVT ioscanVoltage;
    IOSCANPVT ioscanPolarity;
    IOSCANPVT ioscanAlarms;
} OCEM_Slave;

typedef struct {
    char *port;              // nome porta seriale
    int nSlaves;
    int addrList[MAX_SLAVE];
    OCEM_Slave slaves[MAX_SLAVE];
    epicsThreadId threadId;
    int running;
    asynUser *pasynUser;
    asynInterface *pasynInterface;
    asynOctet *pasynOctet;
    epicsMutexId ioLock; 


} OCEM_Driver;





extern  OCEM_Driver *drv;

#endif