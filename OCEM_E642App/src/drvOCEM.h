#ifndef DRVOCEM_H
#define DRVOCEM_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <epicsThread.h>
#include <epicsExport.h>
#include <dbScan.h>
#include <iocsh.h>
#include <errlog.h>
#define MAX_SLAVE 32

typedef struct {
    int addr;           // indirizzo slave
    double value;       // ultimo valore letto
    char status[40];
    char current[40];
    //IOSCANPVT ioscan;   // per notificare record
    IOSCANPVT ioscanStatus;
    IOSCANPVT ioscanCurrent;
    IOSCANPVT ioscanVoltage;
} OCEM_Slave;

typedef struct {
    char *port;              // nome porta seriale
    int nSlaves;
    int addrList[MAX_SLAVE];
    OCEM_Slave slaves[MAX_SLAVE];
    epicsThreadId threadId;
    int running;
} OCEM_Driver;





extern  OCEM_Driver *drv;

#endif