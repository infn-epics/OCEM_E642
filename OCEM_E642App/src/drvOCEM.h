#ifndef DRVOCEM_H
#define DRVOCEM_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <dbScan.h>
#include <dbAccess.h>
#include <dbFldTypes.h>   // tipi di campo
#include <dbCommon.h>     // struct dbCommon
#include <link.h>        
#include <iocsh.h>
#include <asynDriver.h>
#include <asynOctet.h>
#include <asynFloat64.h>

#include <errlog.h>
#define MAX_SLAVE 32
#define LOGLEVEL 0
#define MAX_OCEM_RECORDS 500

#define errlogPrintf1(...) \
    do { if (LOGLEVEL >= 1) errlogPrintf(__VA_ARGS__); } while(0)

typedef struct {
    int addr;        // slave address (0..31)
    char var[32];    // "STATUS", "CURRENT", "VOLTAGE", ...
    dbAddr   *linkedAddr;
    dbCommon *linkedRec;
    //struct OCEM_Var *varRef; // puntatore diretto alla variabile nello slave da usare in caso di generalizzazione.
} ocemDpvt;


typedef enum {
    STATE_IDLE,
    STATE_REQ_SET_CURRENT,      // richiesta da utente
    STATE_WAIT_ZERO,
    STATE_SET_STANDBY,
    STATE_SET_POLARITY,
    STATE_SET_ON,
    STATE_SET_NEW_CURRENT,
    STATE_RAMP_TO_TARGET
} OcemState;

typedef struct {
    int addr;           // indirizzo slave
    char lastSelCommand[32];
    char status[40];
    char current[40];
    char voltage[40];
    char polarity[40];
    char alarms[40];
    char selector[40];
    double IMAX;
    
    int  currentPrgH,currentPrgL;
    int  voltagePrgH,voltagePrgL;
    int  unimagStatus,integerPolarity;
    double requestedCurrent; // da record stringout
    int requestedPolarity;   // +1 o -1
    OcemState Ostate;
    //IOSCANPVT per notificare record
    IOSCANPVT ioscanStatus;
    IOSCANPVT ioscanCurrent;
    IOSCANPVT ioscanVoltage;
    IOSCANPVT ioscanPolarity;
    IOSCANPVT ioscanAlarms;
    IOSCANPVT ioscanSelector;
    IOSCANPVT ioscanInit;
} OCEM_Slave;

typedef struct {
    char *port;              // nome porta seriale
    int nSlaves;
    int addrList[MAX_SLAVE];
    OCEM_Slave slaves[MAX_SLAVE];
    epicsThreadId threadId;
    int running;
    int pollingPeriodParam;
    double ocemPollingPeriod;
    asynUser *pasynUser;
    asynInterface *pasynInterface;
    asynOctet *pasynOctet;
    epicsMutexId ioLock; 


} OCEM_Driver;

int send_command(OCEM_Driver* drv,int slaveAddress,char* cmd,char*response,size_t responseSize);
OCEM_Slave* findSlave(OCEM_Driver* drv,int slaveAddress);
void pad_value(const char *value, char *output);


extern  OCEM_Driver *drv;

#endif