#include <dbAccess.h>
#include <recGbl.h>
#include <devSup.h>
#include <stringinRecord.h>
#include <stringoutRecord.h>

#include "drvOCEM.h"

typedef struct {
    int addr;        // slave address (0..31)
    char var[32];    // "STATUS", "CURRENT", "VOLTAGE", ...
    //struct OCEM_Var *varRef; // puntatore diretto alla variabile nello slave da usare in caso di generalizzazione.
} ocemDpvt;

OCEM_Driver *drv = NULL;   
extern unsigned char ocem_calc_cdc(const unsigned char *buf, size_t cmdLen);
/* ---- init record ---- */
static long si_init_record(stringinRecord *prec) 
{
    // Usare INP come indirizzo slave (es. "@3")
    ocemDpvt *pvt = calloc(1, sizeof(ocemDpvt));
    int addr;
    char varname[32];
    //errlogPrintf("INP is \"%s\"\n",prec->inp.value.instio.string);
    if (sscanf(prec->inp.value.instio.string, "%d %31s", &addr, varname) != 2) 
    {
        errlogPrintf("Bad INP '%s' in record %s\n",
        prec->inp.value.instio.string, prec->name);
        return S_db_badField;
    }
    if (prec->inp.type != INST_IO) {
        recGblRecordError(S_db_badField, (void*)prec,
                          "devOcemStringin (init_record) Illegal INP field");
        return S_db_badField;
    }
    pvt->addr = addr; 
    strncpy(pvt->var, varname, sizeof(pvt->var));
    prec->dpvt = pvt;
    return 0;
}

/* ---- read record ---- */
static long si_read(stringinRecord *prec) {
    ocemDpvt *p = (ocemDpvt*) prec->dpvt;
    if (!p || !drv) return -1;
    
    OCEM_Slave *slave = &drv->slaves[p->addr];

    if (strcasecmp(p->var, "STA") == 0) {
        strncpy(prec->val, slave->status, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "COR") == 0) {
        strncpy(prec->val, slave->current, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "TEN") == 0) {
        strncpy(prec->val, slave->voltage, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "POL") == 0) {
        strncpy(prec->val, slave->polarity, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "ALL") == 0) {
        strncpy(prec->val, slave->alarms, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    // etc...
    return 0;
}

/* ---- IO Intr support ---- */
static long si_get_ioint_info(int cmd, stringinRecord *prec, IOSCANPVT *ppvt) {
    
    ocemDpvt *p = (ocemDpvt*) prec->dpvt;
    if (!p) return -1;

    //OCEM_Slave *slave = &drv->slaves[p->addr];
    errlogPrintf("si_get_ioint_info: %s %d\n",p->var,p->addr);
    if (strcasecmp(p->var, "STA") == 0)
        *ppvt = drv->slaves[p->addr].ioscanStatus;
    else if (strcasecmp(p->var, "COR") == 0)
        *ppvt = drv->slaves[p->addr].ioscanCurrent;
    else if (strcasecmp(p->var, "TEN") == 0)
        *ppvt = drv->slaves[p->addr].ioscanVoltage;
    else if (strcasecmp(p->var, "POL") == 0)
        *ppvt = drv->slaves[p->addr].ioscanPolarity;
     else if (strcasecmp(p->var, "ALL") == 0)
        *ppvt = drv->slaves[p->addr].ioscanAlarms;


    
    return 0;
}

/* ---- Device support entry table ---- */
struct {
    long            num;
    DEVSUPFUN       report;
    DEVSUPFUN       init;
    DEVSUPFUN       init_record;
    DEVSUPFUN       get_ioint_info;
    DEVSUPFUN       read_si;
} devSiOCEM  = {
    5, NULL, NULL, (DEVSUPFUN)si_init_record,
     (DEVSUPFUN)si_get_ioint_info, (DEVSUPFUN)si_read
};
epicsExportAddress(dset, devSiOCEM);

int send_command(OCEM_Driver* drv,int slaveAddress,char* cmd,char*response,size_t responseSize)
{
    char msg[32];
    asynStatus status;
    size_t msgLen;
   
    size_t nbytesIn=0;
    size_t nbytesOut=0;
    int eomReason = 0;
    //printf("Called poll_request\n");
    msg[0] = 0x05;        // ENQ
    msg[1] = (unsigned char) (slaveAddress+0x60);
    msgLen = 2;
    errlogPrintf("Writing ENQ + poll address %x\n",msg[1]);
    
    status = drv->pasynOctet->write(drv->pasynInterface->drvPvt,drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
    if (status != asynSuccess) {
        errlogPrintf("send_command: errore write enq +address\n");
        return -1;
    }
    epicsThreadSleep(0.05);
    memset(response, 0, responseSize);
    status = drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, response,responseSize-1,  &nbytesIn, &eomReason);
    int retVal=0;
    
    if ((nbytesIn == 1) && ( (unsigned char)response[0]==0x6))
    {
        //errlogPrintf("Obtained EOT FIFO EMPTY for slave %d ",slave->addr);
        retVal= 0;
    }
    else
    { 
        errlogPrintf("send_command: errore read risposta al ENQ+address %d\n",slaveAddress);
        errlogPrintf("send_command: read ricevuto, status=%d, nbytesIn=%zu, eomReason=%d\n", status, nbytesIn,eomReason);
        errlogPrintf("send_command: read ricevuto byte 0x%02X\n", (unsigned char)response[0]);
        retVal = -1;
    }
    
    //errlogPrintf("POLL %d :OBTAINED ANSWER: len %ld\n",slave->addr,strlen(response));
    
    if (retVal <0)
        return retVal;
    
    if (retVal == 0)
    {
        //Prepara STX + addr + cmd + ETX + CDC
        msg[0] = 0x02; // STX
        msg[1] = (unsigned char) (slaveAddress+0x60);
        size_t cmdLen=strlen(cmd);
        memcpy(&msg[2], cmd, cmdLen);
        msg[2 + cmdLen] = 0x03; // ETX
        unsigned char cdc = ocem_calc_cdc((const unsigned char*)msg, cmdLen); 
        msg[3 + cmdLen] = cdc;
        msgLen = 4 + cmdLen;
        errlogPrintf("Sending command %s\n",msg);
        status = drv->pasynOctet->write(drv->pasynInterface->drvPvt,drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
    
        
    }
    
    return retVal;

}





static long so_init_record(stringoutRecord *prec)
{
    ocemDpvt *pvt = calloc(1, sizeof(ocemDpvt));
    int addr;
    char varname[32];
    if (!pvt) return S_db_noMemory;



if (sscanf(prec->out.value.instio.string, "%d %31s", &addr, varname) != 2) 
    {
        errlogPrintf("Bad INP '%s' in record %s\n",
        prec->out.value.instio.string, prec->name);
        return S_db_badField;
    }
    if (prec->out.type != INST_IO) {
        recGblRecordError(S_db_badField, (void*)prec,
                          "devOcemStringout (init_record) Illegal OUT field");
        return S_db_badField;
    }
    pvt->addr = addr; 


    //p->addr = parseAddrFromLink(prec->out.value.instio.string);
    strncpy(pvt->var, varname, sizeof(pvt->var));
    prec->dpvt = pvt;
  

    return 0;
}

static long so_write(stringoutRecord *prec)
{
    ocemDpvt *p = (ocemDpvt*)prec->dpvt;
    if (!p || !drv) return -1;
    char response[128];
    size_t responseSize=128;
    // Qui invii il comando al device
    errlogPrintf("so_write_info: %s %d\n",p->var,p->addr);
    epicsMutexLock(drv->ioLock);
    int status=send_command(drv,p->addr,prec->val,response,responseSize);
    if (status != 0) {
        errlogPrintf("OCEM write failed (addr=%d, cmd=%s)\n", p->addr, prec->val);
        return -1;
    }
    epicsMutexUnlock(drv->ioLock);
    return 0;
}

struct {
    long      number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN write;
} devSoOCEM = {
    5,
    NULL,
    NULL,
    so_init_record,
    NULL,        /* no I/O Intr per output */
    so_write
};
epicsExportAddress(dset, devSoOCEM);