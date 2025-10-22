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
    int value,rt;
    ocemDpvt *p = (ocemDpvt*) prec->dpvt;
    if (!p || !drv) return -1;
    
    OCEM_Slave *slave = &drv->slaves[p->addr];

    if (strcasecmp(p->var, "STA") == 0) {
        strncpy(prec->val, slave->status, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "COR") == 0) {
        rt=sscanf(slave->current,"%d",&value);
        if (rt == 1) {
            sprintf(prec->val,"%d",value);
        }
        else {
            errlogPrintf("sscanf failed: rt is %d,string is %s",rt,slave->current);
            strncpy(prec->val, slave->current, sizeof(prec->val));
        }
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "TEN") == 0) {
        rt=sscanf(slave->voltage,"%d",&value);
        if (rt == 1) {
            sprintf(prec->val,"%d",value);
        }
        else {
            errlogPrintf("sscanf failed: rt is %d,string is %s",rt,slave->voltage);
            strncpy(prec->val, slave->voltage, sizeof(prec->val));
        }
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
    else if (strcasecmp(p->var, "SEL") == 0) {
        strncpy(prec->val, slave->selector, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "INI_CURMAX") == 0) {
        sprintf(prec->val,"%d", slave->currentPrgH);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "INI_CURMIN") == 0) {
        sprintf(prec->val,"%d", slave->currentPrgH);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "INI_VOLMAX") == 0) {
        sprintf(prec->val,"%d", slave->voltagePrgH);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "INI_VOLMIN") == 0) {
        sprintf(prec->val,"%d", slave->voltagePrgH);
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
    else if (strcasecmp(p->var, "SEL") == 0)
        *ppvt = drv->slaves[p->addr].ioscanSelector;
    else if (strncmp(p->var, "INI",3) == 0)
        *ppvt = drv->slaves[p->addr].ioscanInit;


    
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
    errlogPrintf1("Writing ENQ + poll address %x\n",msg[1]);
    
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
        errlogPrintf("Sending command %s\n",cmd);
        status = drv->pasynOctet->write(drv->pasynInterface->drvPvt,drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
        //NEED TO ADD READ FOR CLEANING
        memset(response, 0, responseSize);
        status = drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, response,responseSize-1,  &nbytesIn, &eomReason);
        if (status != asynSuccess) 
        {
            errlogPrintf1("send_command: nothing to read after command\n");
        }
        else
        {
            errlogPrintf1("send_command Reply: %s\n",response);
        }
    }
    
    return retVal;

}


void pad_value(const char *value, char *output)
{
    int len = strlen(value);

    if (len >= 7) {
        // Se è già lunga 7 o più, la copiamo così com’è
        strncpy(output, value,7);
    } else {
        // Calcoliamo quanti zeri servono
        int zeros = 7 - len;
        // Scriviamo gli zeri
        memset(output, '0', zeros);
        // Copiamo la parte numerica dopo gli zeri
        strcpy(output + zeros, value);
    }
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

int createCommand(char* outCmd,stringoutRecord *rec )
{
    ocemDpvt *p = (ocemDpvt*)rec->dpvt;
    if (!strcmp(p->var,"SP"))
    {
        char formatted[8];
        errlogPrintf1("Requested to set current. Formatting the value %s\n",rec->val);
        //sprintf(formatted,"SP %07d",prec->val);
        pad_value(rec->val,formatted);
        errlogPrintf1("formatted value is %s\n",formatted);
        sprintf(outCmd,"SP %s",formatted);
        errlogPrintf1("command to launch is %s\n",outCmd);
    }
    else if (!strcmp(p->var,"ON"))
        sprintf(outCmd,"ON");
    else if (!strcmp(p->var,"STB"))
        sprintf(outCmd,"STB");
    else if (!strcmp(p->var,"STR"))
        sprintf(outCmd,"STR");
    else if (!strcmp(p->var,"RES"))
        sprintf(outCmd,"RES");
    else if (!strcmp(p->var,"setPOL"))
    {
        //set Polarity can have OPN NEG or POS
        if ( (!strcmp (rec->val,"OPN")) || (!strcmp (rec->val,"NEG")) || (!strcmp (rec->val,"POS")) )
            sprintf(outCmd,"%s",rec->val);
        else 
        {
            errlogPrintf("value %s is invalid to set Polarity",rec->val);
            return -1;
        }
    }
    else if (!strcmp(p->var,"setSTA"))
    {
        //set Status can have ON or STB
        if ( (!strcmp (rec->val,"ON")) || (!strcmp (rec->val,"STB")))
            sprintf(outCmd,"%s",rec->val);
        else 
        {
            errlogPrintf("value %s is invalid to set Status",rec->val);
            return -1;
        }
    }
     else if (!strcmp(p->var,"PRG"))
        sprintf(outCmd,"PRG S");
    return 0;
}
static long so_write(stringoutRecord *prec)
{
    ocemDpvt *p = (ocemDpvt*)prec->dpvt;
    if (!p || !drv) return -1;
    char response[128];
    size_t responseSize=128;
    char cmdToLaunch[40];
    errlogPrintf("so_write_info: %s %d\n",p->var,p->addr);
    if (createCommand(cmdToLaunch,prec) != 0)
    {
         errlogPrintf("createCommand failed(addr=%d, cmd=%s)\n", p->addr, prec->val);
         return -1;
    }
    
    /* if (!strcmp(p->var,"SP"))
    {
        char formatted[8];
        errlogPrintf1("Requested to set current. Formatting the value %s\n",prec->val);
        //sprintf(formatted,"SP %07d",prec->val);
        pad_value(prec->val,formatted);
        errlogPrintf1("formatted value is %s\n",formatted);
        sprintf(prec->val,"SP %s\0",formatted);
        errlogPrintf1("command to launch is %s\n",prec->val);
        
    } */ 
    epicsMutexLock(drv->ioLock);
    //strcat(prec->val,"\r\n");
    strcpy(prec->val,cmdToLaunch);
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