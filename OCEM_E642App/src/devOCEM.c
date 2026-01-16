
#include <recGbl.h>
#include <devSup.h>
#include <stringinRecord.h>
#include <stringoutRecord.h>
#include <aoRecord.h>
#include <aiRecord.h>
#include <longinRecord.h>
#include <longoutRecord.h>

#include "drvOCEM.h"

#ifndef OCEM_RECORDS_VAR
#define  OCEM_RECORDS_VAR
ocemDpvt *ocem_records[MAX_OCEM_RECORDS];
int ocem_record_count = 0;
#endif





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
    
    // Set record prefix if not yet set (used for IMAX/VMAX lookup)
    ocem_setRecordPrefix(slave, prec->name);

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
        if (strcasecmp(slave->polarity,"NEG") == 0) slave->integerPolarity =-1;
        else if (strcasecmp(slave->polarity,"POS") == 0) slave->integerPolarity =1;
        else if (strcasecmp(slave->polarity,"OPN") == 0) slave->integerPolarity =0;
       
    }
    else if (strcasecmp(p->var, "ALL") == 0) {
        strncpy(prec->val, slave->alarms, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "SEL") == 0) {
        strncpy(prec->val, slave->selector, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "VER") == 0) {
        strncpy(prec->val, slave->version, sizeof(prec->val));
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
    else if (strcasecmp(p->var, "UNIMAG") == 0) {
        sprintf(prec->val,"%d", slave->unimagStatus);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
     else if (strcasecmp(p->var, "INTPOLA") == 0) {
        sprintf(prec->val,"%d", slave->integerPolarity);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "CMDSTATE") == 0) {
        // Return numeric command state for mbbi record
        sprintf(prec->val,"%d", (int)slave->cmdState);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "CMDSTATUS") == 0) {
        // Return human-readable status message
        strncpy(prec->val, slave->cmdStatusMsg, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "LASTCMD") == 0) {
        // Return last command sent
        strncpy(prec->val, slave->lastCommand, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    // UNIMAG state machine PVs
    else if (strcasecmp(p->var, "UNIMAG_STATE") == 0) {
        sprintf(prec->val, "%d", (int)slave->unimag.state);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "UNIMAG_STATUS") == 0) {
        strncpy(prec->val, slave->unimag.statusMsg, sizeof(prec->val));
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "UNIMAG_BUSY") == 0) {
        sprintf(prec->val, "%d", slave->unimag.busy);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "CHANNEL_STATE") == 0) {
        sprintf(prec->val, "%d", (int)slave->channelState);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "CURRENT_RB_A") == 0) {
        sprintf(prec->val, "%.4f", slave->currentRB);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "CURRENT_SP_A") == 0) {
        sprintf(prec->val, "%.4f", slave->currentSP);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    // Config parameters readback
    else if (strcasecmp(p->var, "SET_TOL") == 0) {
        sprintf(prec->val, "%.3f", slave->unimagCfg.setTolerance);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "ZERO_TOL") == 0) {
        sprintf(prec->val, "%.3f", slave->unimagCfg.zeroTolerance);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "SET_TIMEOUT") == 0) {
        sprintf(prec->val, "%.1f", slave->unimagCfg.setTimeoutS);
        prec->val[sizeof(prec->val)-1] = '\0';
    }
    else if (strcasecmp(p->var, "MAX_RETRIES") == 0) {
        sprintf(prec->val, "%d", slave->unimagCfg.maxRetries);
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
    else if (strcasecmp(p->var, "UNIMAG") == 0)
        *ppvt = drv->slaves[p->addr].ioscanStatus;
    else if (strcasecmp(p->var, "COR") == 0)
        *ppvt = drv->slaves[p->addr].ioscanCurrent;
    else if (strcasecmp(p->var, "TEN") == 0)
        *ppvt = drv->slaves[p->addr].ioscanVoltage;
    else if (strcasecmp(p->var, "POL") == 0)
        *ppvt = drv->slaves[p->addr].ioscanPolarity;
    else if (strcasecmp(p->var, "INTPOLA") == 0)
        *ppvt = drv->slaves[p->addr].ioscanPolarity;
     else if (strcasecmp(p->var, "ALL") == 0)
        *ppvt = drv->slaves[p->addr].ioscanAlarms;
    else if (strcasecmp(p->var, "SEL") == 0)
        *ppvt = drv->slaves[p->addr].ioscanSelector;
    else if (strcasecmp(p->var, "VER") == 0)
        *ppvt = drv->slaves[p->addr].ioscanInit;
    else if (strncmp(p->var, "INI",3) == 0)
        *ppvt = drv->slaves[p->addr].ioscanInit;
    else if (strcasecmp(p->var, "CMDSTATE") == 0)
        *ppvt = drv->slaves[p->addr].ioscanCmdState;
    else if (strcasecmp(p->var, "CMDSTATUS") == 0)
        *ppvt = drv->slaves[p->addr].ioscanCmdState;
    else if (strcasecmp(p->var, "LASTCMD") == 0)
        *ppvt = drv->slaves[p->addr].ioscanCmdState;
    // UNIMAG state machine I/O Intr
    else if (strcasecmp(p->var, "UNIMAG_STATE") == 0)
        *ppvt = drv->slaves[p->addr].ioscanUnimag;
    else if (strcasecmp(p->var, "UNIMAG_STATUS") == 0)
        *ppvt = drv->slaves[p->addr].ioscanUnimag;
    else if (strcasecmp(p->var, "UNIMAG_BUSY") == 0)
        *ppvt = drv->slaves[p->addr].ioscanUnimag;
    else if (strcasecmp(p->var, "CHANNEL_STATE") == 0)
        *ppvt = drv->slaves[p->addr].ioscanUnimag;
    else if (strcasecmp(p->var, "CURRENT_RB_A") == 0)
        *ppvt = drv->slaves[p->addr].ioscanUnimag;
    else if (strcasecmp(p->var, "CURRENT_SP_A") == 0)
        *ppvt = drv->slaves[p->addr].ioscanUnimag;


    
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
    
    // Get the slave and update state
    OCEM_Slave *slave = findSlave(drv, slaveAddress);
    if (slave) {
        slave->cmdState = CMD_SENDING;
        strncpy(slave->lastCommand, cmd, sizeof(slave->lastCommand) - 1);
        snprintf(slave->cmdStatusMsg, sizeof(slave->cmdStatusMsg), "Sending: %s", cmd);
        epicsTimeGetCurrent(&slave->cmdStartTime);
        scanIoRequest(slave->ioscanCmdState);
    }
    
    OCEM_INFO("[PS%d] CMD: sending '%s'\n", slaveAddress, cmd);
    
    msg[0] = 0x05;        // ENQ
    msg[1] = (unsigned char) (slaveAddress+0x60);
    msgLen = 2;
    
    OCEM_DETAIL("[PS%d] CMD: ENQ+0x%02X\n", slaveAddress, msg[1]);
    
    status = drv->pasynOctet->write(drv->pasynInterface->drvPvt,drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
    if (status != asynSuccess) {
        OCEM_ERR("[PS%d] CMD: ENQ write error\n", slaveAddress);
        if (slave) {
            slave->cmdState = CMD_NOT_REACHED;
            snprintf(slave->cmdStatusMsg, sizeof(slave->cmdStatusMsg), "FAILED: ENQ write error");
            scanIoRequest(slave->ioscanCmdState);
        }
        return -1;
    }
    
    // Update state to waiting for ACK
    if (slave) {
        slave->cmdState = CMD_WAIT_ACK;
        snprintf(slave->cmdStatusMsg, sizeof(slave->cmdStatusMsg), "Wait ACK: %s", cmd);
        scanIoRequest(slave->ioscanCmdState);
    }
    
    epicsThreadSleep(0.02);
    memset(response, 0, responseSize);
    status = drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, response,responseSize-1,  &nbytesIn, &eomReason);
    int retVal=0;
    
    if ((nbytesIn == 1) && ( (unsigned char)response[0]==0x6))
    {
        OCEM_DETAIL("[PS%d] CMD: got ACK\n", slaveAddress);
        retVal= 0;
    }
    else
    { 
        OCEM_ERR("[PS%d] CMD: expected ACK but got 0x%02X (nbytes=%zu)\n", 
                slaveAddress, (unsigned char)response[0], nbytesIn);
        if (slave) {
            slave->cmdState = CMD_NOT_REACHED;
            snprintf(slave->cmdStatusMsg, sizeof(slave->cmdStatusMsg), "FAILED: No ACK (got 0x%02X)", (unsigned char)response[0]);
            scanIoRequest(slave->ioscanCmdState);
        }
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
        
        OCEM_INFO("[PS%d] CMD: >>> '%s' >>>\n", slaveAddress, cmd);
        
        status = drv->pasynOctet->write(drv->pasynInterface->drvPvt,drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
        epicsThreadSleep(0.02);
        //NEED TO ADD READ FOR CLEANING
        memset(response, 0, responseSize);
        status = drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, response,responseSize-1,  &nbytesIn, &eomReason);
        if (status != asynSuccess) 
        {
            OCEM_DETAIL("[PS%d] CMD: no response after command\n", slaveAddress);
        }
        else
        {
            OCEM_DETAIL("[PS%d] CMD: response='%s'\n", slaveAddress, response);
        }
        
        // Command sent successfully - set to VERIFYING state
        // The polling loop will verify and update to DONE or NOT_REACHED
        if (slave) {
            slave->cmdState = CMD_VERIFYING;
            slave->cmdVerifyCount = 0;
            snprintf(slave->cmdStatusMsg, sizeof(slave->cmdStatusMsg), "Verifying: %s", cmd);
            scanIoRequest(slave->ioscanCmdState);
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
    pvt->var[sizeof(pvt->var)-1] = '\0'; 
    
    
    if (prec->flnk.type == DB_LINK) {
    /* ottieni dbAddr dal link */
    struct dbAddr addr ;
    
    if (dbNameToAddr(prec->flnk.value.pv_link.pvname, &addr) == 0) {
        pvt->linkedAddr = &addr;
        pvt->linkedRec  = addr.precord;
        }
    }
    pvt->linkedAddr = calloc(1, sizeof(struct dbAddr));
    if (dbNameToAddr(prec->flnk.value.pv_link.pvname, pvt->linkedAddr) != 0) 
    {
        free(pvt->linkedAddr);
        pvt->linkedAddr = NULL;
    }
    pvt->linkedRec = pvt->linkedAddr ? pvt->linkedAddr->precord : NULL;

    for (int i = 0; i < MAX_OCEM_RECORDS; i++) {
    if (ocem_records[i] == NULL) {
        ocem_records[i] = pvt;
        break;
        }
    }
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
    else if (!strcmp(p->var,"RMT"))
        sprintf(outCmd,"RMT");
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
    if (!strcmp(p->var,"setPollingPeriod"))
    {
        printf("setting idle polling period. Before: %f ",drv->idlePollingPeriod);
        float newval;
        if (sscanf(prec->val,"%f",&newval) == 1)
        {
            printf(" After: new val %f\n",newval);
            drv->idlePollingPeriod=newval;
            return 0;
        }
        return -1;

    }
    if (!strcmp(p->var,"SETI"))
    {
        OCEM_Slave* slave=findSlave(drv,p->addr);
        double newValue = atof(prec->val);
        epicsMutexLock(drv->ioLock);
        slave->requestedCurrent = newValue;
        slave->requestedPolarity = (newValue >= 0) ? +1 : -1;
        slave->Ostate = STATE_REQ_SET_CURRENT;
        epicsMutexUnlock(drv->ioLock);
        return 0;
    }
    // SETV is just for reading VMAX via FLNK chain (no actual command)
    if (!strcmp(p->var,"SETV"))
    {
        return 0;
    }
    // UNIMAG state machine: set current setpoint (in Amperes)
    if (!strcmp(p->var,"UNIMAG_CURRENT_SP"))
    {
        OCEM_Slave* slave = findSlave(drv, p->addr);
        if (!slave) return -1;
        
        // Check if busy
        if (slave->unimag.busy) {
            errlogPrintf("UNIMAG_CURRENT_SP: rejected (busy)\n");
            return -1;
        }
        
        double newCurrentA = atof(prec->val);
        epicsMutexLock(drv->ioLock);
        unimag_setCurrentSP(slave, newCurrentA);
        epicsMutexUnlock(drv->ioLock);
        return 0;
    }
    // UNIMAG state machine: set channel state (ON, STANDBY, RESET)
    if (!strcmp(p->var,"UNIMAG_STATE_SP"))
    {
        OCEM_Slave* slave = findSlave(drv, p->addr);
        if (!slave) return -1;
        
        // Check if busy
        if (slave->unimag.busy) {
            errlogPrintf("UNIMAG_STATE_SP: rejected (busy)\n");
            return -1;
        }
        
        ChannelState targetState = CH_OFF;
        if (strcasecmp(prec->val, "ON") == 0) {
            targetState = CH_ON;
        } else if (strcasecmp(prec->val, "STANDBY") == 0 || strcasecmp(prec->val, "STB") == 0) {
            targetState = CH_STANDBY;
        } else if (strcasecmp(prec->val, "RESET") == 0 || strcasecmp(prec->val, "RES") == 0) {
            targetState = CH_RESET;
        } else {
            errlogPrintf("UNIMAG_STATE_SP: invalid state '%s'\n", prec->val);
            return -1;
        }
        
        epicsMutexLock(drv->ioLock);
        unimag_setStateSP(slave, targetState);
        epicsMutexUnlock(drv->ioLock);
        return 0;
    } 
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

/* ========== Analog Output device support for UNIMAG config ========== */

static long ao_init_record(aoRecord *prec)
{
    ocemDpvt *pvt = calloc(1, sizeof(ocemDpvt));
    int addr;
    char varname[32];
    if (!pvt) return S_db_noMemory;

    if (sscanf(prec->out.value.instio.string, "%d %31s", &addr, varname) != 2) {
        errlogPrintf("Bad OUT '%s' in record %s\n",
            prec->out.value.instio.string, prec->name);
        return S_db_badField;
    }
    if (prec->out.type != INST_IO) {
        recGblRecordError(S_db_badField, (void*)prec,
            "devOcemAo (init_record) Illegal OUT field");
        return S_db_badField;
    }
    pvt->addr = addr;
    strncpy(pvt->var, varname, sizeof(pvt->var));
    pvt->var[sizeof(pvt->var)-1] = '\0';
    prec->dpvt = pvt;
    
    // For config records, apply the DB value to the driver during init
    // The DB VAL field contains the macro-substituted value (e.g., ZERO_TOLERANCE=1.8)
    // which PINI will process after init, so we don't need to do anything here.
    // Just let PINI fire and ao_write will apply the value.
    
    return 2; // Don't convert
}

static long ao_write(aoRecord *prec)
{
    ocemDpvt *p = (ocemDpvt*)prec->dpvt;
    if (!p || !drv) return 0;  // Not ready yet, succeed silently for PINI
    
    OCEM_Slave* slave = findSlave(drv, p->addr);
    if (!slave) return 0;  // Slave not found, succeed silently for PINI
    
    if (strcasecmp(p->var, "SET_TOL") == 0) {
        slave->unimagCfg.setTolerance = prec->val;
        errlogPrintf("UNIMAG[%d]: set_tolerance = %.3f\n", p->addr, prec->val);
    } else if (strcasecmp(p->var, "ZERO_TOL") == 0) {
        slave->unimagCfg.zeroTolerance = prec->val;
        errlogPrintf("UNIMAG[%d]: zero_tolerance = %.3f\n", p->addr, prec->val);
    } else if (strcasecmp(p->var, "SET_TIMEOUT") == 0) {
        slave->unimagCfg.setTimeoutS = prec->val;
        errlogPrintf("UNIMAG[%d]: set_timeout = %.1f s\n", p->addr, prec->val);
    } else if (strcasecmp(p->var, "INIT_TIMEOUT") == 0) {
        slave->unimagCfg.initTimeoutS = prec->val;
        errlogPrintf("UNIMAG[%d]: init_timeout = %.1f s\n", p->addr, prec->val);
    } else if (strcasecmp(p->var, "RETRY_DELAY") == 0) {
        slave->unimagCfg.retryDelay = prec->val;
        errlogPrintf("UNIMAG[%d]: retry_delay = %.1f s\n", p->addr, prec->val);
    } else if (strcasecmp(p->var, "UNIMAG_CURRENT_SP") == 0) {
        // Allow ao record for current setpoint (alternative to stringout)
        if (slave->unimag.busy) {
            errlogPrintf("UNIMAG_CURRENT_SP: rejected (busy)\n");
            return -1;
        }
        epicsMutexLock(drv->ioLock);
        unimag_setCurrentSP(slave, prec->val);
        epicsMutexUnlock(drv->ioLock);
    } else if (strcasecmp(p->var, "CURRENT_TH") == 0) {
        // Current threshold in Amperes
        slave->currentThresholdA = prec->val;
        errlogPrintf("UNIMAG[%d]: current_threshold = %.3f A\n", p->addr, prec->val);
        // Send immediately if PRG is ready, otherwise will be sent when ready
        if (slave->IMAX > 0 && slave->currentPrgH > 0) {
            ocem_sendThreshold(slave, 0, prec->val);
        }
    } else if (strcasecmp(p->var, "VOLTAGE_TH") == 0) {
        // Voltage threshold in Volts
        slave->voltageThresholdV = prec->val;
        errlogPrintf("UNIMAG[%d]: voltage_threshold = %.3f V\n", p->addr, prec->val);
        // Send immediately if PRG is ready, otherwise will be sent when ready
        if (slave->VMAX > 0 && slave->voltagePrgH > 0) {
            ocem_sendThreshold(slave, 1, prec->val);
        }
    } else if (strcasecmp(p->var, "POLL_STATE") == 0) {
        // Enable/disable periodic SL polling (0=disabled, 1=enabled)
        slave->pollStateEnabled = (prec->val > 0.5) ? 1 : 0;
        // Initialize timestamp so first query happens after interval
        epicsTimeGetCurrent(&slave->lastForceStateTime);
        errlogPrintf("OCEM[%d]: poll_state_enabled = %d\n", p->addr, slave->pollStateEnabled);
    } else if (strcasecmp(p->var, "POLL_ANALOG") == 0) {
        // Enable/disable periodic SA polling (0=disabled, 1=enabled)
        slave->pollAnalogEnabled = (prec->val > 0.5) ? 1 : 0;
        // Initialize timestamp so first query happens after interval
        epicsTimeGetCurrent(&slave->lastForceOperatingTime);
        errlogPrintf("OCEM[%d]: poll_analog_enabled = %d\n", p->addr, slave->pollAnalogEnabled);
    } else if (strcasecmp(p->var, "POLL_STATE_INTERVAL") == 0) {
        // Interval for SL polling in seconds
        slave->pollStateIntervalS = prec->val;
        if (slave->pollStateIntervalS < 0.1) slave->pollStateIntervalS = 0.1;  // Min 100ms
        errlogPrintf("OCEM[%d]: poll_state_interval = %.1f s\n", p->addr, prec->val);
    } else if (strcasecmp(p->var, "POLL_ANALOG_INTERVAL") == 0) {
        // Interval for SA polling in seconds
        slave->pollAnalogIntervalS = prec->val;
        if (slave->pollAnalogIntervalS < 0.1) slave->pollAnalogIntervalS = 0.1;  // Min 100ms
        errlogPrintf("OCEM[%d]: poll_analog_interval = %.1f s\n", p->addr, prec->val);
    } else if (strcasecmp(p->var, "POLL_RATIO_NORMAL") == 0) {
        // Ratio for normal ON state (analog polls per state poll)
        slave->pollRatioNormal = prec->val;
        if (slave->pollRatioNormal < 0.01) slave->pollRatioNormal = 0.01;
        errlogPrintf("OCEM[%d]: poll_ratio_normal = %.2f\n", p->addr, prec->val);
    } else if (strcasecmp(p->var, "POLL_RATIO_SET") == 0) {
        // Ratio for GOING_TO_SET state (more analog polls during ramping)
        slave->pollRatioSet = prec->val;
        if (slave->pollRatioSet < 0.01) slave->pollRatioSet = 0.01;
        errlogPrintf("OCEM[%d]: poll_ratio_set = %.2f\n", p->addr, prec->val);
    } else if (strcasecmp(p->var, "POLL_RATIO_STATE") == 0) {
        // Ratio for CHANGE_POL/WAIT_POL states (more state polls)
        slave->pollRatioState = prec->val;
        if (slave->pollRatioState < 0.01) slave->pollRatioState = 0.01;
        errlogPrintf("OCEM[%d]: poll_ratio_state = %.2f\n", p->addr, prec->val);
    } else if (strcasecmp(p->var, "POLL_RATIO_BALANCED") == 0) {
        // Ratio for ZERO_STBY state (balanced polling)
        slave->pollRatioBalanced = prec->val;
        if (slave->pollRatioBalanced < 0.01) slave->pollRatioBalanced = 0.01;
        errlogPrintf("OCEM[%d]: poll_ratio_balanced = %.2f\n", p->addr, prec->val);
    } else {
        errlogPrintf("ao_write: unknown var '%s'\n", p->var);
        return -1;
    }
    
    return 0;
}

struct {
    long      number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN write;
    DEVSUPFUN special_linconv;
} devAoOCEM = {
    6,
    NULL,
    NULL,
    (DEVSUPFUN)ao_init_record,
    NULL,
    (DEVSUPFUN)ao_write,
    NULL
};
epicsExportAddress(dset, devAoOCEM);

/* ========== Longin device support for UNIMAG state ========== */

static long longin_init_record(longinRecord *prec)
{
    ocemDpvt *pvt = calloc(1, sizeof(ocemDpvt));
    int addr;
    char varname[32];
    if (!pvt) return S_db_noMemory;

    if (sscanf(prec->inp.value.instio.string, "%d %31s", &addr, varname) != 2) {
        errlogPrintf("Bad INP '%s' in record %s\n",
            prec->inp.value.instio.string, prec->name);
        return S_db_badField;
    }
    if (prec->inp.type != INST_IO) {
        recGblRecordError(S_db_badField, (void*)prec,
            "devOcemLongin (init_record) Illegal INP field");
        return S_db_badField;
    }
    pvt->addr = addr;
    strncpy(pvt->var, varname, sizeof(pvt->var));
    pvt->var[sizeof(pvt->var)-1] = '\0';
    prec->dpvt = pvt;
    return 0;
}

static long longin_read(longinRecord *prec)
{
    ocemDpvt *p = (ocemDpvt*)prec->dpvt;
    if (!p || !drv) return -1;
    
    OCEM_Slave* slave = findSlave(drv, p->addr);
    if (!slave) return -1;
    
    if (strcasecmp(p->var, "UNIMAG_STATE") == 0) {
        prec->val = (epicsInt32)slave->unimag.state;
    } else if (strcasecmp(p->var, "CHANNEL_STATE") == 0) {
        prec->val = (epicsInt32)slave->channelState;
    } else if (strcasecmp(p->var, "UNIMAG_BUSY") == 0) {
        prec->val = slave->unimag.busy ? 1 : 0;
    } else if (strcasecmp(p->var, "HAS_ALARM") == 0) {
        prec->val = slave->hasAlarm;
    } else if (strcasecmp(p->var, "ALARM_MASK") == 0) {
        prec->val = (epicsInt32)slave->alarmMask;
    } else if (strcasecmp(p->var, "MAX_RETRIES") == 0) {
        prec->val = slave->unimagCfg.maxRetries;
    } else if (strcasecmp(p->var, "RETRY_COUNT") == 0) {
        prec->val = slave->unimag.retryCount;
    } else if (strcasecmp(p->var, "PRG_INIT") == 0) {
        prec->val = slave->unimag.prgInitialized ? 1 : 0;
    } else if (strcasecmp(p->var, "INI_CURMAX") == 0) {
        prec->val = slave->currentPrgH;
    } else if (strcasecmp(p->var, "INI_CURMIN") == 0) {
        prec->val = slave->currentPrgL;
    } else if (strcasecmp(p->var, "INI_VOLMAX") == 0) {
        prec->val = slave->voltagePrgH;
    } else if (strcasecmp(p->var, "INI_VOLMIN") == 0) {
        prec->val = slave->voltagePrgL;
    } else if (strcasecmp(p->var, "INTPOLA") == 0) {
        prec->val = slave->integerPolarity;
    } else {
        return -1;
    }
    
    return 0;
}

static long longin_get_ioint_info(int cmd, longinRecord *prec, IOSCANPVT *ppvt) {
    ocemDpvt *p = (ocemDpvt*)prec->dpvt;
    if (!p) return -1;
    
    errlogPrintf("li_get_ioint_info: %s %d\n", p->var, p->addr);
    
    if (strcasecmp(p->var, "UNIMAG_STATE") == 0 ||
        strcasecmp(p->var, "CHANNEL_STATE") == 0 ||
        strcasecmp(p->var, "UNIMAG_BUSY") == 0 ||
        strcasecmp(p->var, "RETRY_COUNT") == 0 ||
        strcasecmp(p->var, "INTPOLA") == 0) {
        *ppvt = drv->slaves[p->addr].ioscanUnimag;
    } else if (strcasecmp(p->var, "HAS_ALARM") == 0 ||
               strcasecmp(p->var, "ALARM_MASK") == 0) {
        *ppvt = drv->slaves[p->addr].ioscanAlarms;
    } else if (strcasecmp(p->var, "PRG_INIT") == 0 ||
               strcasecmp(p->var, "INI_CURMAX") == 0 ||
               strcasecmp(p->var, "INI_CURMIN") == 0 ||
               strcasecmp(p->var, "INI_VOLMAX") == 0 ||
               strcasecmp(p->var, "INI_VOLMIN") == 0) {
        *ppvt = drv->slaves[p->addr].ioscanInit;
    }
    return 0;
}

struct {
    long      number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN read;
} devLiOCEM = {
    5,
    NULL,
    NULL,
    (DEVSUPFUN)longin_init_record,
    (DEVSUPFUN)longin_get_ioint_info,
    (DEVSUPFUN)longin_read
};
epicsExportAddress(dset, devLiOCEM);

/* ========== Longout device support for UNIMAG config ========== */

static long longout_init_record(longoutRecord *prec)
{
    ocemDpvt *pvt = calloc(1, sizeof(ocemDpvt));
    int addr;
    char varname[32];
    if (!pvt) return S_db_noMemory;

    if (sscanf(prec->out.value.instio.string, "%d %31s", &addr, varname) != 2) {
        errlogPrintf("Bad OUT '%s' in record %s\n",
            prec->out.value.instio.string, prec->name);
        return S_db_badField;
    }
    if (prec->out.type != INST_IO) {
        recGblRecordError(S_db_badField, (void*)prec,
            "devOcemLongout (init_record) Illegal OUT field");
        return S_db_badField;
    }
    pvt->addr = addr;
    strncpy(pvt->var, varname, sizeof(pvt->var));
    pvt->var[sizeof(pvt->var)-1] = '\0';
    prec->dpvt = pvt;
    
    // Initialize with current value
    OCEM_Slave* slave = findSlave(drv, addr);
    if (slave && strcasecmp(varname, "MAX_RETRIES") == 0) {
        prec->val = slave->unimagCfg.maxRetries;
    }
    
    return 0;
}

static long longout_write(longoutRecord *prec)
{
    ocemDpvt *p = (ocemDpvt*)prec->dpvt;
    if (!p || !drv) return 0;  // Not ready yet, succeed silently for PINI
    
    OCEM_Slave* slave = findSlave(drv, p->addr);
    if (!slave) return 0;  // Slave not found, succeed silently for PINI
    
    if (strcasecmp(p->var, "MAX_RETRIES") == 0) {
        slave->unimagCfg.maxRetries = prec->val;
        errlogPrintf("UNIMAG[%d]: max_retries = %d\n", p->addr, (int)prec->val);
    } else if (strcasecmp(p->var, "UNIMAG_STATE_SP") == 0) {
        // longout for state setpoint by number
        if (slave->unimag.busy) {
            errlogPrintf("UNIMAG_STATE_SP: rejected (busy)\n");
            return -1;
        }
        ChannelState targetState = (ChannelState)prec->val;
        epicsMutexLock(drv->ioLock);
        unimag_setStateSP(slave, targetState);
        epicsMutexUnlock(drv->ioLock);
    } else {
        errlogPrintf("longout_write: unknown var '%s'\n", p->var);
        return -1;
    }
    
    return 0;
}

struct {
    long      number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN write;
} devLoOCEM = {
    5,
    NULL,
    NULL,
    (DEVSUPFUN)longout_init_record,
    NULL,
    (DEVSUPFUN)longout_write
};
epicsExportAddress(dset, devLoOCEM);

/* ========== AI device support for UNIMAG readbacks ========== */

static long ai_init_record(aiRecord *prec)
{
    ocemDpvt *pvt = calloc(1, sizeof(ocemDpvt));
    int addr;
    char varname[32];
    if (!pvt) return S_db_noMemory;

    if (sscanf(prec->inp.value.instio.string, "%d %31s", &addr, varname) != 2) {
        errlogPrintf("Bad INP '%s' in record %s\n",
            prec->inp.value.instio.string, prec->name);
        return S_db_badField;
    }
    if (prec->inp.type != INST_IO) {
        recGblRecordError(S_db_badField, (void*)prec,
            "devOcemAi (init_record) Illegal INP field");
        return S_db_badField;
    }
    pvt->addr = addr;
    strncpy(pvt->var, varname, sizeof(pvt->var));
    pvt->var[sizeof(pvt->var)-1] = '\0';
    prec->dpvt = pvt;
    return 2; // Don't convert
}

static long ai_read(aiRecord *prec)
{
    ocemDpvt *p = (ocemDpvt*)prec->dpvt;
    if (!p || !drv) {
        errlogPrintf("ai_read: NULL dpvt or drv\n");
        return -1;
    }
    
    OCEM_Slave* slave = findSlave(drv, p->addr);
    if (!slave) {
        errlogPrintf("ai_read: findSlave(%d) returned NULL\n", p->addr);
        return -1;
    }
    
    // Set record prefix if not yet set (used for IMAX/VMAX lookup)
    ocem_setRecordPrefix(slave, prec->name);
    
    if (strcasecmp(p->var, "CURRENT_RB_A") == 0) {
        prec->val = slave->currentRB;
    } else if (strcasecmp(p->var, "CURRENT_SP_A") == 0) {
        prec->val = slave->currentSP;
    } else if (strcasecmp(p->var, "VOLTAGE_RB") == 0) {
        prec->val = slave->voltageRB;
    } else if (strcasecmp(p->var, "SET_TOL") == 0) {
        prec->val = slave->unimagCfg.setTolerance;
    } else if (strcasecmp(p->var, "ZERO_TOL") == 0) {
        prec->val = slave->unimagCfg.zeroTolerance;
    } else if (strcasecmp(p->var, "SET_TIMEOUT") == 0) {
        prec->val = slave->unimagCfg.setTimeoutS;
    } else if (strcasecmp(p->var, "IMAX_RB") == 0) {
        prec->val = slave->IMAX;
    } else if (strcasecmp(p->var, "VMAX_RB") == 0) {
        prec->val = slave->VMAX;
    } else if (strcasecmp(p->var, "POLL_RATIO_NORMAL") == 0) {
        prec->val = slave->pollRatioNormal;
    } else if (strcasecmp(p->var, "POLL_RATIO_SET") == 0) {
        prec->val = slave->pollRatioSet;
    } else if (strcasecmp(p->var, "POLL_RATIO_STATE") == 0) {
        prec->val = slave->pollRatioState;
    } else if (strcasecmp(p->var, "POLL_RATIO_BALANCED") == 0) {
        prec->val = slave->pollRatioBalanced;
    } else {
        return -1;
    }
    
    errlogPrintf("ai_read: %s addr=%d val=%.3f\n", p->var, p->addr, prec->val);
    
    prec->udf = 0;  // Clear undefined flag on successful read
    return 2; // Don't convert
}

static long ai_get_ioint_info(int cmd, aiRecord *prec, IOSCANPVT *ppvt) {
    ocemDpvt *p = (ocemDpvt*)prec->dpvt;
    if (!p) return -1;
    
    errlogPrintf("ai_get_ioint_info: %s %d\n", p->var, p->addr);
    
    if (strcasecmp(p->var, "CURRENT_RB_A") == 0 ||
        strcasecmp(p->var, "CURRENT_SP_A") == 0 ||
        strcasecmp(p->var, "VOLTAGE_RB") == 0) {
        *ppvt = drv->slaves[p->addr].ioscanUnimag;
    } else if (strcasecmp(p->var, "IMAX_RB") == 0 ||
               strcasecmp(p->var, "VMAX_RB") == 0) {
        *ppvt = drv->slaves[p->addr].ioscanInit;
    }
    return 0;
}

struct {
    long      number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN read;
    DEVSUPFUN special_linconv;
} devAiOCEM = {
    6,
    NULL,
    NULL,
    (DEVSUPFUN)ai_init_record,
    (DEVSUPFUN)ai_get_ioint_info,
    (DEVSUPFUN)ai_read,
    NULL
};
epicsExportAddress(dset, devAiOCEM);