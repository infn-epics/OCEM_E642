

#include "drvOCEM.h"
#include <ctype.h>
#include <time.h>


#define LOGLEVEL 0
#define errlogPrintf1(...) \
    do { if (LOGLEVEL >= 1) errlogPrintf(__VA_ARGS__); } while(0)
//#define errlogPrintf1
// static const char *simStrings[] = {
//     "JSTA STB", "10COR 2", "11STA ON", "11COR 3", "11TEN 100","10STA ON","10TEN 24","10COR 4.5","11STA STB"
// };
// static const int nSimStrings = sizeof(simStrings)/sizeof(simStrings[0]);

 unsigned char ocem_calc_cdc(const unsigned char *buf, size_t cmdLen) {
    unsigned char cdc = 0;
    //errlogPrintf("Len command = %ld\n",cmdLen);
    if (cmdLen > 1) {
        for (size_t i = 1; i <= 2 + cmdLen; i++) {
            cdc ^= buf[i];
        }
        cdc |=0x80;  //MSB must be always 1
    }
    return cdc;
}
OCEM_Slave* findSlave(OCEM_Driver* drv,int slaveAddress);
void ActivateInterrupt(int slaveId,char* cmd, char* val)
{
            // trova slave corrispondente
    OCEM_Slave *slave = findSlave(drv, slaveId);
    if (!slave) 
    {
        printf("Unknown slave addr=%d\n", slaveId);
        epicsThreadSleep(1.0);
        return;
    }
    if (strcmp(cmd, "STA") == 0) 
    {
        strncpy(slave->status, val, sizeof(slave->status));
        slave->status[sizeof(slave->status)-1] = '\0';
        scanIoRequest(slave->ioscanStatus);
    }
    else if (strcmp(cmd, "COR") == 0) {
        strncpy(slave->current, val, sizeof(slave->current));
        slave->current[sizeof(slave->current)-1] = '\0';
        scanIoRequest(slave->ioscanCurrent);
    }
    else if (strcmp(cmd, "TEN") == 0) 
    {
        strncpy(slave->voltage, val, sizeof(slave->voltage));
        slave->voltage[sizeof(slave->voltage)-1] = '\0';
        scanIoRequest(slave->ioscanVoltage);
    }
    else if (strcmp(cmd, "POL") == 0) 
    {
        
        strncpy(slave->polarity, val, sizeof(slave->polarity));
        slave->polarity[sizeof(slave->polarity)-1] = '\0';
        scanIoRequest(slave->ioscanPolarity);
    }
     else if (strcmp(cmd, "ALL") == 0) 
    {
        
        strncpy(slave->alarms, val, sizeof(slave->alarms));
        slave->alarms[sizeof(slave->alarms)-1] = '\0';
        scanIoRequest(slave->ioscanAlarms);
    }


}
/**
 * parseIntList - converte una stringa separata da virgole in un array di int
 * @param str      La stringa input, es. "10,11,15"
 * @param out      Array di int da riempire
 * @param maxOut   Numero massimo di elementi da scrivere in out
 * @return         Numero di elementi effettivamente parsati
 */
int parseIntList(const char *str, int *out, int maxOut)
{
    if (!str || !out || maxOut <= 0)
        return 0;

    char *tmp = strdup(str);
    if (!tmp)
        return 0;

    int count = 0;
    char *saveptr = NULL;
    char *tok = strtok_r(tmp, ",", &saveptr);

    while (tok && count < maxOut) {
        // rimuove eventuali spazi iniziali/finali
        while (*tok && isspace((unsigned char)*tok)) tok++;
        char *end = tok + strlen(tok) - 1;
        while (end > tok && isspace((unsigned char)*end)) *end-- = '\0';

        out[count++] = atoi(tok);
        tok = strtok_r(NULL, ",", &saveptr);
    }

    free(tmp);
    return count;
}

int parseReplyString(const char *recStr, int* slaveId, char* cmd,char*pvVal)
{
    if (!recStr || !slaveId || !cmd || !pvVal)
    {
        errlogPrintf("parseReplyString: some error occurred");
        return -1;
    }

    int i = 0;
    *slaveId = -1;
    //errlogPrintf("parseReplyString: recStr is %s\n",recStr);
    if (isdigit((unsigned char)recStr[i])) {
        errlogPrintf("address numeric is %c\n",recStr[i]);
        // Caso indirizzo numerico (es: "11COR 3.0")
        *slaveId = 0;
        while (isdigit((unsigned char)recStr[i])) {
            *slaveId = (*slaveId * 10) + (recStr[i] - '0');
            i++;
        }
    }
    else if (isalpha((unsigned char)recStr[i])) {
        //errlogPrintf("address digital is %c\n",recStr[i]);
        // Caso indirizzo letterale (es: "JCOR 3.0")
        *slaveId = (recStr[i] - 'A')+1; // converto in indice (A=0, B=1, ..., J=)
        i++;
    }
    else
    {
        errlogPrintf("%d is not alpha nor digit",recStr[i]);
    }
    int j = 0;
    while (recStr[i] && !isspace((unsigned char)recStr[i]) && j < 31) {
        cmd[j++] = recStr[i++];
    }
    cmd[j] = '\0';

    // 3. Salta eventuali spazi
    while (recStr[i] && isspace((unsigned char)recStr[i])) i++;

    // 4. Copia il valore rimanente
    strncpy(pvVal, recStr + i, 31);
    pvVal[31] = '\0';

   
    return 0;

}
#define MAX_LINE 32
void parseMultiReply(const char *input)
{
    char buffer[MAX_LINE];
    char cleaned[128];
    int i = 1;
    for (i = 1; i < strlen(input) - 2; ++i)
    {
        cleaned[i - 1] = input[i];
    }
    cleaned[i-1] = '\0';
    const char *ptr = cleaned;
    char lastSlaveChar = 'a';  // <-- qui memorizziamo il singolo char
    int firstLine = 1;

    while (*ptr) {
        int len = 0;
        while (*ptr && *ptr != '\n' && *ptr != '\r' && len < MAX_LINE-1) {
            buffer[len++] = *ptr++;
        }
        buffer[len] = '\0';
        //errlogPrintf("Buffer now:%s, len %d ",buffer,len);

        while (*ptr == '\n' || *ptr == '\r') ptr++;
        if (len == 0) continue;

        char fullLine[MAX_LINE+1];
        if (firstLine) {
            strcpy(fullLine, buffer);
            firstLine = 0;
        } else {
         // Prependiamo il singolo char dell'indirizzo salvato
          //errlogPrintf("Prependo address %c",lastSlaveChar);
            snprintf(fullLine, sizeof(fullLine), "%c%s", lastSlaveChar, buffer);
        }

        int slaveId;
        char cmd[32], val[32];
        //errlogPrintf("FullLine to parse is %s\n",fullLine);
        parseReplyString(fullLine, &slaveId, cmd, val);
        
        // Aggiorniamo l'ultimo indirizzo solo se era letterale (non numerico)
        if (isalpha((unsigned char)fullLine[0]))
            lastSlaveChar = fullLine[0];

        //errlogPrintf("-> SlaveID=%d | CMD=%s | VAL=%s\n", slaveId, cmd, val);
        ActivateInterrupt(slaveId,cmd,val);
    }
}





OCEM_Slave* findSlave(OCEM_Driver* drv,int slaveAddress)
{
    OCEM_Slave* pt=NULL;
    for (int i = 0; i < drv->nSlaves; i++) 
    {
        int ad=drv->addrList[i];
        if (drv->slaves[ad].addr==slaveAddress)
        {
            pt=&drv->slaves[ad];
            return pt;
        }
    }
    return pt;

}
char* getNextCommandForSlave(OCEM_Slave* slave)
{
    if (strcmp(slave->lastSelCommand,"SL"))
        return "SL";
    else return "SA";
}

int select_request(OCEM_Driver* drv,OCEM_Slave* slave,char*response,size_t responseSize)
{
     /*. Invia ENQ + address */
    unsigned char msg[32];
    asynStatus status;
    size_t msgLen;
   
    size_t nbytesIn=0;
    size_t nbytesOut=0;
    int eomReason = 0;
    //errlogPrintf("Called select_request on address %d\n",slave->addr);
    msg[0] = 0x05;        // ENQ
    msg[1] = (unsigned char) (slave->addr+0x60);
    msgLen = 2;
    
    status = drv->pasynOctet->write(drv->pasynInterface->drvPvt, drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
    if (status != asynSuccess) 
    {
        errlogPrintf("select_request: errore write ENQ\n");
        return -1;
    }
    epicsThreadSleep(0.05);
    /* 3. Leggi ACK/NAK */
    unsigned char ackBuf[1];
    //errlogPrintf("Reading ENQ + address answer\n");
    status = drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, (char*)ackBuf, 1,  &nbytesIn, &eomReason);
    if (status != asynSuccess || nbytesIn == 0) 
    {
        errlogPrintf("select_request: timeout o errore in attesa di ACK/NAK. \n");
        //epicsThreadSleep(OCEM_ENQ_DELAY_MS / 1000.0);
        return -1;
    }
    else
    {
        if (ackBuf[0] != 0x06) 
        { // 0x06 = ACK
            errlogPrintf("select_request: ricevuto NAK (0x%02X)\n", ackBuf[0]);
            return -1;
        }
        
    }
    /* 4. Prepara STX + addr + cmd + ETX + CDC */
    char *cmd=getNextCommandForSlave(slave);
    strncpy(slave->lastSelCommand,cmd,strlen(cmd));
    size_t cmdLen = strlen(cmd);
    msg[0] = 0x02; // STX
    msg[1] = (unsigned char) (slave->addr+0x60);
    memcpy(&msg[2], cmd, cmdLen);
    msg[2 + cmdLen] = 0x03; // ETX
    unsigned char cdc = ocem_calc_cdc(msg, cmdLen); 
    msg[3 + cmdLen] = cdc;
    msgLen = 4 + cmdLen;
    errlogPrintf1("  msg %s\n",msg);
    status = drv->pasynOctet->write(drv->pasynInterface->drvPvt,drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
    
    if (status != asynSuccess) {
        errlogPrintf("select_request: errore write comando\n");
        return -1;
    }
    epicsThreadSleep(0.05);
    //Leggi l'ack
    memset(response, 0, responseSize);
    status = drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, response,1,  &nbytesIn, &eomReason);
    if (status != asynSuccess) {
        errlogPrintf("select_request: errore read risposta ack al comando %s in select\n",msg);
        errlogPrintf("select_request: read ricevuto, status=%d, nbytesIn=%zu, eomReason=%d\n", status, nbytesIn,eomReason);
        errlogPrintf("select_request: read ricevuto byte 0x%02X\n", (unsigned char)response[0]);
        return -1;
    }
    return 0;
}



int poll_request(OCEM_Driver* drv,OCEM_Slave* slave,char*response,size_t responseSize)
{
    char msg[32];
    asynStatus status;
    size_t msgLen;
   
    size_t nbytesIn=0;
    size_t nbytesOut=0;
    int eomReason = 0;
    //printf("Called poll_request\n");
    msg[0] = 0x05;        // ENQ
    msg[1] = (unsigned char) (slave->addr+0x40);
    msgLen = 2;
    //errlogPrintf("Writing ENQ + poll address\n");
    status = drv->pasynOctet->write(drv->pasynInterface->drvPvt,drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
    if (status != asynSuccess) {
        errlogPrintf("poll_request: errore write comando\n");
        return -1;
    }
    epicsThreadSleep(0.05);
    memset(response, 0, responseSize);
    status = drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, response,responseSize-1,  &nbytesIn, &eomReason);
    int retVal=0;
    if ( (nbytesIn < 5)) 
    {
        if ((nbytesIn == 1) && ( (unsigned char)response[0]==0x4))
        {
            //errlogPrintf("Obtained EOT FIFO EMPTY for slave %d ",slave->addr);
            retVal= 1;
        }
        else
        { 
            errlogPrintf("poll_request: errore read risposta al ENQ+address %d\n",slave->addr);
            errlogPrintf("poll_request: read ricevuto, status=%d, nbytesIn=%zu, eomReason=%d\n", status, nbytesIn,eomReason);
            errlogPrintf("poll_request: read ricevuto byte 0x%02X\n", (unsigned char)response[0]);
            retVal = -1;
        }
    }
    //errlogPrintf("POLL %d :OBTAINED ANSWER: len %ld\n",slave->addr,strlen(response));
    
    if (retVal <0)
        return retVal;
    //REPLY ONLY IF NOT EMPTY
    if (retVal == 0)
    {
        msg[0]=0x06;
        status = drv->pasynOctet->write(drv->pasynInterface->drvPvt,drv->pasynUser, (const char*)msg, 1, &nbytesOut);
        if (status != asynSuccess) {
            errlogPrintf("poll_request: errore writing ack\n");
            return -1;
        }
        char meot[2];
        status =  drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, meot,1,  &nbytesIn, &eomReason);
        if (status != asynSuccess) {
            errlogPrintf("poll_request: errore read ACK risposta\n");
            errlogPrintf("poll_request: read ricevuto, status=%d, nbytesIn=%zu, eomReason=%d\n", status, nbytesIn,eomReason);
            errlogPrintf("poll_request: read ricevuto byte 0x%02X\n", (unsigned char)meot[0]);
            return -1;
        }
    }

    return retVal;

}

/* --- Thread di polling --- */
static void ocem_polling(void *arg) {
    
    char response[128];
    size_t responseSize=128;
    
    while(drv->running) 
    {
        for (int i=0; i < drv->nSlaves;i++)
        {
            //errlogPrintf("polling request on address %d\n",drv->addrList[i]);
            epicsMutexLock(drv->ioLock);
            int ret=poll_request(drv,&drv->slaves[drv->addrList[i]],response,responseSize);
            if ( ret == 0)
            {
                errlogPrintf1("From polling obtained %s (%ld)\n",response,responseSize);
                parseMultiReply(response);
            }
            else if (ret == 1)
            {
                errlogPrintf1("Obtained EOT FIFO EMPTY for slave %d Launching select ",drv->addrList[i]);
                select_request(drv,&drv->slaves[drv->addrList[i]],response,responseSize);
                epicsThreadSleep(0.2);

            }
            epicsMutexUnlock(drv->ioLock);

        }
        epicsThreadSleep(2.0); // periodo di polling
    }
}





/* --- Inizializzazione driver --- */
static void ocem_init(const char *port, int nSlaves, const char *addrListStr) {
    drv = calloc(1, sizeof(OCEM_Driver));
    drv->port = strdup(port);
    drv->nSlaves = nSlaves;
    drv->running = 1;
    int addrList[MAX_SLAVE];
    drv->ioLock = epicsMutexCreate();
    if (!drv->ioLock) {
        errlogPrintf("OCEM: failed to create ioLock mutex\n");
        return;
    }
 // srand(time(NULL));
    parseIntList(addrListStr,addrList, MAX_SLAVE);
    for(int i=0; i<nSlaves; i++) 
    {
        int addr = addrList[i];
        if (addr < 0 || addr >= MAX_SLAVE) 
        {
            printf("ocemInit: indirizzo %d fuori range (0..31)\n", addr);
            continue;
        }

        drv->addrList[i] = addr;
        drv->slaves[addr].addr = addr;
        
        
        scanIoInit(&drv->slaves[addr].ioscanStatus);
        scanIoInit(&drv->slaves[addr].ioscanCurrent);
        scanIoInit(&drv->slaves[addr].ioscanVoltage);
        scanIoInit(&drv->slaves[addr].ioscanPolarity);
         scanIoInit(&drv->slaves[addr].ioscanAlarms);
    }
    printf("Il driver presenta questi slave ID: ");
    for (int i=0; i <nSlaves;i++)
    {
        printf("%d ",drv->addrList[i]);
    }
    printf("\n");
    drv->pasynUser=pasynManager->createAsynUser(0, 0);
    asynStatus st = pasynManager->connectDevice(drv->pasynUser, drv->port, 0);
    if (st) 
    {
        errlogPrintf("OCEM: impossibile connettere asynUser\n");
        pasynManager->freeAsynUser(drv->pasynUser);
        drv->pasynUser = NULL;
        
    }
    drv->pasynInterface=pasynManager->findInterface(drv->pasynUser, asynOctetType, 1);
    drv->pasynOctet = (asynOctet *) drv->pasynInterface->pinterface;
    if (!drv->pasynOctet) 
    {
        errlogPrintf("ocem: asynOctet non disponibile\n");
        return ;
    }
    drv->pasynUser->timeout = 0.5;




    drv->threadId = epicsThreadCreate("ocemPoll",
                                      epicsThreadPriorityMedium,
                                      epicsThreadGetStackSize(epicsThreadStackMedium),
                                      ocem_polling, NULL);

   
}

/* --- IOC shell command --- */
static const iocshArg initArg0 = {"port", iocshArgString};
static const iocshArg initArg1 = {"nSlaves", iocshArgInt};
static const iocshArg initArg2 = {"addrListStr", iocshArgString};
static const iocshArg *initArgs[3] = {&initArg0, &initArg1,&initArg2};
static const iocshFuncDef initFuncDef = {"ocemInit", 3, initArgs};
static void initCall(const iocshArgBuf *args) {
    //printf("initCall ha ricevuto %s",args[2].sval)
    ocem_init(args[0].sval, args[1].ival, args[2].sval);
}
static void drvOCEMRegister(void) {
    iocshRegister(&initFuncDef, initCall);
}
epicsExportRegistrar(drvOCEMRegister);
