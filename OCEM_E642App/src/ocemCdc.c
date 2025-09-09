#include <stdio.h>
#include <string.h>
#include <subRecord.h>
#include <dbAccess.h>
#include <stdlib.h>
#include <iocsh.h>
#include <asynDriver.h>
#include <asynOctet.h>
#include <devSup.h>
#include <recGbl.h>
//#include <aiRecord.h>     // esempio: per un record ai
#include <stringinRecord.h>
#include <alarm.h>
#include <epicsExport.h>
#include <errlog.h>

#define MAX_CMD_LEN 128
#include <stdint.h>
#include <epicsMutex.h>
// Mutex per il singolo slave/porta
static epicsMutexId ocemSlaveMutex = NULL;
static asynUser *ocemSharedAsynUser = NULL;
#define OCEM_MAX_ENQ_RETRY 3
#define OCEM_ENQ_DELAY_MS 20


typedef struct {
    int address;
    char cmd[32];
    char port[32];
} ocemRecordPvt;

/* init_record */
static long init_record_si_devSiOCEM(stringinRecord *prec)
{
    // Se dpvt è già allocato, non facciamo nulla
    if (prec->dpvt) {
        printf("OCEM: init_record_ai_devAiOCEM dpvt già allocato\n");
        return 0;
    }

    printf("OCEM: init_record_si_devSiOCEM chiamata\n");

    // Controllo che INP sia di tipo INST_IO
    if (prec->inp.type != INST_IO) {
        recGblRecordError(S_db_badField, (void*)prec,
            "init_record_si_devSiOCEM: INP must be INST_IO");
        return S_db_badField;
    }

    const char *parm = prec->inp.value.instio.string;
    if (!parm || parm[0] == '\0') {
        recGblRecordError(S_db_badField, (void*)prec,
            "init_record_ai_devAiOCEM: INP string is empty");
        return S_db_badField;
    }

    // Alloca dpvt
    ocemRecordPvt *dpvt = (ocemRecordPvt*) calloc(1, sizeof(ocemRecordPvt));
    if (!dpvt) {
        recGblRecordError(S_db_noMemory, (void*)prec,
            "init_record_ai_devAiOCEM: cannot allocate dpvt");
        return S_db_noMemory;
    }

    // Parse stringa, esempio: "OCEM_PORT 106 STA"
    
    if (sscanf(parm, "%31s %d %31s",dpvt->port, &dpvt->address, dpvt->cmd) < 3) {
        recGblRecordError(S_db_badField, (void*)prec,
            "init_record_ai_devAiOCEM: cannot parse INP string");
        free(dpvt);
        return S_db_badField;
    }
    size_t len = strlen(dpvt->cmd);
    if (len + 2 < sizeof(dpvt->cmd)) {
        //dpvt->cmd[len]   = '\r';
        //dpvt->cmd[len+1] = '\n';
        //dpvt->cmd[len+2] = '\0';
    }
    printf("OCEM: init_record_si_devAiOCEM %s to address %x, port %s\n",dpvt->cmd,dpvt->address,dpvt->port);
    // Salva dpvt nel record
    prec->dpvt = dpvt;
    if (!ocemSlaveMutex) 
    {
        ocemSlaveMutex = epicsMutexCreate();
        if (!ocemSlaveMutex) {
            errlogPrintf("OCEM: impossibile creare mutex!\n");
            return -1;
        }
    
    }
    if (!ocemSharedAsynUser) 
    {
        ocemSharedAsynUser = pasynManager->createAsynUser(0, 0);
        asynStatus st = pasynManager->connectDevice(ocemSharedAsynUser, dpvt->port, 0);
        if (st) 
        {
            errlogPrintf("OCEM: impossibile connettere shared asynUser\n");
            pasynManager->freeAsynUser(ocemSharedAsynUser);
            ocemSharedAsynUser = NULL;
            return -1;
        }
    }
    return 0;
}


static long report_devAiOCEM(int pass) {  errlogPrintf("OCEM: report_devAiOCEM chiamata, pass=%d\n", pass);return 0; }
static long init_devAiOCEM(int pass) {  errlogPrintf("OCEM: init_devAiOCEM chiamata, pass=%d\n", pass);return 0; }


/* Calcolo CDC: XOR di tutti i byte tranne il primo (STX) */
static unsigned char ocem_calc_cdc(const unsigned char *buf, size_t cmdLen) {
    unsigned char cdc = 0;
    //errlogPrintf("Len command = %ld\n",cmdLen);
    if (cmdLen > 1) {
        for (size_t i = 1; i <= 2 + cmdLen; i++) {
            cdc ^= buf[i];
        }
        cdc |=0x80;
    }
    return cdc;
}

int parse_ocem_response(char *response, unsigned char address, const char *cmd) {
    
    unsigned char poll_address = address - 0x20;
    char *ptr = response;

    // controlla STX
    if ((unsigned char)ptr[0] != 0x02) {
        return -1; // non STX
    }
    ptr++; // salta STX

    // ora ptr[0] è il poll_address
    if ((unsigned char)ptr[0] != poll_address) {
        return -2; // address non corrisponde
    }
    ptr++; // salta poll_address

    // controllo comando
    if (strncmp(ptr, cmd, strlen(cmd)) != 0) {
        errlogPrintf("Errore parsing cmd expected: %s, received %s\n",cmd,ptr);
        return -3; // comando non corrisponde
    }
    ptr += strlen(cmd);

    // Sposta tutto il resto all’inizio della stringa
    size_t rest_len = strlen(ptr);
    memmove(response, ptr, rest_len);
    response[rest_len] = '\0';
    return 0; // tutto ok
}

/*
 * Esegue una transazione con l’OCEM:
 * 1) Invia ENQ + address
 * 2) Attende ACK (0x06) o NAK (0x15)
 * 3) Se ACK → invia STX + address + cmd + ETX + CDC
 * 4) Legge la risposta
 */
int ocem_transaction(asynUser *pasynUser,
                     int address,
                     const char *cmd,
                     char *response,
                     size_t responseSize)
{
    asynOctet *pOctet = NULL;
    asynInterface *pInterface = NULL;
    size_t nbytesOut = 0, nbytesIn = 0;
    int eomReason = 0;
    asynStatus status;
    int attempts = 0;
    unsigned char msg[256];
    size_t msgLen;
    //errlogPrintf("Into ocem_transaction pasynUser=%p\n", pasynUser);
    /* 1. Ottieni interfaccia Octet */
    pInterface=pasynManager->findInterface(pasynUser, asynOctetType, 1);
    pOctet = (asynOctet *) pInterface->pinterface;
    if (!pOctet) 
    {
        errlogPrintf("ocem_transaction: asynOctet non disponibile\n");
        return -1;
    }
    //errlogPrintf("pOctet =%p\n",pOctet);
    //errlogPrintf("pInterface->drvPvt =%p\n",pInterface->drvPvt);
    
    /* 2. Invia ENQ + address */
    msg[0] = 0x05;        // ENQ
    msg[1] = (unsigned char) address;
    msgLen = 2;
    
    
   
    /* status = pOctet->flush(pInterface->drvPvt, pasynUser);
    if (status != asynSuccess) {
        errlogPrintf("OCEM: flush failed\n");
    }
    epicsThreadSleep(0.05);
 */
    pasynUser->timeout = 0.5;
    while (attempts < OCEM_MAX_ENQ_RETRY) 
    {
        //errlogPrintf("Writing ENQ + address %c\n",msg[1]);
        status = pOctet->write(pInterface->drvPvt, pasynUser, (const char*)msg, msgLen, &nbytesOut);
        if (status != asynSuccess) {
            errlogPrintf("ocem_transaction: errore write ENQ\n");
            return -1;
        }
        epicsThreadSleep(0.05);
        /* 3. Leggi ACK/NAK */
        unsigned char ackBuf[1];
        //errlogPrintf("Reading ENQ + address answer\n");
        status = pOctet->read(pInterface->drvPvt,pasynUser, (char*)ackBuf, 1,  &nbytesIn, &eomReason);
        if (status != asynSuccess || nbytesIn == 0) 
        {
            
            attempts++;
            errlogPrintf("ocem_transaction: timeout o errore in attesa di ACK/NAK. Retrying %d (%ld)\n",attempts,nbytesIn);
            if (attempts == OCEM_MAX_ENQ_RETRY )
                return -1;
            epicsThreadSleep(OCEM_ENQ_DELAY_MS / 1000.0);
            continue;
        }
        else
        {
           if (ackBuf[0] != 0x06) 
           { // 0x06 = ACK
                errlogPrintf("ocem_transaction: ricevuto NAK (0x%02X)\n", ackBuf[0]);
                return -1;
            }
            break;
        }
    }
    //errlogPrintf("ocem_transaction: ricevuto ACK (0x%02X)\n", ackBuf[0]);

    /* 4. Prepara STX + addr + cmd + ETX + CDC */
    size_t cmdLen = strlen(cmd);
    msg[0] = 0x02; // STX
    msg[1] = (unsigned char) address;
    memcpy(&msg[2], cmd, cmdLen);
    msg[2 + cmdLen] = 0x03; // ETX
    unsigned char cdc = ocem_calc_cdc(msg, cmdLen); 
    msg[3 + cmdLen] = cdc;
    msgLen = 4 + cmdLen;

    /* 5. Scrivi comando completo */
    // Debug: dump dei byte da inviare
    /*errlogPrintf("OCEM: sending message (len=%zu): ", msgLen);
    for (size_t i = 0; i < msgLen; i++) {
        errlogPrintf("%02X ", (unsigned char)msg[i]);
    }
    errlogPrintf("\n");*/
    
    status = pOctet->write(pInterface->drvPvt,pasynUser, (const char*)msg, msgLen, &nbytesOut);
    
    if (status != asynSuccess) {
        errlogPrintf("ocem_transaction: errore write comando\n");
        return -1;
    }
    epicsThreadSleep(0.05);
    /* 6. Leggi risposta */
    
    //errlogPrintf("Reading answer\n");

    memset(response, 0, responseSize);
    status = pOctet->read(pInterface->drvPvt,pasynUser, response,1,  &nbytesIn, &eomReason);
    if (status != asynSuccess) {
        errlogPrintf("ocem_transaction: errore read risposta\n");
        errlogPrintf("ocem_transaction: read ricevuto, status=%d, nbytesIn=%zu, eomReason=%d\n", status, nbytesIn,eomReason);
        errlogPrintf("ocem_transaction: read ricevuto byte 0x%02X\n", (unsigned char)response[0]);
        return -1;
    }
    //poll request
    msg[0] = 0x05;        // ENQ
    msg[1] = (unsigned char) (address-0x20);
    msgLen = 2;
    //errlogPrintf("Writing ENQ + poll address\n");
    status = pOctet->write(pInterface->drvPvt,pasynUser, (const char*)msg, msgLen, &nbytesOut);
    if (status != asynSuccess) {
        errlogPrintf("ocem_transaction: errore write comando\n");
        return -1;
    }
    epicsThreadSleep(0.05);
    memset(response, 0, responseSize);
    status = pOctet->read(pInterface->drvPvt,pasynUser, response,responseSize-1,  &nbytesIn, &eomReason);
    if ( (nbytesIn < 5)) {
        errlogPrintf("ocem_transaction: errore read risposta al poll\n");
        errlogPrintf("ocem_transaction: read ricevuto, status=%d, nbytesIn=%zu, eomReason=%d\n", status, nbytesIn,eomReason);
        errlogPrintf("ocem_transaction: read ricevuto byte 0x%02X\n", (unsigned char)response[0]);
        return -1;
    }
    // errlogPrintf("OCEM: receving message (len=%zu): ", nbytesIn);
    // for (size_t i = 0; i < nbytesIn; i++) {
    //     errlogPrintf("%02X ", (unsigned char)response[i]);
    // }
    // errlogPrintf("\n");
    response[nbytesIn-4] = '\0';
    errlogPrintf("Read answer %s\n",response);
    int rs=parse_ocem_response(response,address,cmd);
    

    
    msg[0]=0x06;
    status = pOctet->write(pInterface->drvPvt,pasynUser, (const char*)msg, 1, &nbytesOut);
    if (status != asynSuccess) {
        errlogPrintf("ocem_transaction: errore writing ack\n");
        return -1;
    }
    char meot[2];
    status = pOctet->read(pInterface->drvPvt,pasynUser, meot,1,  &nbytesIn, &eomReason);
    if (status != asynSuccess) {
        errlogPrintf("ocem_transaction: errore read risposta\n");
        errlogPrintf("ocem_transaction: read ricevuto, status=%d, nbytesIn=%zu, eomReason=%d\n", status, nbytesIn,eomReason);
        errlogPrintf("ocem_transaction: read ricevuto byte 0x%02X\n", (unsigned char)meot[0]);
        return -1;
    }
    if ( rs !=0)
    {
        errlogPrintf("ocem_transaction: errore parsing %d\n",rs);
        return -1;
    }


       
    return 0;
}

/* read_ai */
static long read_si_devSiOCEM(stringinRecord *prec)
{
    if (!prec->dpvt) 
    {
        long ret=init_record_si_devSiOCEM(prec);
        if (ret != 0) 
        {
            recGblSetSevr(prec, READ_ALARM, INVALID_ALARM);
            return -1;
        }
        errlogPrintf("returning from init_record\n");
    } 

    ocemRecordPvt *dpvt = (ocemRecordPvt*)prec->dpvt;
    if (!dpvt) {
        errlogPrintf("OCEM %s: dpvt NULL (init_record failed)\n", prec->name);
        recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
        return 2; // niente conversione, evita crash
    }
    //errlogPrintf("assigned *dpvt\n");

   /*  asynUser *pau = pasynManager->createAsynUser(0,0);
    asynStatus st = pasynManager->connectDevice(pau, dpvt->port, 0);
    
    if (st) {
        errlogPrintf("OCEM %s: connectDevice failed (%d)\n", prec->name, st);
        pasynManager->freeAsynUser(pau);
        recGblSetSevr(prec, COMM_ALARM, INVALID_ALARM);
        return 2;
    } */
    //errlogPrintf("prima di ocem_transaction \npasynUser=%p\n", pau);
    char resp[256] = {0};
    // Acquisisci lock prima della transazione
    epicsMutexMustLock(ocemSlaveMutex);
    int ret = ocem_transaction(ocemSharedAsynUser, dpvt->address, dpvt->cmd, resp, sizeof(resp));
    // Rilascia lock
    epicsMutexUnlock(ocemSlaveMutex);
    //errlogPrintf("dopo ocem_transaction \n");
    //pasynManager->freeAsynUser(pau);

    if (ret == 0) {
        errlogPrintf("OCEM %s: resp='%s'\n", prec->name, resp);
        
        strncpy(prec->val, resp, sizeof(prec->val)-1);
        prec->val[sizeof(prec->val)-1] = '\0';
        prec->udf = 0;
        return 2;                   // VAL già impostato
    } else {
        errlogPrintf("OCEM %s: transaction failed\n", prec->name);
        recGblSetSevr(prec, READ_ALARM, INVALID_ALARM);
        return 2;
    }
}





/* Tabella device support */


struct {
    long            num;
    DEVSUPFUN       report;
    DEVSUPFUN       init;
    DEVSUPFUN       init_record;
    DEVSUPFUN       get_ioint_info;
    DEVSUPFUN       read_si;
} devSiOCEM  = {
    5,
    report_devAiOCEM,
    init_devAiOCEM,
     (DEVSUPFUN) init_record_si_devSiOCEM,
    NULL,
    (DEVSUPFUN) read_si_devSiOCEM
};

epicsExportAddress(dset, devSiOCEM );



/* long cdcSubroutine(subRecord *precord) {
    char cmd[MAX_STRING_SIZE] = {0};
    char param[MAX_STRING_SIZE] = {0};
    char output[MAX_STRING_SIZE] = {0};
    char addressStr[32];

    // --- Address da FTA (campo a, LONG) ---
    long address = (long) precord->a;
    snprintf(addressStr, sizeof(addressStr), "%ld", address);

    // --- Legge input string da INPB / INPC ---
    dbGetLink(&precord->inpb, DBR_STRING, cmd, 0, 0);
    dbGetLink(&precord->inpc, DBR_STRING, param, 0, 0);

   

    // --- Costruzione stringa ---
    snprintf(output, MAX_STRING_SIZE, "\x02%s%s%s\x03", addressStr, cmd, param);
    unsigned char cdc = 0;
    for (size_t i = 1; i < strlen(output); i++) {  // escluso il primo byte
        cdc ^= (unsigned char)output[i];
        }
    size_t len = strlen(output);
    if (len + 1 < MAX_STRING_SIZE) {
    output[len] = cdc;
}
    // --- Scrive output su VALA (link a stringout) ---
    dbPutLink(&precord->flnk, DBR_STRING, output, 1);

    return 0;
}
 */