

#include "drvOCEM.h"
#include <ctype.h>
const char* ocem_getString(int addr) {
    for (int i=0; i<drv->nSlaves; i++) {
        if (drv->slaves[i].addr == addr)
            return drv->slaves[i].status;
    }
    return NULL;
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


/* --- Thread di polling --- */
static void ocem_polling(void *arg) {
    int toggle = 0;
    errlogPrintf("IN OCEM POLLING\n");
    while(drv->running) 
    {
         for (int i = 0; i < drv->nSlaves; i++) 
        {
            int ad=drv->addrList[i];
            // Simula uno stato testuale che cambia A <-> B
            if (toggle)
                strncpy(drv->slaves[ad].status, "A", sizeof(drv->slaves[ad].status));
            else
                strncpy(drv->slaves[ad].status, "B", sizeof(drv->slaves[ad].status));

            drv->slaves[ad].status[sizeof(drv->slaves[ad].status)-1] = '\0';

            // Notifica i record collegati a questo slave
            scanIoRequest(drv->slaves[ad].ioscanStatus);
        }


        toggle = !toggle;
        epicsThreadSleep(1.0); // periodo di polling
    }
}





/* --- Inizializzazione driver --- */
static void ocem_init(const char *port, int nSlaves, const char *addrListStr) {
    drv = calloc(1, sizeof(OCEM_Driver));
    drv->port = strdup(port);
    drv->nSlaves = nSlaves;
    drv->running = 1;
    int addrList[MAX_SLAVE];
   
    parseIntList(addrListStr,addrList, MAX_SLAVE);
    for(int i=0; i<nSlaves; i++) 
    {
        int addr = addrList[i];//FANCULO
        if (addr < 0 || addr >= MAX_SLAVE) 
        {
            printf("ocemInit: indirizzo %d fuori range (0..31)\n", addr);
            continue;
        }

        drv->addrList[i] = addr;
        drv->slaves[addr].addr = addr;
        drv->slaves[i].value = 0.0;
        
        scanIoInit(&drv->slaves[addr].ioscanStatus);
        scanIoInit(&drv->slaves[addr].ioscanCurrent);
    }
    printf("Il driver presenta questi slave ID: ");
    for (int i=0; i <nSlaves;i++)
    {
        printf("%d ",drv->addrList[i]);
    }
    printf("\n");

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
