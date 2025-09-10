#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <devSup.h>
#include <stringinRecord.h>
#include <epicsExport.h>


#include "drvOCEM.h"

//extern IOSCANPVT ocem_getIOSCAN(int addr);
extern const char* ocem_getString(int addr);
//extern OCEM_Driver *drv;

typedef struct {
    int addr;        // slave address (0..31)
    char var[32];    // "STATUS", "CURRENT", "VOLTAGE", ...
} ocemDpvt;

OCEM_Driver *drv = NULL;   // non static!

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
    pvt->addr = addr; // uso l’indirizzo vero, non base-1
    strncpy(pvt->var, varname, sizeof(pvt->var));
    prec->dpvt = pvt;



    
    //prec->dpvt = (void*)(intptr_t)atoi(prec->inp.value.instio.string);





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
    else if (strcasecmp(p->var, "VOLTAGE") == 0)
        *ppvt = drv->slaves[p->addr].ioscanVoltage;



    
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
