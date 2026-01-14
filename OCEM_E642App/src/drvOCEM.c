

#include "drvOCEM.h"
#include <ctype.h>
#include <time.h>

// Global debug level - can be set from IOC shell
int ocemDebugLevel = 0;
epicsExportAddress(int, ocemDebugLevel);

extern ocemDpvt *ocem_records[MAX_OCEM_RECORDS];
//double ocemPollingPeriod = 0.5;   // default 1 second
//epicsExportAddress(double, ocemPollingPeriod);
// 777 : x = 65535 : 380 

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

int parsePRGAnswer(const char*answer,char*chan,int* minval,int*maxval)
{
    char readChan[3];
    if (strlen(answer) < 2)
        return -1;
    strncpy(readChan,answer,2);
    readChan[2]='\0';
    if (strcmp(readChan,chan))
    {
        return -1;
    }
    char minimum[7],maximum[7];
    for (int i=3;i <10;i++)
    {
        minimum[i-3]=answer[i];
    }
    *minval=atoi(minimum);
    
    for (int i=11;i <18;i++)
    {
        maximum[i-11]=answer[i];
    }
    *maxval=atoi(maximum);
    return 0;


}

// Helper to verify if command was applied based on status
static void verifyCommandApplied(OCEM_Slave *slave)
{
    if (slave->cmdState != CMD_VERIFYING)
        return;
    
    slave->cmdVerifyCount++;
    
    // Check if the last command matches the expected status
    int verified = 0;
    
    if (strcmp(slave->lastCommand, "ON") == 0) {
        if (strcmp(slave->status, "ATT") == 0) {
            verified = 1;
        }
    }
    else if (strcmp(slave->lastCommand, "STB") == 0) {
        if (strcmp(slave->status, "STB") == 0) {
            verified = 1;
        }
    }
    else if (strcmp(slave->lastCommand, "POS") == 0) {
        if (strcmp(slave->polarity, "POS") == 0) {
            verified = 1;
        }
    }
    else if (strcmp(slave->lastCommand, "NEG") == 0) {
        if (strcmp(slave->polarity, "NEG") == 0) {
            verified = 1;
        }
    }
    else if (strcmp(slave->lastCommand, "OPN") == 0) {
        if (strcmp(slave->polarity, "OPN") == 0) {
            verified = 1;
        }
    }
    else if (strncmp(slave->lastCommand, "SP ", 3) == 0 ||
             strcmp(slave->lastCommand, "STR") == 0 ||
             strcmp(slave->lastCommand, "RMT") == 0 ||
             strcmp(slave->lastCommand, "RES") == 0 ||
             strncmp(slave->lastCommand, "PRG", 3) == 0) {
        // These commands don't have direct status verification, assume OK after send
        verified = 1;
    }
    
    if (verified) {
        slave->cmdState = CMD_DONE;
        snprintf(slave->cmdStatusMsg, sizeof(slave->cmdStatusMsg), "Done: %s", slave->lastCommand);
        OCEM_INFO("[PS%d] Command verified: %s -> DONE\n", slave->addr, slave->lastCommand);
        scanIoRequest(slave->ioscanCmdState);
    }
    else if (slave->cmdVerifyCount >= 10) {
        // After 10 polls, mark as NOT_REACHED
        slave->cmdState = CMD_NOT_REACHED;
        snprintf(slave->cmdStatusMsg, sizeof(slave->cmdStatusMsg), "NOT REACHED: %s (status=%s)", 
                 slave->lastCommand, slave->status);
        OCEM_ERR("[PS%d] Command NOT REACHED: %s (current status=%s)\n", 
                 slave->addr, slave->lastCommand, slave->status);
        scanIoRequest(slave->ioscanCmdState);
    }
}

void ActivateInterrupt(int slaveId,char* cmd, char* val)
{
            // trova slave corrispondente
    OCEM_Slave *slave = findSlave(drv, slaveId);
    if (!slave) 
    {
        OCEM_ERR("ActivateInterrupt: Unknown slave addr=%d (cmd=%s, val=%s)\n", slaveId, cmd, val);
        return;
    }
    
    OCEM_DETAIL("[PS%d] Update: %s = '%s'\n", slaveId, cmd, val);
    
    if (strcmp(cmd, "STA") == 0) 
    {
        strncpy(slave->status, val, sizeof(slave->status));
        slave->status[sizeof(slave->status)-1] = '\0';

        if (strcmp(val, "ATT") == 0)
            slave->unimagStatus = 1;
        else if (strcmp(val, "STB") == 0)
            slave->unimagStatus = 2;
        else
            slave->unimagStatus = 5;

        if (strncmp(slave->alarms,"NO",2))
        {
            slave->unimagStatus = 4;
        }
        scanIoRequest(slave->ioscanStatus);
        
        // Check if this status update verifies a pending command
        verifyCommandApplied(slave);
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
        if (strcasecmp(val,"NEG") == 0) slave->integerPolarity =-1;
        else if (strcasecmp(val,"POS") == 0) slave->integerPolarity =1;
        else if (strcasecmp(val,"OPN") == 0) slave->integerPolarity =0;
        scanIoRequest(slave->ioscanPolarity);
        
        // Check if this polarity update verifies a pending command
        verifyCommandApplied(slave);
    }
    else if (strcmp(cmd, "ALL") == 0) 
    {
        
        strncpy(slave->alarms, val, sizeof(slave->alarms));
        slave->alarms[sizeof(slave->alarms)-1] = '\0';
        scanIoRequest(slave->ioscanAlarms);
        
    }
    else if (strcmp(cmd, "SEL") == 0) 
    {
        if (!strcmp(val,"PRE"))
        {
            strncpy(slave->selector, "REMOTE", sizeof(slave->selector));
        }
        else
        {
            strncpy(slave->selector, val, sizeof(slave->selector));
        }
        slave->selector[sizeof(slave->selector)-1] = '\0';
        scanIoRequest(slave->ioscanSelector);
    }
    else  if (strcmp(cmd, "PRG") == 0) 
    {
        int minvalue;int maxvalue;
        if (parsePRGAnswer(val,"O0",&minvalue,&maxvalue) == 0)
        {
            slave->currentPrgL=minvalue;
            slave->currentPrgH=maxvalue;
            scanIoRequest(slave->ioscanInit);
            errlogPrintf("CURRENT PRG: min : %d max %d\n",minvalue,maxvalue);
        }
        else if (parsePRGAnswer(val,"O1",&minvalue,&maxvalue) == 0)
        {
            slave->voltagePrgL=minvalue;
            slave->voltagePrgH=maxvalue;
            scanIoRequest(slave->ioscanInit);
            errlogPrintf("VOLTAGE PRG:  min : %d max %d\n",minvalue,maxvalue);
        }
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
        OCEM_ERR("parseReplyString: null pointer argument\n");
        return -1;
    }

    int i = 0;
    *slaveId = -1;
    
    OCEM_TRACE("parseReplyString: input='%s' (len=%zu)\n", recStr, strlen(recStr));
    
    // Check for poll address format: 0x40 + addr
    // '@' (0x40) = addr 0, 'A' (0x41) = addr 1, 'B' (0x42) = addr 2, etc.
    unsigned char firstChar = (unsigned char)recStr[i];
    
    OCEM_TRACE("parseReplyString: firstChar=0x%02X '%c'\n", firstChar, 
               (firstChar >= 0x20 && firstChar < 0x7F) ? firstChar : '?');
    
    if (firstChar >= 0x40 && firstChar <= 0x5F) {
        // Poll address format: '@'=0, 'A'=1, 'B'=2, ..., 'O'=15, etc.
        *slaveId = firstChar - 0x40;
        OCEM_TRACE("parseReplyString: poll address format, slaveId=%d\n", *slaveId);
        i++;
    }
    // Check for select address format: 0x60 + addr
    // '`' (0x60) = addr 0, 'a' (0x61) = addr 1, 'b' (0x62) = addr 2, etc.
    else if (firstChar >= 0x60 && firstChar <= 0x7F) {
        *slaveId = firstChar - 0x60;
        OCEM_TRACE("parseReplyString: select address format, slaveId=%d\n", *slaveId);
        i++;
    }
    else if (isdigit(firstChar)) {
        OCEM_DETAIL("parseReplyString: numeric address starting with '%c'\n", firstChar);
        // Caso indirizzo numerico (es: "11COR 3.0")
        *slaveId = 0;
        while (isdigit((unsigned char)recStr[i])) {
            *slaveId = (*slaveId * 10) + (recStr[i] - '0');
            i++;
        }
        OCEM_TRACE("parseReplyString: numeric address, slaveId=%d\n", *slaveId);
    }
    else
    {
        OCEM_ERR("parseReplyString: unexpected char 0x%02X at pos 0 in '%s'\n", firstChar, recStr);
        // Try to dump the raw bytes for debugging
        OCEM_ERR("parseReplyString: raw bytes: ");
        for (int k = 0; k < 10 && recStr[k]; k++) {
            OCEM_ERR("%02X ", (unsigned char)recStr[k]);
        }
        OCEM_ERR("\n");
        return -1;
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
    
    OCEM_TRACE("parseMultiReply: raw input (len=%zu): ", strlen(input));
    if (ocemDebugLevel >= 3) {
        for (size_t k = 0; k < strlen(input) && k < 40; k++) {
            OCEM_TRACE("%02X ", (unsigned char)input[k]);
        }
        OCEM_TRACE("\n");
    }
    
    for (i = 1; i < strlen(input) - 2; ++i)
    {
        cleaned[i - 1] = input[i];
    }
    cleaned[i-1] = '\0';
    
    OCEM_DETAIL("parseMultiReply: cleaned='%s'\n", cleaned);
    
    const char *ptr = cleaned;
    char lastSlaveChar = '@';  // <-- default to address 0 (0x40 = '@')
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
        
        // Save address char if it's a valid poll address (0x40-0x5F) or select address (0x60-0x7F)
        unsigned char fc = (unsigned char)fullLine[0];
        if ((fc >= 0x40 && fc <= 0x5F) || (fc >= 0x60 && fc <= 0x7F))
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
    
    OCEM_DETAIL("[PS%d] SELECT: sending ENQ+0x%02X\n", slave->addr, slave->addr + 0x60);
    
    msg[0] = 0x05;        // ENQ
    msg[1] = (unsigned char) (slave->addr+0x60);
    msgLen = 2;
    
    status = drv->pasynOctet->write(drv->pasynInterface->drvPvt, drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
    if (status != asynSuccess) 
    {
        OCEM_ERR("[PS%d] SELECT: ENQ write error\n", slave->addr);
        return -1;
    }
    epicsThreadSleep(0.02);
    /* 3. Leggi ACK/NAK */
    unsigned char ackBuf[1];
    status = drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, (char*)ackBuf, 1,  &nbytesIn, &eomReason);
    if (status != asynSuccess || nbytesIn == 0) 
    {
        OCEM_INFO("[PS%d] SELECT: timeout waiting for ACK\n", slave->addr);
        return -1;
    }
    else
    {
        if (ackBuf[0] != 0x06) 
        { // 0x06 = ACK
            OCEM_INFO("[PS%d] SELECT: got NAK (0x%02X)\n", slave->addr, ackBuf[0]);
            return -1;
        }
        OCEM_DETAIL("[PS%d] SELECT: got ACK\n", slave->addr);
        
    }
    /* 4. Prepara STX + addr + cmd + ETX + CDC */
    char *cmd=getNextCommandForSlave(slave);
    strcpy(slave->lastSelCommand,cmd);
    size_t cmdLen = strlen(cmd);
    msg[0] = 0x02; // STX
    msg[1] = (unsigned char) (slave->addr+0x60);
    memcpy(&msg[2], cmd, cmdLen);
    msg[2 + cmdLen] = 0x03; // ETX
    unsigned char cdc = ocem_calc_cdc(msg, cmdLen); 
    msg[3 + cmdLen] = cdc;
    msgLen = 4 + cmdLen;
    
    OCEM_DETAIL("[PS%d] SELECT: sending cmd='%s'\n", slave->addr, cmd);
    
    status = drv->pasynOctet->write(drv->pasynInterface->drvPvt,drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
    
    if (status != asynSuccess) {
        OCEM_ERR("[PS%d] SELECT: cmd write error\n", slave->addr);
        return -1;
    }
    epicsThreadSleep(0.02);
    //Leggi l'ack
    memset(response, 0, responseSize);
    status = drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, response,1,  &nbytesIn, &eomReason);
    if (status != asynSuccess) {
        OCEM_INFO("[PS%d] SELECT: no ACK for cmd '%s'\n", slave->addr, cmd);
        return -1;
    }
    
    OCEM_DETAIL("[PS%d] SELECT: cmd '%s' done\n", slave->addr, cmd);
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
    
    OCEM_DETAIL("[PS%d] POLL: sending ENQ+0x%02X\n", slave->addr, slave->addr + 0x40);
    
    msg[0] = 0x05;        // ENQ
    msg[1] = (unsigned char) (slave->addr+0x40);
    msgLen = 2;
    
    status = drv->pasynOctet->write(drv->pasynInterface->drvPvt,drv->pasynUser, (const char*)msg, msgLen, &nbytesOut);
    if (status != asynSuccess) {
        OCEM_ERR("[PS%d] POLL: write error\n", slave->addr);
        return -1;
    }
    epicsThreadSleep(0.02);
    memset(response, 0, responseSize);
    status = drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, response,responseSize-1,  &nbytesIn, &eomReason);
    int retVal=0;
    if ( (nbytesIn < 5)) 
    {
        if ((nbytesIn == 1) && ( (unsigned char)response[0]==0x4))
        {
            OCEM_DETAIL("[PS%d] POLL: EOT (FIFO empty)\n", slave->addr);
            retVal= 1;
        }
        else
        { 
            OCEM_INFO("[PS%d] POLL: short response, nbytes=%zu, byte=0x%02X\n", 
                     slave->addr, nbytesIn, (unsigned char)response[0]);
            retVal = -1;
        }
    }
    else
    {
        OCEM_DETAIL("[PS%d] POLL: got %zu bytes\n", slave->addr, nbytesIn);
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
            OCEM_ERR("[PS%d] POLL: ACK write error\n", slave->addr);
            return -1;
        }
        char meot[2];
        status =  drv->pasynOctet->read(drv->pasynInterface->drvPvt,drv->pasynUser, meot,1,  &nbytesIn, &eomReason);
        if (status != asynSuccess) {
            OCEM_INFO("[PS%d] POLL: EOT read error\n", slave->addr);
            return -1;
        }
    }

    return retVal;

}
double bitInAmpere(int bits,OCEM_Slave *slave)
{
   
    return (bits * slave->IMAX)/ slave->currentPrgH;

}

int ampereInBits(double ampere,OCEM_Slave *slave)
{
   //A= (b*IMAX)/currh 
   ampere=fabs(ampere);
   return (int) ((slave->currentPrgH * ampere )/ slave->IMAX);

}

void checkCurrentSetStatus(OCEM_Slave* slave)
{
    char response[128];
    size_t responseSize=128;
    if (slave->IMAX <= 0)
    {
        for (int i = 0; i < MAX_OCEM_RECORDS; i++)
        {
            ocemDpvt *p = ocem_records[i];
            if (!p) continue;

            if (p->linkedAddr)
            {
            double value = 0;
                long status=0;
                long options = 0;
                long nreq = 1;

                status = dbGetField(p->linkedAddr, DBR_DOUBLE, &value,&options,&nreq,NULL);
                
                if (status == 0)
                {
                    
                    slave->IMAX=value;
                    printf( "IMAX :%f for address %d\n",slave->IMAX, slave->addr);
                    
                }

            }
        }
    }
    switch(slave->Ostate)
    {

        case STATE_IDLE:
            break;

        case STATE_REQ_SET_CURRENT:
            if (slave->currentPrgH == 0)
            {
                errlogPrintf("Richiesta di set di movimentazione corrente Ignorata: Non conosco ancora il fattore di scala\n");
                slave->Ostate = STATE_IDLE;
            }
            if (!strcmp(slave->status,"STB"))
            {
                // non fare nulla, richiesta ignorata
                errlogPrintf("Richiesta di set di movimentazione corrente, in stato STB\nIgnorata\n");
                slave->Ostate = STATE_IDLE;
            }
            else if (slave->integerPolarity != slave->requestedPolarity) {
                printf("serve cambio polarità\n");
                send_command(drv,slave->addr,"SP 0000000",response,responseSize);
                epicsThreadSleep(0.1);
                send_command(drv,slave->addr,"STR",response,responseSize);
                slave->Ostate = STATE_WAIT_ZERO;
            } else {
                // stessa polarità → vai diretto
                printf("Non serve cambio polarità\n");
                
                
                //send_command("GO");
                slave->Ostate = STATE_SET_ON;
            }
            break;

        case STATE_WAIT_ZERO:
            int curVal;
            sscanf(slave->current,"%d",&curVal);
            double actualCurrent=bitInAmpere(curVal,slave);

            
            if (actualCurrent < 5.0) 
            {
                //send_command("STB");
                printf("Abbassata corrente, metto in STB\n");
                send_command(drv,slave->addr,"STB",response,responseSize);
                slave->Ostate = STATE_SET_STANDBY;
            }
            break;

        case STATE_SET_STANDBY:
            if (!strcmp(slave->status,"STB"))
            {
                printf("Switching polarity\n");
                char* cmd=slave->requestedPolarity > 0 ? "POS" : "NEG";
                send_command(drv,slave->addr,cmd,response,responseSize);
                slave->Ostate = STATE_SET_POLARITY;
            }
            break;

        case STATE_SET_POLARITY:
            if (slave->integerPolarity == slave->requestedPolarity) 
            {
                printf("Sending Operational Again\n");
                send_command(drv,slave->addr,"ON",response,responseSize);
                //send_command("SETI %f", fabs(slave->requestedCurrent));
                slave->Ostate = STATE_SET_ON;
            }
            break;
        case STATE_SET_ON:
        {
            if (!strcmp(slave->status,"ATT"))
            {
                
                int curBits=ampereInBits(slave->requestedCurrent,slave);
                char bitStr[7],outStr[7],cmd[11];
                snprintf(bitStr,sizeof(bitStr),"%d",curBits);
                pad_value(bitStr,outStr);
                snprintf(cmd,sizeof(cmd),"SP %s",outStr);
                printf("Sending Command %s\n",cmd);
                send_command(drv,slave->addr,cmd,response,responseSize);
                slave->Ostate = STATE_SET_NEW_CURRENT;
            }
            break;

        }
        case STATE_SET_NEW_CURRENT:
            // appena SETI è accettato
            //send_command("GO");
            printf("Sending Start Ramp\n");
            send_command(drv,slave->addr,"STR",response,responseSize);
            slave->Ostate = STATE_RAMP_TO_TARGET;
            break;

        case STATE_RAMP_TO_TARGET:
            //Non necessario.. per ora.
            //printf("Checking current set achieved\n");
            //if (fabs(slave->currentRB - fabs(slave->requestedCurrent)) < threshold) {
                slave->Ostate = STATE_IDLE;
            //}
            break; 

        
    }




}

/* --- Command Queue Functions --- */
int queue_command(OCEM_Driver* drv, int slaveAddr, const char* cmd)
{
    CmdQueueEntry *entry = calloc(1, sizeof(CmdQueueEntry));
    if (!entry) return -1;
    
    entry->slaveAddr = slaveAddr;
    strncpy(entry->cmd, cmd, sizeof(entry->cmd) - 1);
    entry->next = NULL;
    
    // Mark slave as having pending command
    OCEM_Slave *slave = findSlave(drv, slaveAddr);
    if (slave) {
        slave->hasPendingCommand = 1;
        epicsTimeGetCurrent(&slave->lastCommandTime);
    }
    
    epicsMutexLock(drv->cmdQueueLock);
    if (drv->cmdQueueTail) {
        drv->cmdQueueTail->next = entry;
    } else {
        drv->cmdQueueHead = entry;
    }
    drv->cmdQueueTail = entry;
    epicsMutexUnlock(drv->cmdQueueLock);
    
    return 0;
}

static CmdQueueEntry* dequeue_command(OCEM_Driver* drv)
{
    epicsMutexLock(drv->cmdQueueLock);
    CmdQueueEntry *entry = drv->cmdQueueHead;
    if (entry) {
        drv->cmdQueueHead = entry->next;
        if (!drv->cmdQueueHead) {
            drv->cmdQueueTail = NULL;
        }
    }
    epicsMutexUnlock(drv->cmdQueueLock);
    return entry;
}

static void process_queued_commands(OCEM_Driver* drv, char* response, size_t responseSize)
{
    CmdQueueEntry *entry;
    int maxCmdsPerCycle = 4; // Process up to 4 commands per polling cycle
    int cmdCount = 0;
    
    while ((entry = dequeue_command(drv)) != NULL && cmdCount < maxCmdsPerCycle) {
        errlogPrintf1("Processing queued command: addr=%d cmd=%s\n", entry->slaveAddr, entry->cmd);
        
        // Update last command time for the slave
        OCEM_Slave *slave = findSlave(drv, entry->slaveAddr);
        if (slave) {
            epicsTimeGetCurrent(&slave->lastCommandTime);
        }
        
        send_command(drv, entry->slaveAddr, entry->cmd, response, responseSize);
        free(entry);
        cmdCount++;
        epicsThreadSleep(0.01); // Small delay between commands
    }
}

// Check if any slave has active commands (for determining polling rate)
static int hasActiveCommands(OCEM_Driver* drv)
{
    epicsTimeStamp now;
    epicsTimeGetCurrent(&now);
    
    for (int i = 0; i < drv->nSlaves; i++) {
        OCEM_Slave *slave = &drv->slaves[drv->addrList[i]];
        
        // Check if slave has pending command or is in a non-idle state
        if (slave->Ostate != STATE_IDLE) {
            return 1;
        }
        
        // Check if recent command was sent (within activeTimeout)
        if (slave->hasPendingCommand) {
            double elapsed = epicsTimeDiffInSeconds(&now, &slave->lastCommandTime);
            if (elapsed < drv->commandActiveTimeout) {
                return 1;
            } else {
                slave->hasPendingCommand = 0; // Clear flag after timeout
            }
        }
    }
    
    // Also check if there are queued commands
    epicsMutexLock(drv->cmdQueueLock);
    int hasQueued = (drv->cmdQueueHead != NULL);
    epicsMutexUnlock(drv->cmdQueueLock);
    
    return hasQueued;
}

// Helper function to update command state and notify records
static void setCmdState(OCEM_Slave *slave, CmdState newState, const char *statusMsg)
{
    slave->cmdState = newState;
    strncpy(slave->cmdStatusMsg, statusMsg, sizeof(slave->cmdStatusMsg) - 1);
    slave->cmdStatusMsg[sizeof(slave->cmdStatusMsg) - 1] = '\0';
    OCEM_INFO("[PS%d] CmdState: %d (%s)\n", slave->addr, newState, statusMsg);
    scanIoRequest(slave->ioscanCmdState);
}

// Get state name for display
static const char* getCmdStateName(CmdState state)
{
    switch(state) {
        case CMD_IDLE:        return "IDLE";
        case CMD_SENDING:     return "SENDING";
        case CMD_WAIT_ACK:    return "WAIT_ACK";
        case CMD_VERIFYING:   return "VERIFYING";
        case CMD_DONE:        return "DONE";
        case CMD_NOT_REACHED: return "NOT_REACHED";
        default:              return "UNKNOWN";
    }
}

// ============================================
// UNIMAG STATE MACHINE IMPLEMENTATION
// ============================================

const char* unimag_getStateName(UnimagState state)
{
    switch(state) {
        case UNIMAG_OK:          return "OK";
        case UNIMAG_NOT_REACHED: return "NOT_REACHED";
        case UNIMAG_ZERO_STBY:   return "ZERO_STBY";
        case UNIMAG_CHANGE_POL:  return "CHANGE_POL";
        case UNIMAG_GOING_TO_SET: return "GOING_TO_SET";
        default:                 return "UNKNOWN";
    }
}

const char* unimag_getChannelStateName(ChannelState state)
{
    switch(state) {
        case CH_OFF:     return "OFF";
        case CH_ON:      return "ON";
        case CH_STANDBY: return "STANDBY";
        case CH_FAULT:   return "FAULT";
        case CH_RESET:   return "RESET";
        default:         return "UNKNOWN";
    }
}

// Initialize UNIMAG state machine for a slave
void unimag_init(OCEM_Slave *slave)
{
    // Default configuration
    slave->unimagCfg.setTolerance = 1.0;    // 1A tolerance
    slave->unimagCfg.zeroTolerance = 1.0;   // 1A for zero
    slave->unimagCfg.setTimeoutS = 10.0;    // 10 second timeout
    slave->unimagCfg.maxRetries = 3;        // 3 retries
    slave->unimagCfg.retryDelay = 1.0;      // 1s between retries
    
    // Initial state
    slave->unimag.state = UNIMAG_OK;
    slave->unimag.busy = 0;
    slave->unimag.retryCount = 0;
    slave->unimag.targetCurrent = 0.0;
    slave->unimag.targetPolarity = 0;
    slave->unimag.targetState = CH_OFF;
    strcpy(slave->unimag.statusMsg, "Ready");
    
    slave->channelState = CH_OFF;
    slave->currentRB = 0.0;
    slave->voltageRB = 0.0;
    slave->currentSP = 0.0;
}

// Helper to set UNIMAG state and notify
static void unimag_setState(OCEM_Slave *slave, UnimagState newState, const char *msg)
{
    slave->unimag.state = newState;
    strncpy(slave->unimag.statusMsg, msg, sizeof(slave->unimag.statusMsg) - 1);
    slave->unimag.statusMsg[sizeof(slave->unimag.statusMsg) - 1] = '\0';
    
    OCEM_INFO("[PS%d] UNIMAG: %s - %s\n", slave->addr, unimag_getStateName(newState), msg);
    scanIoRequest(slave->ioscanUnimag);
}

// Update channel state from raw status
static void unimag_updateChannelState(OCEM_Slave *slave)
{
    ChannelState prev = slave->channelState;
    
    if (strcmp(slave->status, "ATT") == 0) {
        slave->channelState = CH_ON;
    } else if (strcmp(slave->status, "STB") == 0) {
        slave->channelState = CH_STANDBY;
    } else if (strncmp(slave->alarms, "NO", 2) != 0 && strlen(slave->alarms) > 0) {
        slave->channelState = CH_FAULT;
    } else {
        slave->channelState = CH_OFF;
    }
    
    if (prev != slave->channelState) {
        OCEM_INFO("[PS%d] Channel state: %s -> %s\n", slave->addr, 
                  unimag_getChannelStateName(prev), unimag_getChannelStateName(slave->channelState));
    }
}

// Calculate current readback in Amperes
static void unimag_updateCurrentRB(OCEM_Slave *slave)
{
    if (slave->currentPrgH <= 0 || slave->IMAX <= 0) {
        slave->currentRB = 0.0;
        return;
    }
    
    int rawValue = 0;
    sscanf(slave->current, "%d", &rawValue);
    
    // Current in Amperes = (raw * IMAX) / PRG_MAX * polarity
    slave->currentRB = ((double)rawValue * slave->IMAX) / (double)slave->currentPrgH;
    slave->currentRB *= slave->integerPolarity;  // Apply sign based on polarity
}

// Check if timeout has expired
static int unimag_isTimeout(OCEM_Slave *slave)
{
    epicsTimeStamp now;
    epicsTimeGetCurrent(&now);
    double elapsed = epicsTimeDiffInSeconds(&now, &slave->unimag.stepStartTime);
    return (elapsed >= slave->unimagCfg.setTimeoutS);
}

// Start timer for current step
static void unimag_startTimer(OCEM_Slave *slave)
{
    epicsTimeGetCurrent(&slave->unimag.stepStartTime);
}

// Set current setpoint and start state machine
void unimag_setCurrentSP(OCEM_Slave *slave, double currentA)
{
    OCEM_INFO("[PS%d] UNIMAG: New setpoint request: %.3f A\n", slave->addr, currentA);
    
    slave->currentSP = currentA;
    slave->unimag.targetCurrent = currentA;
    slave->unimag.targetPolarity = (currentA >= 0) ? 1 : -1;
    if (fabs(currentA) < 0.001) slave->unimag.targetPolarity = 0;
    
    slave->unimag.retryCount = 0;
    slave->unimag.busy = 1;
    unimag_startTimer(slave);
    
    // Check if polarity change is needed
    if (slave->unimag.targetPolarity != 0 && 
        slave->unimag.targetPolarity != slave->integerPolarity) {
        // Need polarity change - first go to zero
        unimag_setState(slave, UNIMAG_ZERO_STBY, "Ramping to zero for polarity change");
    } else {
        // Same polarity or zero - go directly to setpoint
        unimag_setState(slave, UNIMAG_GOING_TO_SET, "Ramping to setpoint");
    }
    
    scanIoRequest(slave->ioscanUnimag);
}

// Set state (ON/STANDBY/RESET)
void unimag_setStateSP(OCEM_Slave *slave, ChannelState state)
{
    OCEM_INFO("[PS%d] UNIMAG: State change request: %s\n", slave->addr, unimag_getChannelStateName(state));
    
    slave->unimag.targetState = state;
    slave->unimag.retryCount = 0;
    slave->unimag.busy = 1;
    unimag_startTimer(slave);
    
    if (state == CH_STANDBY && slave->channelState == CH_ON) {
        // Going from ON to STANDBY - need to ramp to zero first if not already
        if (fabs(slave->currentRB) > slave->unimagCfg.zeroTolerance) {
            unimag_setState(slave, UNIMAG_ZERO_STBY, "Ramping to zero before STANDBY");
        } else {
            // Already at zero, can go directly to STANDBY
            unimag_setState(slave, UNIMAG_OK, "Transitioning to STANDBY");
        }
    } else {
        // Direct state transition
        unimag_setState(slave, UNIMAG_OK, "State transition in progress");
    }
    
    scanIoRequest(slave->ioscanUnimag);
}

// Process UNIMAG state machine (called from polling loop)
void unimag_process(OCEM_Driver *drv, OCEM_Slave *slave)
{
    char response[128];
    size_t responseSize = sizeof(response);
    
    // Update calculated values
    unimag_updateChannelState(slave);
    unimag_updateCurrentRB(slave);
    
    // If not busy, nothing to do
    if (!slave->unimag.busy) {
        return;
    }
    
    double absCurrentRB = fabs(slave->currentRB);
    double absTargetCurrent = fabs(slave->unimag.targetCurrent);
    
    switch (slave->unimag.state) {
        case UNIMAG_OK:
            // Reached OK state - done
            slave->unimag.busy = 0;
            break;
            
        case UNIMAG_NOT_REACHED:
            // Error state - stay here until new command
            slave->unimag.busy = 0;
            break;
            
        case UNIMAG_ZERO_STBY:
            // Ramping to zero current
            if (absCurrentRB <= slave->unimagCfg.zeroTolerance) {
                // Zero reached - check what's next
                if (slave->unimag.targetPolarity != 0 && 
                    slave->unimag.targetPolarity != slave->integerPolarity) {
                    // Need polarity change
                    unimag_startTimer(slave);
                    slave->unimag.retryCount = 0;
                    unimag_setState(slave, UNIMAG_CHANGE_POL, "Changing polarity");
                    
                    // Send polarity command
                    const char *polCmd = (slave->unimag.targetPolarity > 0) ? "POS" : "NEG";
                    send_command(drv, slave->addr, (char*)polCmd, response, responseSize);
                } else if (slave->unimag.targetState == CH_STANDBY) {
                    // Transition to STANDBY complete
                    send_command(drv, slave->addr, "STB", response, responseSize);
                    unimag_setState(slave, UNIMAG_OK, "Reached STANDBY");
                    slave->unimag.busy = 0;
                } else {
                    // Go to setpoint
                    unimag_startTimer(slave);
                    slave->unimag.retryCount = 0;
                    unimag_setState(slave, UNIMAG_GOING_TO_SET, "Ramping to setpoint");
                }
            } else if (unimag_isTimeout(slave)) {
                // Timeout - retry or fail
                slave->unimag.retryCount++;
                if (slave->unimag.retryCount > slave->unimagCfg.maxRetries) {
                    unimag_setState(slave, UNIMAG_NOT_REACHED, "Failed: Zero not reached");
                    slave->unimag.busy = 0;
                } else {
                    // Retry: send zero setpoint again
                    OCEM_INFO("[PS%d] UNIMAG: Retry %d/%d for zero\n", 
                             slave->addr, slave->unimag.retryCount, slave->unimagCfg.maxRetries);
                    send_command(drv, slave->addr, "SP 0000000", response, responseSize);
                    send_command(drv, slave->addr, "STR", response, responseSize);
                    unimag_startTimer(slave);
                    snprintf(slave->unimag.statusMsg, sizeof(slave->unimag.statusMsg),
                             "Retry %d: Ramping to zero", slave->unimag.retryCount);
                }
            }
            break;
            
        case UNIMAG_CHANGE_POL:
            // Waiting for polarity change
            if (slave->integerPolarity == slave->unimag.targetPolarity) {
                // Polarity changed successfully
                unimag_startTimer(slave);
                slave->unimag.retryCount = 0;
                
                // Now go ON and ramp to setpoint
                send_command(drv, slave->addr, "ON", response, responseSize);
                unimag_setState(slave, UNIMAG_GOING_TO_SET, "Ramping to setpoint");
            } else if (unimag_isTimeout(slave)) {
                // Timeout - retry or fail
                slave->unimag.retryCount++;
                if (slave->unimag.retryCount > slave->unimagCfg.maxRetries) {
                    unimag_setState(slave, UNIMAG_NOT_REACHED, "Failed: Polarity change failed");
                    slave->unimag.busy = 0;
                } else {
                    // Retry polarity change
                    OCEM_INFO("[PS%d] UNIMAG: Retry %d/%d for polarity\n",
                             slave->addr, slave->unimag.retryCount, slave->unimagCfg.maxRetries);
                    const char *polCmd = (slave->unimag.targetPolarity > 0) ? "POS" : "NEG";
                    send_command(drv, slave->addr, (char*)polCmd, response, responseSize);
                    unimag_startTimer(slave);
                    snprintf(slave->unimag.statusMsg, sizeof(slave->unimag.statusMsg),
                             "Retry %d: Changing polarity", slave->unimag.retryCount);
                }
            }
            break;
            
        case UNIMAG_GOING_TO_SET:
            // Ramping to setpoint
            {
                double error = fabs(absCurrentRB - absTargetCurrent);
                
                // If tolerance is 0, skip verification
                if (slave->unimagCfg.setTolerance == 0) {
                    unimag_setState(slave, UNIMAG_OK, "Setpoint sent (no verify)");
                    slave->unimag.busy = 0;
                } else if (error <= slave->unimagCfg.setTolerance) {
                    // Setpoint reached
                    unimag_setState(slave, UNIMAG_OK, "Setpoint reached");
                    slave->unimag.busy = 0;
                } else if (unimag_isTimeout(slave)) {
                    // Timeout - retry or fail
                    slave->unimag.retryCount++;
                    if (slave->unimag.retryCount > slave->unimagCfg.maxRetries) {
                        char msg[80];
                        snprintf(msg, sizeof(msg), "Failed: Setpoint not reached (err=%.2fA)", error);
                        unimag_setState(slave, UNIMAG_NOT_REACHED, msg);
                        slave->unimag.busy = 0;
                    } else {
                        // Retry: send setpoint again
                        OCEM_INFO("[PS%d] UNIMAG: Retry %d/%d for setpoint\n",
                                 slave->addr, slave->unimag.retryCount, slave->unimagCfg.maxRetries);
                        
                        // Calculate and send setpoint
                        int bits = (int)((absTargetCurrent * slave->currentPrgH) / slave->IMAX);
                        char spCmd[20];
                        snprintf(spCmd, sizeof(spCmd), "SP %07d", bits);
                        send_command(drv, slave->addr, spCmd, response, responseSize);
                        send_command(drv, slave->addr, "STR", response, responseSize);
                        
                        unimag_startTimer(slave);
                        snprintf(slave->unimag.statusMsg, sizeof(slave->unimag.statusMsg),
                                 "Retry %d: Ramping to %.2fA", slave->unimag.retryCount, absTargetCurrent);
                    }
                }
            }
            break;
    }
}

/* --- Thread di polling --- */
static void ocem_polling(void *arg) {
    
    char response[128];
    size_t responseSize=128;
    epicsThreadSleep(0.1);
    
    OCEM_INFO("Initializing %d power supplies...\n", drv->nSlaves);
    
    for (int i=0; i < drv->nSlaves;i++)
    {
        OCEM_INFO("[PS%d] Sending RMT (remote mode)\n", drv->addrList[i]);
        send_command(drv,drv->addrList[i],"RMT",response,responseSize);
        epicsThreadSleep(0.05);
        OCEM_INFO("[PS%d] Sending PRG S (read parameters)\n", drv->addrList[i]);
        send_command(drv,drv->addrList[i],"PRG S",response,responseSize);
    }
    
    printf("OCEM Polling started - Idle: %.2fs, Active: %.2fs, Timeout: %.2fs\n",
           drv->idlePollingPeriod, drv->activePollingPeriod, drv->commandActiveTimeout);
    OCEM_INFO("Debug level: %d (0=errors, 1=info, 2=detail, 3=trace)\n", ocemDebugLevel);
    
    int cycleCount = 0;
    
    while(drv->running) 
    {
        cycleCount++;
        
        // Determine current polling rate based on activity
        int isActive = hasActiveCommands(drv);
        double currentPollingPeriod = isActive ? drv->activePollingPeriod : drv->idlePollingPeriod;
        
        OCEM_TRACE("=== Polling cycle %d (mode=%s, period=%.2fs) ===\n", 
                   cycleCount, isActive ? "ACTIVE" : "IDLE", currentPollingPeriod);
        
        // Process any queued commands first (priority to command processing)
        epicsMutexLock(drv->ioLock);
        process_queued_commands(drv, response, responseSize);
        epicsMutexUnlock(drv->ioLock);
        
        // Priority polling: First poll devices with pending commands or active states
        for (int pass = 0; pass < 2; pass++) {
            for (int i=0; i < drv->nSlaves;i++)
            {
                OCEM_Slave *slave = &drv->slaves[drv->addrList[i]];
                
                // Pass 0: Only poll slaves with pending commands or non-idle state
                // Pass 1: Poll all remaining slaves
                int hasPriority = (slave->hasPendingCommand || slave->Ostate != STATE_IDLE);
                if ((pass == 0 && !hasPriority) || (pass == 1 && hasPriority))
                    continue;
                
                if (pass == 0) {
                    OCEM_INFO("[PS%d] Priority poll (state=%d, pending=%d)\n", 
                             slave->addr, slave->Ostate, slave->hasPendingCommand);
                }
                
                checkCurrentSetStatus(slave);

                epicsMutexLock(drv->ioLock);
                int ret=poll_request(drv, slave, response, responseSize);
                if ( ret == 0)
                {
                    OCEM_INFO("[PS%d] POLL response: '%s'\n", slave->addr, response);
                    parseMultiReply(response);
                }
                else if (ret == 1)
                {
                    OCEM_DETAIL("[PS%d] FIFO empty, doing select\n", slave->addr);
                    select_request(drv, slave, response, responseSize);
                    epicsThreadSleep(0.05);
                }
                
                // Process UNIMAG state machine
                unimag_process(drv, slave);
                
                epicsMutexUnlock(drv->ioLock);
            }
        }
        
        epicsThreadSleep(currentPollingPeriod);
    }
}





/* --- Inizializzazione driver --- */
static void ocem_init(const char *port, int nSlaves, const char *addrListStr) {
    drv = calloc(1, sizeof(OCEM_Driver));
    drv->pollingPeriodParam = -1;
    
    
    drv->port = strdup(port);
    drv->nSlaves = nSlaves;
    drv->running = 1;
    // Default polling rates
    drv->idlePollingPeriod = 1.0;     // 1s when idle (just monitoring)
    drv->activePollingPeriod = 0.1;   // 100ms when commands active
    drv->commandActiveTimeout = 5.0;  // Stay in active mode for 5s after last command
    int addrList[MAX_SLAVE];
    drv->ioLock = epicsMutexCreate();
    if (!drv->ioLock) {
        errlogPrintf("OCEM: failed to create ioLock mutex\n");
        return;
    }
    drv->cmdQueueLock = epicsMutexCreate();
    if (!drv->cmdQueueLock) {
        errlogPrintf("OCEM: failed to create cmdQueueLock mutex\n");
        return;
    }
    drv->cmdQueueHead = NULL;
    drv->cmdQueueTail = NULL;
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
        drv->slaves[addr].IMAX = -1;
        drv->slaves[addr].VMAX = -1;
        drv->slaves[addr].currentPrgH=0;
        drv->slaves[addr].hasPendingCommand = 0;
        drv->slaves[addr].Ostate = STATE_IDLE;
        drv->slaves[addr].cmdState = CMD_IDLE;
        drv->slaves[addr].cmdRetryCount = 0;
        drv->slaves[addr].cmdVerifyCount = 0;
        strcpy(drv->slaves[addr].cmdStatusMsg, "Idle");
        strcpy(drv->slaves[addr].lastCommand, "");
        
        // Initialize UNIMAG state machine
        unimag_init(&drv->slaves[addr]);
        
        scanIoInit(&drv->slaves[addr].ioscanStatus);
        scanIoInit(&drv->slaves[addr].ioscanCurrent);
        scanIoInit(&drv->slaves[addr].ioscanVoltage);
        scanIoInit(&drv->slaves[addr].ioscanPolarity);
        scanIoInit(&drv->slaves[addr].ioscanAlarms);
        scanIoInit(&drv->slaves[addr].ioscanSelector);
        scanIoInit(&drv->slaves[addr].ioscanInit);
        scanIoInit(&drv->slaves[addr].ioscanCmdState);
        scanIoInit(&drv->slaves[addr].ioscanUnimag);
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
    drv->pasynUser->timeout = 0.3;  // Reduced from 0.5




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

/* --- Set polling rates command --- */
void ocem_setPollingRates(double idlePeriod, double activePeriod, double activeTimeout)
{
    if (!drv) {
        errlogPrintf("ocemSetPollingRates: driver not initialized\n");
        return;
    }
    if (idlePeriod > 0) drv->idlePollingPeriod = idlePeriod;
    if (activePeriod > 0) drv->activePollingPeriod = activePeriod;
    if (activeTimeout > 0) drv->commandActiveTimeout = activeTimeout;
    
    printf("OCEM Polling rates updated - Idle: %.2fs, Active: %.2fs, Timeout: %.2fs\n",
           drv->idlePollingPeriod, drv->activePollingPeriod, drv->commandActiveTimeout);
}

/* --- Set debug level command --- */
static void ocem_setDebugLevel(int level)
{
    ocemDebugLevel = level;
    printf("OCEM Debug level set to %d\n", ocemDebugLevel);
    printf("  0 = Errors only\n");
    printf("  1 = Basic info (commands, polling)\n");
    printf("  2 = Protocol details (ENQ/ACK, bytes)\n");
    printf("  3 = Full trace (all function calls)\n");
}

static const iocshArg pollArg0 = {"idlePeriod", iocshArgDouble};
static const iocshArg pollArg1 = {"activePeriod", iocshArgDouble};
static const iocshArg pollArg2 = {"activeTimeout", iocshArgDouble};
static const iocshArg *pollArgs[3] = {&pollArg0, &pollArg1, &pollArg2};
static const iocshFuncDef pollFuncDef = {"ocemSetPollingRates", 3, pollArgs};
static void pollCall(const iocshArgBuf *args) {
    ocem_setPollingRates(args[0].dval, args[1].dval, args[2].dval);
}

static const iocshArg debugArg0 = {"level", iocshArgInt};
static const iocshArg *debugArgs[1] = {&debugArg0};
static const iocshFuncDef debugFuncDef = {"ocemSetDebug", 1, debugArgs};
static void debugCall(const iocshArgBuf *args) {
    ocem_setDebugLevel(args[0].ival);
}

static void drvOCEMRegister(void) {
    iocshRegister(&initFuncDef, initCall);
    iocshRegister(&pollFuncDef, pollCall);
    iocshRegister(&debugFuncDef, debugCall);
}
epicsExportRegistrar(drvOCEMRegister);
