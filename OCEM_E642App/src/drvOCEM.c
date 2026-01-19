

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

// Helper to strip trailing " **" or "*" from value strings
static void stripTrailingStar(char *val) {
    char *p = strstr(val, " **");
    if (p) *p = '\0';
    else {
        p = strstr(val, " *");
        if (p) *p = '\0';
    }
    // Also strip trailing whitespace
    size_t len = strlen(val);
    while (len > 0 && (val[len-1] == ' ' || val[len-1] == '\t' || val[len-1] == '\r' || val[len-1] == '\n')) {
        val[--len] = '\0';
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
        slave->hasSTA = 1;  // Mark STA received for initialization tracking

        // Determine base unimagStatus from PS status
        // Values: 0=OFF, 1=ON, 2=STANDBY, 3=RESET, 4=INTERLOCK, 5=ERROR, 6=SETPOINT_NOT_REACHED, 7=STATE_NOT_REACHED
        if (strcmp(val, "ATT") == 0)
            slave->unimagStatus = 1;  // ON
        else if (strcmp(val, "STB") == 0)
            slave->unimagStatus = 2;  // STANDBY
        else if (strcmp(val, "RES") == 0)
            slave->unimagStatus = 3;  // RESET
        else if (strcmp(val, "OFF") == 0)
            slave->unimagStatus = 0;  // OFF
        else
            slave->unimagStatus = 5;  // ERROR (unknown state)

        // Override with INTERLOCK if alarms are present
        // Check if alarms string is non-empty AND doesn't start with "NO"
        if (slave->alarms[0] != '\0' && strncmp(slave->alarms, "NO", 2) != 0)
        {
            slave->unimagStatus = 4;  // INTERLOCK
        }
        
        // Override with STATE_NOT_REACHED if UNIMAG state machine indicates failure
        if (slave->unimag.state == UNIMAG_NOT_REACHED)
        {
            slave->unimagStatus = 7;  // STATE_NOT_REACHED
        }
        
        scanIoRequest(slave->ioscanStatus);
        
        // Check if this status update verifies a pending command
        verifyCommandApplied(slave);
    }
    else if (strcmp(cmd, "COR") == 0) {
        // Strip trailing " **" suffix before storing
        char cleanVal[40];
        strncpy(cleanVal, val, sizeof(cleanVal) - 1);
        cleanVal[sizeof(cleanVal)-1] = '\0';
        stripTrailingStar(cleanVal);
        
        strncpy(slave->current, cleanVal, sizeof(slave->current));
        slave->current[sizeof(slave->current)-1] = '\0';
        slave->hasCOR = 1;  // Mark COR received for initialization tracking
        scanIoRequest(slave->ioscanCurrent);
    }
    else if (strcmp(cmd, "TEN") == 0) 
    {
        // Strip trailing " **" suffix before storing
        char cleanVal[40];
        strncpy(cleanVal, val, sizeof(cleanVal) - 1);
        cleanVal[sizeof(cleanVal)-1] = '\0';
        stripTrailingStar(cleanVal);
        
        strncpy(slave->voltage, cleanVal, sizeof(slave->voltage));
        slave->voltage[sizeof(slave->voltage)-1] = '\0';
        slave->hasTEN = 1;  // Mark TEN received for initialization tracking
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
        // Store raw alarm string
        strncpy(slave->alarms, val, sizeof(slave->alarms));
        slave->alarms[sizeof(slave->alarms)-1] = '\0';
        
        // Parse alarm codes and build bitmask
        // Format: "NO " or "NO" = no alarms, "01 03 24" = alarms 1, 3, 24 active
        slave->alarmMask = 0;
        slave->hasAlarm = 0;
        
        // Check for "NO" (no alarms)
        if (strncasecmp(val, "NO", 2) != 0) {
            // Parse space-separated alarm numbers
            char *valCopy = strdup(val);
            if (valCopy) {
                char *saveptr = NULL;
                char *token = strtok_r(valCopy, " ", &saveptr);
                while (token) {
                    int alarmNum = atoi(token);
                    if (alarmNum > 0 && alarmNum <= 32) {
                        slave->alarmMask |= (1U << (alarmNum - 1));
                        slave->hasAlarm = 1;
                    }
                    token = strtok_r(NULL, " ", &saveptr);
                }
                free(valCopy);
            }
            if (slave->hasAlarm) {
                OCEM_INFO("[PS%d] ALARMS active: %s (mask=0x%08X)\n", 
                         slave->addr, val, slave->alarmMask);
            }
        }
        
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
    else if (strcmp(cmd, "VER") == 0)
    {
        // Store firmware version
        strncpy(slave->version, val, sizeof(slave->version));
        slave->version[sizeof(slave->version)-1] = '\0';
        // Strip trailing whitespace
        size_t len = strlen(slave->version);
        while (len > 0 && (slave->version[len-1] == ' ' || slave->version[len-1] == '\r' || slave->version[len-1] == '\n')) {
            slave->version[--len] = '\0';
        }
        slave->hasVER = 1;  // Mark VER received for initialization tracking
        OCEM_INFO("[PS%d] VER: '%s'\n", slaveId, slave->version);
        scanIoRequest(slave->ioscanInit);  // Use init scan for version update
    }
    else  if (strcmp(cmd, "PRG") == 0) 
    {
        int minvalue;int maxvalue;
        if (parsePRGAnswer(val,"O0",&minvalue,&maxvalue) == 0)
        {
            slave->currentPrgL=minvalue;
            slave->currentPrgH=maxvalue;
            slave->hasPRG = 1;  // Mark PRG received for initialization tracking
            scanIoRequest(slave->ioscanInit);
            errlogPrintf("CURRENT PRG: min : %d max %d\n",minvalue,maxvalue);
            
            // Check if PRG initialization is complete (both current and voltage ranges)
            if (slave->currentPrgH > 0 && slave->voltagePrgH > 0 && slave->IMAX > 0) {
                if (!slave->unimag.prgInitialized) {
                    slave->unimag.prgInitialized = 1;
                    OCEM_INFO("[PS%d] PRG initialization complete\n", slave->addr);
                }
            }
        }
        else if (parsePRGAnswer(val,"O1",&minvalue,&maxvalue) == 0)
        {
            slave->voltagePrgL=minvalue;
            slave->voltagePrgH=maxvalue;
            scanIoRequest(slave->ioscanInit);
            errlogPrintf("VOLTAGE PRG:  min : %d max %d\n",minvalue,maxvalue);
            
            // Check if PRG initialization is complete (both current and voltage ranges)
            if (slave->currentPrgH > 0 && slave->voltagePrgH > 0 && slave->IMAX > 0) {
                if (!slave->unimag.prgInitialized) {
                    slave->unimag.prgInitialized = 1;
                    OCEM_INFO("[PS%d] PRG initialization complete\n", slave->addr);
                }
            }
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
    size_t inputLen = strlen(input);
    
    OCEM_TRACE("parseMultiReply: raw input (len=%zu): ", inputLen);
    if (ocemDebugLevel >= 3) {
        for (size_t k = 0; k < inputLen && k < 40; k++) {
            OCEM_TRACE("%02X ", (unsigned char)input[k]);
        }
        OCEM_TRACE("\n");
    }
    
    // Skip leading STX (0x02) if present
    size_t startPos = 0;
    if (inputLen > 0 && (unsigned char)input[0] == 0x02) {
        startPos = 1;
    }
    
    // Find ETX (0x03) or end of valid data - stop at any STX in middle of message
    size_t endPos = inputLen;
    for (size_t k = startPos; k < inputLen; k++) {
        unsigned char c = (unsigned char)input[k];
        if (c == 0x03) {  // ETX - end of this message
            endPos = k;
            break;
        }
        if (k > startPos && c == 0x02) {  // STX in middle - data from another message mixed in
            OCEM_DETAIL("parseMultiReply: found STX at pos %zu, truncating\n", k);
            endPos = k;
            break;
        }
    }
    
    // Copy clean portion
    size_t cleanLen = 0;
    for (i = startPos; i < endPos && cleanLen < sizeof(cleaned) - 1; i++) {
        unsigned char c = (unsigned char)input[i];
        // Skip control characters except CR/LF
        if (c >= 0x20 || c == '\r' || c == '\n') {
            cleaned[cleanLen++] = input[i];
        }
    }
    cleaned[cleanLen] = '\0';
    
    if (cleanLen == 0) {
        OCEM_DETAIL("parseMultiReply: no valid data after cleaning\n");
        return;
    }
    
    OCEM_DETAIL("parseMultiReply: cleaned='%s' (len=%zu)\n", cleaned, cleanLen);
    
    const char *ptr = cleaned;
    char lastSlaveChar = '@';  // <-- default to address 0 (0x40 = '@')
    int firstLine = 1;

    while (*ptr) {
        int len = 0;
        while (*ptr && *ptr != '\n' && *ptr != '\r' && len < MAX_LINE-1) {
            buffer[len++] = *ptr++;
        }
        buffer[len] = '\0';

        while (*ptr == '\n' || *ptr == '\r') ptr++;
        if (len == 0) continue;

        char fullLine[MAX_LINE+1];
        if (firstLine) {
            strcpy(fullLine, buffer);
            firstLine = 0;
        } else {
            // Prepend the saved address character
            snprintf(fullLine, sizeof(fullLine), "%c%s", lastSlaveChar, buffer);
        }

        int slaveId;
        char cmd[32], val[32];
        if (parseReplyString(fullLine, &slaveId, cmd, val) == 0) {
            // Save address char if it's a valid poll address (0x40-0x5F) or select address (0x60-0x7F)
            unsigned char fc = (unsigned char)fullLine[0];
            if ((fc >= 0x40 && fc <= 0x5F) || (fc >= 0x60 && fc <= 0x7F))
                lastSlaveChar = fullLine[0];

            ActivateInterrupt(slaveId, cmd, val);
        }
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

// Send a select command (SL for state, SA for operating data)
// If specificCmd is NULL, alternates between SL and SA
int select_command(OCEM_Driver* drv, OCEM_Slave* slave, const char* specificCmd, char* response, size_t responseSize)
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
    const char *cmd = specificCmd ? specificCmd : getNextCommandForSlave(slave);
    strncpy(slave->lastSelCommand, cmd, sizeof(slave->lastSelCommand) - 1);
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

// Store record prefix from a record name (e.g., "BTF:MAG:OCEME642:QUATB102:STATUS" -> "BTF:MAG:OCEME642:QUATB102")
void ocem_setRecordPrefix(OCEM_Slave* slave, const char* recordName)
{
    if (!slave || !recordName || slave->recordPrefix[0] != '\0') return;
    
    // Find the last ':' to strip the field name
    const char* lastColon = strrchr(recordName, ':');
    if (!lastColon) return;
    
    size_t prefixLen = lastColon - recordName;
    if (prefixLen >= sizeof(slave->recordPrefix)) {
        prefixLen = sizeof(slave->recordPrefix) - 1;
    }
    strncpy(slave->recordPrefix, recordName, prefixLen);
    slave->recordPrefix[prefixLen] = '\0';
    OCEM_INFO("[PS%d] Record prefix set to '%s'\n", slave->addr, slave->recordPrefix);
}

// Read IMAX and VMAX from the database records directly by name
void ocem_readImaxVmax(OCEM_Slave* slave)
{
    if (!slave || slave->recordPrefix[0] == '\0') return;
    
    char pvName[128];
    struct dbAddr addr;
    double value = 0;
    long status;
    long options = 0;
    long nreq = 1;
    
    // Read IMAX if not yet set
    if (slave->IMAX <= 0) {
        snprintf(pvName, sizeof(pvName), "%s:IMAX", slave->recordPrefix);
        if (dbNameToAddr(pvName, &addr) == 0) {
            status = dbGetField(&addr, DBR_DOUBLE, &value, &options, &nreq, NULL);
            if (status == 0 && value > 0) {
                slave->IMAX = value;
                OCEM_INFO("[PS%d] IMAX read from '%s' = %.1f A\n", slave->addr, pvName, slave->IMAX);
            }
        } else {
            OCEM_DETAIL("[PS%d] IMAX record '%s' not found\n", slave->addr, pvName);
        }
    }
    
    // Read VMAX if not yet set
    if (slave->VMAX <= 0) {
        snprintf(pvName, sizeof(pvName), "%s:VMAX", slave->recordPrefix);
        if (dbNameToAddr(pvName, &addr) == 0) {
            value = 0;
            status = dbGetField(&addr, DBR_DOUBLE, &value, &options, &nreq, NULL);
            if (status == 0 && value > 0) {
                slave->VMAX = value;
                OCEM_INFO("[PS%d] VMAX read from '%s' = %.1f V\n", slave->addr, pvName, slave->VMAX);
            }
        } else {
            OCEM_DETAIL("[PS%d] VMAX record '%s' not found\n", slave->addr, pvName);
        }
    }
}

void checkCurrentSetStatus(OCEM_Slave* slave)
{
    char response[128];
    size_t responseSize=128;
    
    // Read IMAX and VMAX from database records directly by name
    if (slave->IMAX <= 0 || slave->VMAX <= 0)
    {
        ocem_readImaxVmax(slave);
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
// POLL RATIO HELPER FUNCTIONS
// ============================================

// Get the effective poll ratio based on current UNIMAG state
// Returns ratio = analog polls per state poll
static double getPollRatio(OCEM_Slave *slave)
{
    switch (slave->unimag.state) {
        case UNIMAG_GOING_TO_SET:
            return slave->pollRatioSet;         // High analog ratio when ramping
        case UNIMAG_CHANGE_POL:
        case UNIMAG_WAIT_POL:
            return slave->pollRatioState;       // Low analog ratio during polarity change
        case UNIMAG_ZERO_STBY:
            return slave->pollRatioBalanced;    // Balanced during zero/standby
        case UNIMAG_OK:
        case UNIMAG_INIT:
        case UNIMAG_NOT_REACHED:
        default:
            return slave->pollRatioNormal;      // Normal operation
    }
}

// Determine whether to poll analog (SA) or state (SL) based on ratio
// Returns: 1 = poll analog (SA), 0 = poll state (SL)
// Uses cycle counter to distribute polls according to ratio
static int shouldPollAnalog(OCEM_Slave *slave)
{
    double ratio = getPollRatio(slave);
    
    // Handle special cases
    if (ratio <= 0.01) {
        // Effectively 0 ratio: always poll state
        return 0;
    }
    if (ratio >= 100.0) {
        // Very high ratio: always poll analog
        return 1;
    }
    
    // Calculate target counts per cycle
    // ratio = analog/state, so if ratio=2, we want 2 analog per 1 state
    // If ratio=0.33, we want 1 analog per 3 state
    int targetAnalog, targetState;
    
    if (ratio >= 1.0) {
        // More analog polls than state
        targetAnalog = (int)(ratio + 0.5);  // Round to nearest
        targetState = 1;
    } else {
        // More state polls than analog
        targetAnalog = 1;
        targetState = (int)(1.0/ratio + 0.5);  // Round to nearest
    }
    
    // Decide based on current counts in cycle
    int totalTarget = targetAnalog + targetState;
    int totalDone = slave->pollAnalogCount + slave->pollStateCount;
    
    // Reset cycle if complete
    if (totalDone >= totalTarget) {
        slave->pollAnalogCount = 0;
        slave->pollStateCount = 0;
        OCEM_DETAIL("[PS%d] Poll cycle reset (ratio=%.2f, target A=%d S=%d)\n",
                   slave->addr, ratio, targetAnalog, targetState);
    }
    
    // Decide what to poll next
    // Priority: if analog count < target ratio, poll analog
    // Use floating point to handle fractional ratios
    double analogRatio = (slave->pollStateCount > 0) ? 
                         (double)slave->pollAnalogCount / slave->pollStateCount : 
                         (double)slave->pollAnalogCount;
    
    if (slave->pollStateCount == 0) {
        // Haven't polled state yet - check if we should do analog first
        return (slave->pollAnalogCount < targetAnalog);
    }
    
    // Compare current ratio to target
    return (analogRatio < ratio);
}

// ============================================
// UNIMAG STATE MACHINE IMPLEMENTATION
// ============================================

const char* unimag_getStateName(UnimagState state)
{
    switch(state) {
        case UNIMAG_INIT:        return "INIT";
        case UNIMAG_OK:          return "OK";
        case UNIMAG_NOT_REACHED: return "NOT_REACHED";
        case UNIMAG_ZERO_STBY:   return "ZERO_STBY";
        case UNIMAG_CHANGE_POL:  return "CHANGE_POL";
        case UNIMAG_WAIT_POL:    return "WAIT_POL";
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
    slave->unimagCfg.initTimeoutS = 60.0;   // 60 second init timeout
    slave->unimagCfg.maxRetries = 3;        // 3 retries
    slave->unimagCfg.retryDelay = 1.0;      // 1s between retries
    
    // Start in OK state since default PRG values are already set
    // PRG data from polling will update the values but calculation can start immediately
    slave->unimag.state = UNIMAG_OK;
    slave->unimag.busy = 0;
    slave->unimag.retryCount = 0;
    slave->unimag.targetCurrent = 0.0;
    slave->unimag.targetPolarity = 0;
    slave->unimag.targetState = CH_OFF;
    slave->unimag.prgInitialized = 1;  // Defaults are set, consider initialized
    epicsTimeGetCurrent(&slave->unimag.initStartTime);
    strcpy(slave->unimag.statusMsg, "Ready (using default PRG values)");
    
    slave->channelState = CH_OFF;
    slave->currentRB = 0.0;
    slave->voltageRB = 0.0;
    slave->currentSP = 0.0;
    
    // Initialize alarms to "NO " so alarm check works correctly at startup
    strcpy(slave->alarms, "NO ");
    slave->hasAlarm = 0;
    slave->alarmMask = 0;
    slave->unimagStatus = 0;  // Start as OFF until first STA is received
    
    // Threshold defaults: will be set to 5% of IMAX/VMAX when those are known
    // -1 means "use default when IMAX/VMAX become available"
    slave->currentThresholdA = -1.0;
    slave->voltageThresholdV = -1.0;
    slave->thresholdsSent = 0;
}

// Send threshold command to PS
// channel: 0 = current (I0), 1 = voltage (I1)
// valueA: threshold value in Amperes (for I0) or Volts (for I1)
// Returns 0 on success, -1 on error
int ocem_sendThreshold(OCEM_Slave *slave, int channel, double value)
{
    if (!slave || !drv) return -1;
    
    char response[128];
    size_t responseSize = sizeof(response);
    
    // Convert to raw units
    int rawValue = 0;
    if (channel == 0) {
        // Current threshold: convert Amperes to raw bits
        if (slave->currentPrgH <= 0 || slave->IMAX <= 0) {
            OCEM_ERR("[PS%d] Cannot send current threshold - PRG not initialized\n", slave->addr);
            return -1;
        }
        rawValue = (int)((value * slave->currentPrgH) / slave->IMAX);
    } else if (channel == 1) {
        // Voltage threshold: convert Volts to raw bits
        if (slave->voltagePrgH <= 0 || slave->VMAX <= 0) {
            OCEM_ERR("[PS%d] Cannot send voltage threshold - PRG not initialized\n", slave->addr);
            return -1;
        }
        rawValue = (int)((value * slave->voltagePrgH) / slave->VMAX);
    } else {
        OCEM_ERR("[PS%d] Invalid threshold channel %d\n", slave->addr, channel);
        return -1;
    }
    
    // Clamp to valid range (6 digits max = 999999)
    if (rawValue < 0) rawValue = 0;
    if (rawValue > 999999) rawValue = 999999;
    
    // Format TH command: "TH I<channel> <6-digit value>"
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "TH I%d %06d", channel, rawValue);
    
    OCEM_INFO("[PS%d] Sending threshold: %s (%.3f %s)\n", 
              slave->addr, cmd, value, channel == 0 ? "A" : "V");
    
    epicsMutexLock(drv->ioLock);
    send_command(drv, slave->addr, cmd, response, responseSize);
    epicsMutexUnlock(drv->ioLock);
    
    return 0;
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

// Check if current is moving toward target
// Returns: 1 if moving toward target, 0 if stalled
static int unimag_isProgressing(OCEM_Slave *slave)
{
    double absCurrent = fabs(slave->currentRB);
    double absTarget = fabs(slave->unimag.targetCurrent);
    double lastAbs = fabs(slave->unimag.lastCurrentRB);
    double errorNow = fabs(absCurrent - absTarget);
    double errorLast = fabs(lastAbs - absTarget);
    
    // If error has decreased (moving toward target), we're progressing
    // Use a small threshold to avoid noise triggering false positives
    double progressThreshold = 0.01; // 10mA minimum movement
    if (errorLast - errorNow > progressThreshold) {
        return 1;
    }
    return 0;
}

// Update progress tracking and check if timeout has expired
// Timeout only counts when current is NOT moving toward target
static int unimag_isTimeout(OCEM_Slave *slave)
{
    epicsTimeStamp now;
    epicsTimeGetCurrent(&now);
    
    // Check if we're making progress
    if (unimag_isProgressing(slave)) {
        // Reset timeout timer when progressing
        epicsTimeGetCurrent(&slave->unimag.lastProgressTime);
        OCEM_DETAIL("[PS%d] UNIMAG: Progress detected, resetting timeout\n", slave->addr);
    }
    
    // Update last current for next progress check
    slave->unimag.lastCurrentRB = slave->currentRB;
    
    // Timeout based on last progress time, not step start time
    double elapsed = epicsTimeDiffInSeconds(&now, &slave->unimag.lastProgressTime);
    return (elapsed >= slave->unimagCfg.setTimeoutS);
}

// Start timer for current step and initialize progress tracking
static void unimag_startTimer(OCEM_Slave *slave)
{
    epicsTimeGetCurrent(&slave->unimag.stepStartTime);
    epicsTimeGetCurrent(&slave->unimag.lastProgressTime);
    slave->unimag.lastCurrentRB = slave->currentRB;
}

// Helper to send SP and STR commands for current setpoint
// Returns 0 on success, -1 if PRG parameters not yet available
static int unimag_sendSetpointCmd(OCEM_Slave *slave, double currentA)
{
    char response[128];
    size_t responseSize = sizeof(response);
    
    // Check if PRG parameters are valid
    if (slave->currentPrgH <= 0 || slave->IMAX <= 0) {
        OCEM_ERR("[PS%d] UNIMAG: Cannot send setpoint - PRG not initialized (currentPrgH=%d, IMAX=%.1f)\n",
                   slave->addr, slave->currentPrgH, slave->IMAX);
        return -1;
    }
    
    // Convert Amperes to raw bits (always positive)
    int rawBits = ampereInBits(fabs(currentA), slave);
    
    // Format SP command with 7-digit value
    char spCmd[16];
    snprintf(spCmd, sizeof(spCmd), "SP %07d", rawBits);
    
    // send_command already logs the command
    send_command(drv, slave->addr, spCmd, response, responseSize);
    send_command(drv, slave->addr, "STR", response, responseSize);
    
    return 0;
}

// Set current setpoint and start state machine
void unimag_setCurrentSP(OCEM_Slave *slave, double currentA)
{
    OCEM_INFO("[PS%d] UNIMAG: New setpoint request: %.3f A\n", slave->addr, currentA);
    
    // Check if still in INIT state
    if (slave->unimag.state == UNIMAG_INIT) {
        OCEM_ERR("[PS%d] UNIMAG: Rejected - still initializing, wait for PRG data\n", slave->addr);
        // Don't change state, just log and return
        return;
    }
    
    // Check if PRG parameters are ready before accepting command
    if (slave->currentPrgH <= 0 || slave->IMAX <= 0 || !slave->unimag.prgInitialized) {
        OCEM_ERR("[PS%d] UNIMAG: Rejected - PRG parameters not initialized\n", slave->addr);
        unimag_setState(slave, UNIMAG_NOT_REACHED, "PRG not initialized - wait for init");
        scanIoRequest(slave->ioscanUnimag);
        return;
    }
    
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
        // Need polarity change - first go to zero, then change polarity
        unimag_setState(slave, UNIMAG_ZERO_STBY, "Ramping to zero for polarity change");
        // Send SP 0 and STR to start ramping to zero
        unimag_sendSetpointCmd(slave, 0.0);
    } else {
        // Same polarity or zero - go directly to setpoint
        unimag_setState(slave, UNIMAG_GOING_TO_SET, "Ramping to setpoint");
        // Send SP and STR to start ramping
        unimag_sendSetpointCmd(slave, currentA);
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
    
    if (state == CH_RESET) {
        // RESET command: clear state machine errors and send RES command
        // This cancels STATE_NOT_REACHED and SET_NOT_REACHED errors
        OCEM_INFO("[PS%d] UNIMAG: RESET - clearing state machine errors\n", slave->addr);
        
        // Clear the error state - go back to OK
        unimag_setState(slave, UNIMAG_OK, "Reset - errors cleared");
        
        // Clear busy flag - reset is immediate
        slave->unimag.busy = 0;
        
        // Update unimagStatus to clear any error display (will be updated on next STA)
        slave->unimagStatus = 3;  // RESET status
        
    } else if (state == CH_STANDBY && slave->channelState == CH_ON) {
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
// SIMPLIFIED: No retry logic - if timeout occurs, report failure immediately
// Retries at this level mask underlying communication issues
void unimag_process(OCEM_Driver *drv, OCEM_Slave *slave)
{
    char response[128];
    size_t responseSize = sizeof(response);
    
    // Update calculated values
    unimag_updateChannelState(slave);
    unimag_updateCurrentRB(slave);
    
    // Always trigger I/O Intr for UNIMAG PVs (CURRENT_RB, VOLTAGE_RB, etc.)
    // This MUST be done BEFORE any early returns so PVs update on every poll
    OCEM_DETAIL("[PS%d] Triggering ioscanUnimag (currentRB=%.3f)\n", slave->addr, slave->currentRB);
    scanIoRequest(slave->ioscanUnimag);
    
    // Send thresholds when IMAX/VMAX become available (one-time)
    if (!slave->thresholdsSent && slave->IMAX > 0 && slave->VMAX > 0 &&
        slave->currentPrgH > 0 && slave->voltagePrgH > 0) {
        
        // Set defaults if not configured (5% of max)
        if (slave->currentThresholdA < 0) {
            slave->currentThresholdA = slave->IMAX * 0.05;
        }
        if (slave->voltageThresholdV < 0) {
            slave->voltageThresholdV = slave->VMAX * 0.05;
        }
        
        // Send threshold commands
        ocem_sendThreshold(slave, 0, slave->currentThresholdA);
        ocem_sendThreshold(slave, 1, slave->voltageThresholdV);
        slave->thresholdsSent = 1;
        
        OCEM_INFO("[PS%d] Thresholds initialized: current=%.3fA, voltage=%.3fV\n",
                  slave->addr, slave->currentThresholdA, slave->voltageThresholdV);
    }
    
    // Handle INIT state - wait for global init phase to complete
    if (slave->unimag.state == UNIMAG_INIT) {
        // The main polling loop handles transition from INIT to OK
        // after startInitTimeS expires. Just wait here.
        if (slave->unimag.prgInitialized) {
            // PRG data received - transition to OK
            unimag_setState(slave, UNIMAG_OK, "Initialization complete");
            slave->unimag.busy = 0;
        }
        return;  // Don't process other states while in INIT
    }
    
    // If not busy, nothing to do
    if (!slave->unimag.busy) {
        return;
    }
    
    double absCurrentRB = fabs(slave->currentRB);
    double absTargetCurrent = fabs(slave->unimag.targetCurrent);
    
    switch (slave->unimag.state) {
        case UNIMAG_INIT:
            // Handled above, should not reach here
            break;
            
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
                    // Need polarity change - first go to STANDBY
                    unimag_startTimer(slave);
                    unimag_setState(slave, UNIMAG_CHANGE_POL, "Going to STANDBY for polarity change");
                    send_command(drv, slave->addr, "STB", response, responseSize);
                } else if (slave->unimag.targetState == CH_STANDBY) {
                    // Transition to STANDBY complete
                    send_command(drv, slave->addr, "STB", response, responseSize);
                    unimag_setState(slave, UNIMAG_OK, "Reached STANDBY");
                    slave->unimag.busy = 0;
                } else {
                    // Go to setpoint - send SP and STR commands
                    unimag_startTimer(slave);
                    unimag_setState(slave, UNIMAG_GOING_TO_SET, "Ramping to setpoint");
                    unimag_sendSetpointCmd(slave, slave->unimag.targetCurrent);
                }
            } else if (unimag_isTimeout(slave)) {
                // Timeout - fail immediately (no retries)
                char msg[80];
                snprintf(msg, sizeof(msg), "Failed: Zero not reached (current=%.2fA) tolerance %.2fA", absCurrentRB, slave->unimagCfg.zeroTolerance);
                unimag_setState(slave, UNIMAG_NOT_REACHED, msg);
                slave->unimag.busy = 0;
            }
            break;
            
        case UNIMAG_CHANGE_POL:
            // Waiting for STANDBY status before sending polarity command
            if (slave->channelState == CH_STANDBY) {
                // In STANDBY - now send polarity command
                unimag_startTimer(slave);
                const char *polCmd = (slave->unimag.targetPolarity > 0) ? "POS" : "NEG";
                send_command(drv, slave->addr, (char*)polCmd, response, responseSize);
                unimag_setState(slave, UNIMAG_WAIT_POL, "Waiting for polarity change");
            } else if (unimag_isTimeout(slave)) {
                unimag_setState(slave, UNIMAG_NOT_REACHED, "Failed: STANDBY not reached for polarity change");
                slave->unimag.busy = 0;
            }
            break;
            
        case UNIMAG_WAIT_POL:
            // Waiting for polarity change to complete
            if (slave->integerPolarity == slave->unimag.targetPolarity) {
                // Polarity changed successfully - now go ON and ramp to setpoint
                unimag_startTimer(slave);
                send_command(drv, slave->addr, "ON", response, responseSize);
                unimag_setState(slave, UNIMAG_GOING_TO_SET, "Ramping to setpoint");
                unimag_sendSetpointCmd(slave, slave->unimag.targetCurrent);
            } else if (unimag_isTimeout(slave)) {
                unimag_setState(slave, UNIMAG_NOT_REACHED, "Failed: Polarity change failed");
                slave->unimag.busy = 0;
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
                    // Timeout - fail immediately (no retries)
                    char msg[80];
                    snprintf(msg, sizeof(msg), "Failed: Setpoint not reached (err=%.2fA)", error);
                    unimag_setState(slave, UNIMAG_NOT_REACHED, msg);
                    slave->unimag.busy = 0;
                }
            }
            break;
    }
}

// Check if a slave has received all required init data
static int ocem_isSlaveInitComplete(OCEM_Slave *slave)
{
    // Required: VER, PRG (at least currentPrgH), STA, COR, TEN
    // Note: hasPRG is set when O0 is received (currentPrgH)
    return (slave->hasVER && slave->hasPRG && slave->hasSTA && 
            slave->hasCOR && slave->hasTEN);
}

// Get initialization state name for logging
static const char* ocem_getInitStateName(PSInitState state)
{
    switch(state) {
        case PS_INIT_NOT_STARTED: return "NOT_STARTED";
        case PS_INIT_DRAINING_FIFO: return "DRAINING_FIFO";
        case PS_INIT_SENDING_RMT: return "SENDING_RMT";
        case PS_INIT_WAITING_DATA: return "WAITING_DATA";
        case PS_INIT_COMPLETE: return "COMPLETE";
        case PS_INIT_FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

/* --- Thread di polling --- */
static void ocem_polling(void *arg) {
    
    char response[128];
    size_t responseSize=128;
    epicsThreadSleep(0.1);
    OCEM_INFO("Debug level: %d (0=errors, 1=info, 2=detail, 3=trace)\n", ocemDebugLevel);

    OCEM_INFO("Starting per-slave initialization for %d power supplies...\n", drv->nSlaves);
    printf("OCEM Polling started - Idle: %.2fs, Active: %.2fs, InitTimeout: %.1fs, MaxRetries: %d\n",
           drv->idlePollingPeriod, drv->activePollingPeriod, 
           drv->initTimeoutS, drv->initMaxRetries);
    
    int cycleCount = 0;
    
    while(drv->running) 
    {
        cycleCount++;
        
        // ============================================
        // PHASE 1: Per-slave initialization
        // ============================================
        if (!drv->allSlavesInitialized) 
        {
            // Process current slave being initialized
            if (drv->currentInitSlaveIdx < drv->nSlaves) 
            {
                int addr = drv->addrList[drv->currentInitSlaveIdx];
                OCEM_Slave *slave = &drv->slaves[addr];
                
                switch (slave->initState) 
                {
                    case PS_INIT_NOT_STARTED:
                        // First step: set thresholds to max to stop spontaneous messages
                        OCEM_INFO("[PS%d] === Starting initialization (retry %d/%d) ===\n", 
                                 addr, slave->initRetryCount + 1, drv->initMaxRetries);
                        OCEM_INFO("[PS%d] Setting thresholds to max to stop spontaneous messages...\n", addr);
                        // TH I0 = current threshold, TH I1 = voltage threshold
                        // Set to maximum (999999) to stop COR/TEN messages
                        send_command(drv, addr, "TH I0 999999", response, responseSize);
                        epicsThreadSleep(0.05);
                        send_command(drv, addr, "TH I1 999999", response, responseSize);
                        OCEM_INFO("[PS%d] Draining FIFO before RMT...\n", addr);
                        slave->initState = PS_INIT_DRAINING_FIFO;
                        break;
                    
                    case PS_INIT_DRAINING_FIFO:
                    {
                        // Poll until FIFO is empty (returns 1 = EOT)
                        epicsMutexLock(drv->ioLock);
                        int ret = poll_request(drv, slave, response, responseSize);
                        if (ret == 0) {
                            // Got data - discard it and continue polling
                            OCEM_INFO("[PS%d] DRAIN: discarding '%s'\n", addr, response);
                        } else if (ret == 1) {
                            // FIFO empty - now send RMT
                            OCEM_INFO("[PS%d] FIFO empty - sending RMT\n", addr);
                            epicsMutexUnlock(drv->ioLock);
                            send_command(drv, addr, "RMT", response, responseSize);
                            epicsTimeGetCurrent(&slave->initStartTime);
                            slave->initState = PS_INIT_WAITING_DATA;
                            break;
                        }
                        // ret < 0 means error - try again next cycle
                        epicsMutexUnlock(drv->ioLock);
                        break;
                    }
                        
                    case PS_INIT_WAITING_DATA:
                    {
                        // Poll for data and check if all required data received
                        epicsMutexLock(drv->ioLock);
                        int ret = poll_request(drv, slave, response, responseSize);
                        if (ret == 0) {
                            OCEM_INFO("[PS%d] INIT response: '%s'\n", addr, response);
                            parseMultiReply(response);
                        }
                        epicsMutexUnlock(drv->ioLock);
                        
                        // Check if all required data received
                        if (ocem_isSlaveInitComplete(slave)) {
                            // Read IMAX/VMAX from database records
                            ocem_readImaxVmax(slave);
                            
                            OCEM_INFO("[PS%d] === Initialization COMPLETE ===\n", addr);
                            OCEM_INFO("[PS%d]   VER='%s'\n", addr, slave->version);
                            OCEM_INFO("[PS%d]   PRG: currentPrgH=%d, voltagePrgH=%d\n", 
                                     addr, slave->currentPrgH, slave->voltagePrgH);
                            OCEM_INFO("[PS%d]   STA='%s', COR='%s', TEN='%s'\n", 
                                     addr, slave->status, slave->current, slave->voltage);
                            OCEM_INFO("[PS%d]   IMAX=%.1f, VMAX=%.1f\n", addr, slave->IMAX, slave->VMAX);
                            
                            // Set thresholds if configured
                            if (slave->currentThresholdA > 0) {
                                ocem_sendThreshold(slave, 0, slave->currentThresholdA);
                            }
                            if (slave->voltageThresholdV > 0) {
                                ocem_sendThreshold(slave, 1, slave->voltageThresholdV);
                            }
                            
                            // Transition UNIMAG to OK state
                            slave->unimag.state = UNIMAG_OK;
                            slave->unimag.prgInitialized = 1;
                            strcpy(slave->unimag.statusMsg, "Ready");
                            scanIoRequest(slave->ioscanUnimag);
                            
                            slave->initState = PS_INIT_COMPLETE;
                            drv->currentInitSlaveIdx++;  // Move to next slave
                        }
                        else {
                            // Check timeout
                            epicsTimeStamp now;
                            epicsTimeGetCurrent(&now);
                            double elapsed = epicsTimeDiffInSeconds(&now, &slave->initStartTime);
                            
                            if (elapsed >= drv->initTimeoutS) {
                                // Timeout - retry or fail
                                slave->initRetryCount++;
                                if (slave->initRetryCount < drv->initMaxRetries) {
                                    OCEM_ERR("[PS%d] Init timeout (%.1fs) - retrying (%d/%d)\n", 
                                             addr, elapsed, slave->initRetryCount, drv->initMaxRetries);
                                    OCEM_INFO("[PS%d]   Missing: VER=%d PRG=%d STA=%d COR=%d TEN=%d\n",
                                             addr, slave->hasVER, slave->hasPRG, slave->hasSTA, 
                                             slave->hasCOR, slave->hasTEN);
                                    slave->initState = PS_INIT_NOT_STARTED;  // Retry
                                } else {
                                    OCEM_ERR("[PS%d] === Initialization FAILED after %d retries ===\n", 
                                             addr, drv->initMaxRetries);
                                    slave->initState = PS_INIT_FAILED;
                                    slave->unimag.state = UNIMAG_NOT_REACHED;
                                    strcpy(slave->unimag.statusMsg, "Init failed");
                                    scanIoRequest(slave->ioscanUnimag);
                                    drv->currentInitSlaveIdx++;  // Move to next slave
                                }
                            } else {
                                OCEM_TRACE("[PS%d] Waiting for data... %.1fs/%.1fs (VER=%d PRG=%d STA=%d COR=%d TEN=%d)\n",
                                          addr, elapsed, drv->initTimeoutS,
                                          slave->hasVER, slave->hasPRG, slave->hasSTA, 
                                          slave->hasCOR, slave->hasTEN);
                            }
                        }
                        break;
                    }
                    
                    case PS_INIT_COMPLETE:
                    case PS_INIT_FAILED:
                        // Already processed - move to next
                        drv->currentInitSlaveIdx++;
                        break;
                        
                    default:
                        slave->initState = PS_INIT_NOT_STARTED;
                        break;
                }
                
                // Fast polling during init
                epicsThreadSleep(drv->initPollPeriod);
                continue;  // Skip normal polling during init phase
            }
            else {
                // All slaves processed
                drv->allSlavesInitialized = 1;
                int successCount = 0, failCount = 0;
                for (int i = 0; i < drv->nSlaves; i++) {
                    OCEM_Slave *s = &drv->slaves[drv->addrList[i]];
                    if (s->initState == PS_INIT_COMPLETE) successCount++;
                    else if (s->initState == PS_INIT_FAILED) failCount++;
                }
                OCEM_INFO("=== All slaves initialized: %d OK, %d FAILED ===\n", 
                         successCount, failCount);
            }
        }
        
        // ============================================
        // PHASE 2: Normal polling (after init complete)
        // ============================================
        
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
                
                // Skip failed slaves in normal polling
                if (slave->initState == PS_INIT_FAILED) continue;
                
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
                    // FIFO empty - use ratio-based polling to balance SL and SA queries
                    epicsTimeStamp now;
                    epicsTimeGetCurrent(&now);
                    int didQuery = 0;
                    
                    // Both state and analog polling must be enabled for ratio balancing
                    if (slave->pollStateEnabled && slave->pollAnalogEnabled) {
                        // Use poll ratio to decide whether to poll SA or SL
                        double pollInterval = slave->pollStateIntervalS;  // Use state interval as base
                        if (pollInterval <= 0) pollInterval = 1.0;
                        
                        // Check if enough time has elapsed since last poll of either type
                        double elapsedState = epicsTimeDiffInSeconds(&now, &slave->lastForceStateTime);
                        double elapsedAnalog = epicsTimeDiffInSeconds(&now, &slave->lastForceOperatingTime);
                        double minElapsed = (elapsedState < elapsedAnalog) ? elapsedState : elapsedAnalog;
                        
                        if (minElapsed >= pollInterval) {
                            // Time to poll - use ratio to decide which type
                            if (shouldPollAnalog(slave)) {
                                OCEM_INFO("[PS%d] Polling SA (ratio=%.2f, A=%d S=%d, state=%s)\n", 
                                          slave->addr, getPollRatio(slave),
                                          slave->pollAnalogCount, slave->pollStateCount,
                                          unimag_getStateName(slave->unimag.state));
                                select_command(drv, slave, "SA", response, responseSize);
                                slave->lastForceOperatingTime = now;
                                slave->pollAnalogCount++;
                                didQuery = 1;
                            } else {
                                OCEM_INFO("[PS%d] Polling SL (ratio=%.2f, A=%d S=%d, state=%s)\n", 
                                          slave->addr, getPollRatio(slave),
                                          slave->pollAnalogCount, slave->pollStateCount,
                                          unimag_getStateName(slave->unimag.state));
                                select_command(drv, slave, "SL", response, responseSize);
                                slave->lastForceStateTime = now;
                                slave->pollStateCount++;
                                didQuery = 1;
                            }
                        }
                    } else {
                        // Fallback: original behavior when only one type is enabled
                        // Check if SL (state) query is due
                        if (slave->pollStateEnabled) {
                            double slQueryInterval = slave->pollStateIntervalS;
                            if (slQueryInterval <= 0) slQueryInterval = 1.0;
                            
                            double elapsed = epicsTimeDiffInSeconds(&now, &slave->lastForceStateTime);
                            if (elapsed >= slQueryInterval) {
                                OCEM_INFO("[PS%d] Polling SL (%.1fs elapsed, interval=%.1fs)\n", 
                                          slave->addr, elapsed, slQueryInterval);
                                select_command(drv, slave, "SL", response, responseSize);
                                slave->lastForceStateTime = now;
                                didQuery = 1;
                            }
                        }
                        
                        // Check if SA (analog) query is due (only if we didn't just do SL)
                        if (!didQuery && slave->pollAnalogEnabled) {
                            double saQueryInterval = slave->pollAnalogIntervalS;
                            if (saQueryInterval <= 0) saQueryInterval = 1.0;
                            
                            double elapsed = epicsTimeDiffInSeconds(&now, &slave->lastForceOperatingTime);
                            if (elapsed >= saQueryInterval) {
                                OCEM_INFO("[PS%d] Polling SA (%.1fs elapsed, interval=%.1fs)\n", 
                                          slave->addr, elapsed, saQueryInterval);
                                select_command(drv, slave, "SA", response, responseSize);
                                slave->lastForceOperatingTime = now;
                                didQuery = 1;
                            }
                        }
                    }
                    
                    if (!didQuery) {
                        OCEM_DETAIL("[PS%d] FIFO empty\n", slave->addr);
                    }
                }
                
                // Process UNIMAG state machine
                unimag_process(drv, slave);
                
                epicsMutexUnlock(drv->ioLock);
                
                // Inter-slave delay to prevent RS485 bus collisions
                // Critical for multi-slave setups - allows bus to settle
                epicsThreadSleep(0.08);
            }
        }
        
        epicsThreadSleep(currentPollingPeriod);
    }
}





/* --- Inizializzazione driver --- */
static void ocem_init(const char *port, const char *addrListStr) {
    drv = calloc(1, sizeof(OCEM_Driver));
    drv->pollingPeriodParam = -1;
    
    
    drv->port = strdup(port);
    drv->running = 1;
    // Default polling rates
    drv->idlePollingPeriod = 1.0;     // 1s when idle (just monitoring)
    drv->activePollingPeriod = 0.1;   // 100ms when commands active
    drv->commandActiveTimeout = 5.0;  // Stay in active mode for 5s after last command
    
    // Initialization phase configuration
    drv->initTimeoutS = 5.0;          // 5s timeout per PS before retry
    drv->initMaxRetries = 3;          // 3 retries before marking PS failed
    drv->initPollPeriod = 0.1;        // Fast polling during init
    drv->currentInitSlaveIdx = 0;     // Start with first slave
    drv->allSlavesInitialized = 0;    // Not yet initialized
    
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
    int nSlaves = parseIntList(addrListStr, addrList, MAX_SLAVE);
    if (nSlaves <= 0) {
        errlogPrintf("ocemInit: no valid addresses in list '%s'\n", addrListStr ? addrListStr : "(null)");
        return;
    }
    drv->nSlaves = nSlaves;
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
        drv->slaves[addr].recordPrefix[0] = '\0';  // Will be set on first record read
        drv->slaves[addr].IMAX = -1;
        drv->slaves[addr].VMAX = -1;
        
        // Per-slave initialization state
        drv->slaves[addr].initState = PS_INIT_NOT_STARTED;
        drv->slaves[addr].initRetryCount = 0;
        drv->slaves[addr].hasVER = 0;
        drv->slaves[addr].hasPRG = 0;
        drv->slaves[addr].hasSTA = 0;
        drv->slaves[addr].hasCOR = 0;
        drv->slaves[addr].hasTEN = 0;
        strcpy(drv->slaves[addr].alarms, "NO ");  // Initialize to no alarms
        
        // Polling configuration - disabled by default
        // Use database macros POLL_STATE and POLL_ANALOG to enable
        drv->slaves[addr].pollStateEnabled = 0;
        drv->slaves[addr].pollAnalogEnabled = 0;
        drv->slaves[addr].pollStateIntervalS = 1.0;
        drv->slaves[addr].pollAnalogIntervalS = 1.0;
        
        // Poll ratio configuration defaults
        // ratio = analog polls per state poll
        // E.g., ratio=2.0 means 2 analog, 1 state per 3 cycles
        drv->slaves[addr].pollRatioNormal = 2.0;    // ON: 2 analog, 1 state
        drv->slaves[addr].pollRatioSet = 5.0;       // GOING_TO_SET: 5 analog, 1 state
        drv->slaves[addr].pollRatioState = 0.33;    // CHANGE_POL/WAIT_POL: 1 analog, 3 state
        drv->slaves[addr].pollRatioBalanced = 1.0;  // ZERO_STBY: 1 analog, 1 state
        drv->slaves[addr].pollCycleCounter = 0;
        drv->slaves[addr].pollAnalogCount = 0;
        drv->slaves[addr].pollStateCount = 0;
        
        // Set default PRG values so current/voltage can be calculated immediately
        // These will be updated when actual PRG data is received from polling
        // I0=65535 (current input), I1=255 (voltage input), I2-I4=0 (unused)
        // O0=65535 (current setpoint), O1=4095 (rampup), O2=4095 (rampdown), O3-O4=0
        drv->slaves[addr].currentPrgL = 0;
        drv->slaves[addr].currentPrgH = 65535;  // Default I0/O0 max
        drv->slaves[addr].voltagePrgL = 0;
        drv->slaves[addr].voltagePrgH = 4095;   // Default O1 max (ramp)
        
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
static const iocshArg initArg1 = {"addrListStr", iocshArgString};
static const iocshArg *initArgs[2] = {&initArg0, &initArg1};
static const iocshFuncDef initFuncDef = {"ocemInit", 2, initArgs};
static void initCall(const iocshArgBuf *args) {
    //printf("initCall ha ricevuto %s",args[1].sval)
    ocem_init(args[0].sval, args[1].sval);
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

/* --- Set init timeout/retries command --- */
static void ocem_setInitParams(double timeoutS, int maxRetries)
{
    if (!drv) {
        errlogPrintf("ocemSetInitParams: driver not initialized\n");
        return;
    }
    if (timeoutS >= 0) drv->initTimeoutS = timeoutS;
    if (maxRetries >= 0) drv->initMaxRetries = maxRetries;
    
    printf("OCEM Init params: timeout=%.1fs, maxRetries=%d\n",
           drv->initTimeoutS, drv->initMaxRetries);
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

static const iocshArg initParamArg0 = {"timeoutS", iocshArgDouble};
static const iocshArg initParamArg1 = {"maxRetries", iocshArgInt};
static const iocshArg *initParamArgs[2] = {&initParamArg0, &initParamArg1};
static const iocshFuncDef initParamsFuncDef = {"ocemSetInitParams", 2, initParamArgs};
static void initParamsCall(const iocshArgBuf *args) {
    ocem_setInitParams(args[0].dval, args[1].ival);
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
    iocshRegister(&initParamsFuncDef, initParamsCall);
    iocshRegister(&debugFuncDef, debugCall);
}
epicsExportRegistrar(drvOCEMRegister);
