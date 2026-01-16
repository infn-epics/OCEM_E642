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
#define MAX_OCEM_RECORDS 500

// Debug levels:
// 0 = Errors only
// 1 = Basic info (commands sent, polling cycles)
// 2 = Detailed protocol (ENQ/ACK/NAK, raw bytes)
// 3 = Full trace (every function entry/exit, all data)
extern int ocemDebugLevel;

// Timestamp helper for debug output
static inline void ocem_debug_timestamp(void) {
    epicsTimeStamp now;
    epicsTimeGetCurrent(&now);
    char timeStr[32];
    epicsTimeToStrftime(timeStr, sizeof(timeStr), "%H:%M:%S.%03f", &now);
    errlogPrintf("[%s] ", timeStr);
}

#define OCEM_DEBUG(level, ...) \
    do { \
        if (ocemDebugLevel >= (level)) { \
            ocem_debug_timestamp(); \
            errlogPrintf(__VA_ARGS__); \
        } \
    } while(0)

// Convenience macros for different levels
#define OCEM_ERR(...)   do { ocem_debug_timestamp(); errlogPrintf(__VA_ARGS__); } while(0)
#define OCEM_INFO(...)  OCEM_DEBUG(1, __VA_ARGS__)
#define OCEM_DETAIL(...) OCEM_DEBUG(2, __VA_ARGS__)
#define OCEM_TRACE(...) OCEM_DEBUG(3, __VA_ARGS__)

// Keep old macro for compatibility
#define errlogPrintf1(...) OCEM_DEBUG(1, __VA_ARGS__)

typedef struct {
    int addr;        // slave address (0..31)
    char var[32];    // "STATUS", "CURRENT", "VOLTAGE", ...
    dbAddr   *linkedAddr;
    dbCommon *linkedRec;
    //struct OCEM_Var *varRef; // puntatore diretto alla variabile nello slave da usare in caso di generalizzazione.
} ocemDpvt;

// Per-slave initialization state
typedef enum {
    PS_INIT_NOT_STARTED = 0,    // Not yet attempted
    PS_INIT_DRAINING_FIFO,      // Draining stale data from FIFO before RMT
    PS_INIT_SENDING_RMT,        // About to send RMT command
    PS_INIT_WAITING_DATA,       // RMT sent, waiting for response data
    PS_INIT_COMPLETE,           // All required data received
    PS_INIT_FAILED              // Failed after max retries
} PSInitState;

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

// Command state machine - tracks what IOC is doing after a command
typedef enum {
    CMD_IDLE = 0,           // No command pending
    CMD_SENDING = 1,        // Command being sent to PS
    CMD_WAIT_ACK = 2,       // Waiting for ACK from PS
    CMD_VERIFYING = 3,      // Verifying command was applied
    CMD_DONE = 4,           // Command completed successfully
    CMD_NOT_REACHED = 5     // Command failed or timeout
} CmdState;

// ============================================
// UNIMAG State Machine - High Level Interface
// ============================================

// Channel state values (STATE_RB / STATE_SP)
typedef enum {
    CH_OFF = 0,         // Magnet is powered off
    CH_ON = 1,          // Magnet is powered and active (ATT)
    CH_STANDBY = 2,     // Magnet is in standby mode (STB)
    CH_FAULT = 3,       // A fault condition has been detected
    CH_RESET = 4        // Reset operation in progress
} ChannelState;

// UNIMAG state machine state (UNIMAG_STATE_RB)
typedef enum {
    UNIMAG_INIT = 0,            // Waiting for PRG initialization
    UNIMAG_OK = 1,              // Ready to accept new commands
    UNIMAG_NOT_REACHED = 2,     // Requested state/current not reached (MAJOR alarm)
    UNIMAG_ZERO_STBY = 3,       // Ramping down to zero current
    UNIMAG_CHANGE_POL = 4,      // Waiting for STANDBY before polarity change
    UNIMAG_WAIT_POL = 5,        // Waiting for polarity change to complete
    UNIMAG_GOING_TO_SET = 6     // Ramping toward requested setpoint
} UnimagState;

// UNIMAG configuration parameters (per slave)
typedef struct {
    double setTolerance;        // Tolerance for current setpoint verification (A)
    double zeroTolerance;       // Tolerance for zero current check (A)
    double setTimeoutS;         // Timeout for reaching setpoint (seconds)
    double initTimeoutS;        // Timeout for initialization before retry (seconds)
    int maxRetries;             // Max retries before STATE_NOT_REACHED
    double retryDelay;          // Delay between retries (seconds)
} UnimagConfig;

// UNIMAG state machine context (per slave)
typedef struct {
    UnimagState state;          // Current UNIMAG state
    int busy;                   // 1 = state machine active, 0 = idle
    int retryCount;             // Current retry counter
    double targetCurrent;       // Requested current setpoint (signed, in Amperes)
    int targetPolarity;         // Target polarity: +1, -1, or 0
    ChannelState targetState;   // Requested channel state
    epicsTimeStamp stepStartTime;   // Timestamp when current step started
    epicsTimeStamp initStartTime;   // Timestamp when initialization started
    epicsTimeStamp lastProgressTime; // Last time current was moving toward target
    double lastCurrentRB;           // Last current readback for progress check
    int prgInitialized;         // 1 = PRG data received, 0 = waiting
    char statusMsg[80];         // Human-readable status message
} UnimagContext;

typedef struct {
    int addr;           // indirizzo slave
    char recordPrefix[80];  // Record name prefix (e.g., "BTF:MAG:OCEME642:QUATB102")
    char lastSelCommand[32];
    char status[40];
    char current[40];
    char voltage[40];
    char polarity[40];
    char alarms[40];        // Raw alarm string from PS (e.g., "01 03 24" or "NO ")
    int hasAlarm;           // 1 if any alarm is active, 0 if no alarms
    epicsUInt32 alarmMask;  // Bitmask of active alarms (bit N = alarm N+1)
    char selector[40];
    char version[80];   // Firmware version string from VER command
    double IMAX;
    double VMAX;
    
    int  currentPrgH,currentPrgL;
    int  voltagePrgH,voltagePrgL;
    int  unimagStatus,integerPolarity;
    double requestedCurrent; // da record stringout
    int requestedPolarity;   // +1 o -1
    OcemState Ostate;
    int hasPendingCommand;   // flag for active command processing
    epicsTimeStamp lastCommandTime; // timestamp of last command
    
    // Command state machine (low level)
    CmdState cmdState;              // Current command state
    char cmdStatusMsg[64];          // Human-readable status message
    char lastCommand[40];           // Last command sent
    int cmdRetryCount;              // Retry counter
    int cmdVerifyCount;             // Verify poll counter
    epicsTimeStamp cmdStartTime;    // When command was initiated
    
    // UNIMAG high-level state machine
    UnimagConfig unimagCfg;         // Configuration parameters
    UnimagContext unimag;           // State machine context
    ChannelState channelState;      // Current channel state (derived from status)
    double currentRB;               // Current readback in Amperes (calculated)
    double voltageRB;               // Voltage readback in Volts (calculated)
    double currentSP;               // Current setpoint in Amperes (from user)
    
    // Threshold for unsolicited messages (TH command)
    double currentThresholdA;       // Current threshold in Amperes (sent to PS)
    double voltageThresholdV;       // Voltage threshold in Volts (sent to PS)
    int thresholdsSent;             // Flag: thresholds have been sent to PS
    
    // Per-slave initialization state machine
    PSInitState initState;          // Current initialization state
    int initRetryCount;             // Number of RMT retries
    epicsTimeStamp initStartTime;   // When init started for this PS
    int hasVER;                     // 1 = VER received
    int hasPRG;                     // 1 = PRG data received (currentPrgH > 0)
    int hasSTA;                     // 1 = STA received (status is set)
    int hasCOR;                     // 1 = COR received (current value)
    int hasTEN;                     // 1 = TEN received (voltage value)
    
    // Polling-based query configuration (when FIFO is empty)
    // Use these instead of relying on unsolicited messages
    int pollStateEnabled;           // 1 = periodically query SL for state (instead of waiting for unsolicited)
    int pollAnalogEnabled;          // 1 = periodically query SA for COR/TEN (instead of waiting for unsolicited)
    double pollStateIntervalS;      // Seconds between SL queries when pollStateEnabled (default 1.0)
    double pollAnalogIntervalS;     // Seconds between SA queries when pollAnalogEnabled (default 1.0)
    epicsTimeStamp lastForceStateTime;   // Timestamp of last forced SL query
    epicsTimeStamp lastForceOperatingTime; // Timestamp of last forced SA query
    
    // Poll ratio configuration: ratio = analog polls per state poll
    // E.g., ratio=2.0 means 2 analog polls for every 1 state poll (3 cycles: A, A, S)
    // ratio=0.33 means 1 analog poll for every 3 state polls (4 cycles: S, S, S, A)
    double pollRatioNormal;         // Ratio when in normal ON state (default 2.0)
    double pollRatioSet;            // Ratio when GOING_TO_SET (default 5.0)
    double pollRatioState;          // Ratio when CHANGE_POL/WAIT_POL (default 0.33)
    double pollRatioBalanced;       // Ratio when ZERO_STBY (default 1.0)
    int pollCycleCounter;           // Counter for determining next poll type
    int pollAnalogCount;            // Analog polls done in current ratio cycle
    int pollStateCount;             // State polls done in current ratio cycle
    
    //IOSCANPVT per notificare record
    IOSCANPVT ioscanStatus;
    IOSCANPVT ioscanCurrent;
    IOSCANPVT ioscanVoltage;
    IOSCANPVT ioscanPolarity;
    IOSCANPVT ioscanAlarms;
    IOSCANPVT ioscanSelector;
    IOSCANPVT ioscanInit;
    IOSCANPVT ioscanCmdState;       // For command state changes
    IOSCANPVT ioscanUnimag;         // For UNIMAG state changes
} OCEM_Slave;

// Command queue entry
typedef struct CmdQueueEntry {
    int slaveAddr;
    char cmd[40];
    struct CmdQueueEntry *next;
} CmdQueueEntry;

typedef struct {
    char *port;              // nome porta seriale
    int nSlaves;
    int addrList[MAX_SLAVE];
    OCEM_Slave slaves[MAX_SLAVE];
    epicsThreadId threadId;
    int running;
    int pollingPeriodParam;
    double idlePollingPeriod;    // polling period when no commands pending (slower)
    double activePollingPeriod;  // polling period when commands are active (faster)
    double commandActiveTimeout; // how long a slave stays in "active" mode after command
    
    // Initialization phase configuration
    double initTimeoutS;         // timeout for each PS init before retry (default 5s)
    int initMaxRetries;          // max retries before marking PS failed (default 3)
    double initPollPeriod;       // polling period during init (fast, default 0.1s)
    int currentInitSlaveIdx;     // index into addrList of slave currently being initialized
    int allSlavesInitialized;    // 1 = all slaves init complete or failed
    
    asynUser *pasynUser;
    asynInterface *pasynInterface;
    asynOctet *pasynOctet;
    epicsMutexId ioLock;
    // Command queue
    CmdQueueEntry *cmdQueueHead;
    CmdQueueEntry *cmdQueueTail;
    epicsMutexId cmdQueueLock;


} OCEM_Driver;

int send_command(OCEM_Driver* drv,int slaveAddress,char* cmd,char*response,size_t responseSize);
OCEM_Slave* findSlave(OCEM_Driver* drv,int slaveAddress);
void pad_value(const char *value, char *output);
int queue_command(OCEM_Driver* drv, int slaveAddr, const char* cmd);
void ocem_setPollingRates(double idlePeriod, double activePeriod, double activeTimeout);

// UNIMAG state machine functions
void unimag_init(OCEM_Slave *slave);
void unimag_setCurrentSP(OCEM_Slave *slave, double currentA);
void unimag_setStateSP(OCEM_Slave *slave, ChannelState state);
void unimag_process(OCEM_Driver *drv, OCEM_Slave *slave);
const char* unimag_getStateName(UnimagState state);
const char* unimag_getChannelStateName(ChannelState state);
int ocem_sendThreshold(OCEM_Slave *slave, int channel, double valueA);
void ocem_setRecordPrefix(OCEM_Slave* slave, const char* recordName);
void ocem_readImaxVmax(OCEM_Slave* slave);

extern  OCEM_Driver *drv;

#endif