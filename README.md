# OCEM E642 Power Supply IOC

An EPICS IOC for controlling OCEM E642 power supplies over RS485/Ethernet communication.

## Overview

This IOC provides a high-level interface to OCEM E642 power supplies using the UNIMAG state machine for safe magnet operation. It supports:

- Multi-slave power supply control via RS485 over Ethernet
- Automatic initialization and parameter discovery
- Safe current ramping with polarity change protection  
- Configurable polling rates and timeout handling
- Comprehensive error handling and retry logic
- Real-time status monitoring with EPICS I/O Intr scanning

## Architecture

### UNIMAG State Machine

The IOC implements a high-level state machine for safe magnet operation:

| State | Description | Typical Use |
|-------|-------------|-------------|
| `UNIMAG_INIT` (0) | Waiting for PRG parameter initialization | Startup phase |
| `UNIMAG_OK` (1) | Ready to accept new commands | Normal operation |
| `UNIMAG_NOT_REACHED` (2) | Setpoint/state not reached (MAJOR alarm) | Error condition |
| `UNIMAG_ZERO_STBY` (3) | Ramping current to zero before state change | Safety transition |
| `UNIMAG_CHANGE_POL` (4) | Waiting for STANDBY before polarity change | Polarity reversal |
| `UNIMAG_WAIT_POL` (5) | Waiting for polarity change to complete | Polarity reversal |
| `UNIMAG_GOING_TO_SET` (6) | Ramping toward requested setpoint | Active ramping |

### Channel States

The power supply channel can be in these hardware states (exposed via `STATE_RB` mbbi record):

| Value | State | Description | Severity |
|-------|-------|-------------|----------|
| 0 | `OFF` | Magnet is powered off | NO_ALARM |
| 1 | `ON` | Magnet is powered and active (ATT) | NO_ALARM |
| 2 | `STANDBY` | Magnet is in standby mode (STB) | NO_ALARM |
| 3 | `FAULT` | Internal power-supply fault | MAJOR |
| 4 | `EXT_INTLK` | External interlock active | MAJOR |
| 5 | `CONN_FAULT` | IOC-to-power-supply communication fault | MAJOR |

A setpoint or state not reached is a software state: `STATE_SM` reports it (`SPNOTSET` /
`STATENOTSET`), `STATE_RB` keeps the state of the supply.

## Configuration Reference

### IOC Shell Commands

Configure these in your `st.cmd` file **before** `iocInit()`:

#### Basic Initialization

```bash
# Initialize the OCEM driver
# Syntax: ocemInit "portName", "addressList"
ocemInit "OCEM_PORT", "0,1,2,3"
```

- **portName**: Asyn port name (configured via drvAsynIPPortConfigure)
- **addressList**: Comma-separated list of slave addresses (0-31)

#### Initialization Parameters

```bash
# Configure per-slave initialization parameters
# Syntax: ocemSetInitParams timeoutS, maxRetries
ocemSetInitParams 20.0, 3
```

- **timeoutS**: Timeout for each PS initialization before retry (default: 5s)
- **maxRetries**: Maximum retries before marking PS as failed (default: 3)

**Note**: Each power supply must receive VER, PRG, STA, COR, TEN data during initialization.

#### Polling Configuration

```bash
# Configure adaptive polling rates
# Syntax: ocemSetPollingRates idlePeriod, activePeriod, activeTimeout
ocemSetPollingRates 1.0, 0.1, 5.0
```

- **idlePeriod**: Polling interval when no commands are active (seconds, default: 1.0)
- **activePeriod**: Polling interval when commands are being processed (seconds, default: 0.1)  
- **activeTimeout**: Time to stay in active mode after last command (seconds, default: 5.0)

#### Debug Level

```bash
# Set debug verbosity
# Syntax: ocemSetDebug level
ocemSetDebug 2
```

Debug levels:
- **0**: Errors only (default)
- **1**: Basic info (commands sent, polling cycles)
- **2**: Protocol details (ENQ/ACK/NAK, raw bytes) 
- **3**: Full trace (every function call, all data)

#### Slave Names

```bash
# Give a slave a logical name, used in every log line
# Syntax: ocemSetName addr, "name"   (call after ocemInit)
ocemSetName 5, "DHPTT001"
```

Log lines identify a slave by its bus address (`[PS5]`). With a name set they read
`[DHPTT001/PS5]`, so a log covering several supplies can be followed without looking up
which address is which. Slaves without a name keep the plain `[PS<addr>]` tag. The address
must be one of those passed to `ocemInit`. In ibek the `E642` entity calls this
automatically with its `ADDR` and `R` parameters.

### Database Record Parameters

Configure these when loading the database template:

```bash
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=SYS:PS,R=PS01,ADDR=0,IMAX=100,VMAX=25,SET_TOLERANCE=1.0,ZERO_TOLERANCE=0.5,SET_TIMEOUT_S=30,CURRENT_THRESHOLD=5,VOLTAGE_THRESHOLD=1,POLL_STATE=1,POLL_ANALOG=1")
```

#### Mandatory Parameters

| Parameter | Description | Example |
|-----------|-------------|---------|
| `P` | PV name prefix | `"BTF:MAG:OCEME642"` |
| `R` | Record suffix | `"QUATB102"` |
| `ADDR` | Slave address (0-31) | `0` |
| `IMAX` | Maximum current rating (A) | `100` |
| `VMAX` | Maximum voltage rating (V) | `25` |

#### UNIMAG Configuration

| Parameter | Description | Default | Units |
|-----------|-------------|---------|-------|
| `SET_TOLERANCE` | Current setpoint tolerance | 1.0 | Amperes |
| `ZERO_TOLERANCE` | Zero current tolerance | 0.5 | Amperes |
| `SET_TIMEOUT_S` | Setpoint timeout (resets on progress) | 30 | Seconds |

**Important**: The timeout timer resets whenever the power supply makes progress toward the target current.

#### Threshold Configuration

| Parameter | Description | Special Values |
|-----------|-------------|----------------|
| `CURRENT_THRESHOLD` | Current change threshold for unsolicited messages | -1 = auto (5% of IMAX) |
| `VOLTAGE_THRESHOLD` | Voltage change threshold for unsolicited messages | -1 = auto (5% of VMAX) |

#### Basic Polling

| Parameter | Description | Values |
|-----------|-------------|---------|
| `POLL_STATE` | Enable periodic state (SL) polling | 0=disabled, 1=enabled |
| `POLL_ANALOG` | Enable periodic analog (SA) polling | 0=disabled, 1=enabled |

#### Advanced Poll Ratio Configuration

When both `POLL_STATE=1` and `POLL_ANALOG=1`, the IOC uses intelligent ratio-based polling that adapts to the UNIMAG state:

| UNIMAG State | Default Ratio | Description |
|--------------|---------------|-------------|
| Normal (OK, INIT) | 2.0 | 2 analog polls per 1 state poll |
| GOING_TO_SET | 5.0 | 5 analog polls per 1 state poll (more current monitoring) |
| CHANGE_POL, WAIT_POL | 0.33 | 1 analog poll per 3 state polls (more state monitoring) |
| ZERO_STBY | 1.0 | 1 analog poll per 1 state poll (balanced) |

**Runtime Poll Ratio Configuration** (via ao records):
```bash
# Create records to tune poll ratios at runtime
record(ao, "$(P):$(R):POLL_RATIO_NORMAL") {
    field(DTYP, "OCEME642")
    field(OUT, "@$(ADDR) POLL_RATIO_NORMAL")
    field(VAL, "2.0")
}

record(ao, "$(P):$(R):POLL_RATIO_SET") {
    field(DTYP, "OCEME642") 
    field(OUT, "@$(ADDR) POLL_RATIO_SET")
    field(VAL, "5.0")
}

record(ao, "$(P):$(R):POLL_RATIO_STATE") {
    field(DTYP, "OCEME642")
    field(OUT, "@$(ADDR) POLL_RATIO_STATE") 
    field(VAL, "0.33")
}

record(ao, "$(P):$(R):POLL_RATIO_BALANCED") {
    field(DTYP, "OCEME642")
    field(OUT, "@$(ADDR) POLL_RATIO_BALANCED")
    field(VAL, "1.0")
}
```

## UNIMAG Interface PV Reference

The UNIMAG interface provides a complete set of PVs for controlling and monitoring power supplies. All PV names follow the pattern `$(P):$(R):PVNAME`.

### Main Control PVs (Setpoints)

| PV Name | Type | Description |
|---------|------|-------------|
| `CURRENT_SP` | ao | **Current setpoint in Amperes** (signed, main control interface) |
| `STATE_SP` | mbbo | **State setpoint**: OFF(0), ON(1), STANDBY(2), RESET(3) |
| `SetON` | stringout | Direct ON command (low-level) |
| `SetSTANDBY` | stringout | Direct STANDBY command (low-level) |
| `SetPolarity` | stringout | Direct polarity command: "POS", "NEG", "OPN" (low-level) |
| `RESET_ALARMS` | stringout | Clear alarms command |

### Main Status PVs (Readbacks)

| PV Name | Type | Description |
|---------|------|-------------|
| `STATE_RB` | mbbi | **Channel state**: OFF/ON/STANDBY/FAULT/EXT_INTLK/CONN_FAULT |
| `CURRENT_RB` | ai | **Current readback in Amperes** (signed with polarity) |
| `CURRENT_SP_RB` | ai | Current setpoint readback in Amperes |
| `Polarity` | stringin | Polarity string: "POS", "NEG", "OPN" |
| `IntPolarity` | longin | Numeric polarity: +1, -1, 0 |
| `Status` | stringin | Raw status string from PS |
| `VERSION` | stringin | Power supply firmware version |

### UNIMAG State Machine PVs

| PV Name | Type | Description |
|---------|------|-------------|
| `SM_STATE_RB` | mbbi | **State machine state**: INIT/OK/NOT_REACHED/ZERO_STBY/CHANGE_POL/WAIT_POL/GOING_TO_SET/GOING_TO_STATE |
| `SM_STATE_RAW` | longin | Raw state machine value (0-7) |
| `STATE_SM` | mbbi | **Unified UNIMAG state machine**: READY/GOTOSET/GOTOSTATE/GOZERO/GOPOS/GONEG/SPNOTSET (MINOR)/STATENOTSET (MAJOR). `STATE_SP` = `RESET` brings it back to READY |
| `STATE_SM_RAW` | longin | Raw `STATE_SM` value (0-7) |
| `BUSY_RB` | bi | **Busy indicator**: Idle/Busy (MINOR alarm when busy) |
| `BUSY_RAW` | longin | Raw busy value |
| `STATUS_MSG` | stringin | Human-readable status message |
| `RETRY_COUNT` | longin | Retries of the current command |
| `RETRY` | longout | Times a command not applied within `set_timeout_s` is sent again (macro `RETRY`, default 3) |

### Command State Machine PVs

| PV Name | Type | Description |
|---------|------|-------------|
| `CMD_STATE` | mbbi | Command state: IDLE/SENDING/WAIT_ACK/VERIFYING/DONE/NOT_REACHED |
| `CMD_STATUS` | stringin | Command status message |
| `LAST_CMD` | stringin | Last command sent to PS |

### Analog Measurements

| PV Name | Type | Description |
|---------|------|-------------|
| `VOLTAGE_RB` | calc | Voltage readback in Volts |
| `VoltageStr` | stringin | Raw voltage string from PS |
| `CurrentStr` | stringin | Raw current string from PS |

### Alarm PVs

| PV Name | Type | Description |
|---------|------|-------------|
| `Alarms` | stringin | Raw alarm string from PS (e.g., "01 03 24" or "NO ") |
| `HAS_ALARM` | bi | Alarm indicator: OK/ALARM (MAJOR when alarm) |
| `HAS_ALARM_RAW` | longin | Raw alarm presence (0/1) |
| `ALARM_MASK` | longin | 32-bit alarm bitmask (bit N = alarm N+1) |

### Configuration PVs

| PV Name | Type | Description | Default |
|---------|------|-------------|---------|
| `set_tolerance` | ao | Setpoint verification tolerance (A) | `$(SET_TOLERANCE)` |
| `zero_tolerance` | ao | Zero current tolerance (A) | `$(ZERO_TOLERANCE)` |
| `set_timeout_s` | ao | Setpoint timeout in seconds | `$(SET_TIMEOUT_S)` |
| `init_timeout_s` | ao | Init timeout before retry (s) | 60 |

### Threshold Configuration PVs

| PV Name | Type | Description |
|---------|------|-------------|
| `THRESHOLD:current` | ao | Current change threshold for unsolicited messages (A) |
| `THRESHOLD:voltage` | ao | Voltage change threshold for unsolicited messages (V) |

### Polling Configuration PVs

| PV Name | Type | Description | Default |
|---------|------|-------------|---------|
| `POLL_STATE` | ao | Enable SL polling (0/1) | `$(POLL_STATE)` |
| `POLL_ANALOG` | ao | Enable SA polling (0/1) | `$(POLL_ANALOG)` |
| `POLL_STATE_INTERVAL` | ao | SL poll interval (s) | 1.0 |
| `POLL_ANALOG_INTERVAL` | ao | SA poll interval (s) | 1.0 |

### PRG Data PVs (Initialization Parameters)

| PV Name | Type | Description |
|---------|------|-------------|
| `PRG:initialized` | longin | PRG init status (0=waiting, 1=ready) |
| `PRG:current_max` | longin | Current range max (raw DAC counts) |
| `PRG:current_min` | longin | Current range min (raw DAC counts) |
| `PRG:voltage_max` | longin | Voltage range max (raw DAC counts) |
| `PRG:voltage_min` | longin | Voltage range min (raw DAC counts) |
| `PRG:imax` | ai | Max current setting (A) |
| `PRG:vmax` | ai | Max voltage setting (V) |
| `IMAX` | ai | Maximum current rating (A) |
| `VMAX` | ai | Maximum voltage rating (V) |
| `SELECTOR` | stringin | LOCAL or REMOTE mode |

## Example Configuration

### Basic st.cmd Template

```bash
#!../../bin/linux-x86_64/OCEM_E642Ioc

epicsEnvSet("TOP","../../")

## Register all support components  
dbLoadDatabase "../../dbd/OCEM_E642Support.dbd"
OCEM_E642Ioc_registerRecordDeviceDriver(pdbbase)

# Configure network connection to MOXA terminal server
drvAsynIPPortConfigure("OCEM_PORT", "192.168.1.100:4001")

# Initialize OCEM driver for 4 power supplies
ocemInit "OCEM_PORT", "0,1,2,3"

# Configure initialization (generous timeout for slow PSes)
ocemSetInitParams 30.0, 5

# Configure polling (fast active polling for responsive control)
ocemSetPollingRates 1.0, 0.1, 10.0

# Set moderate debug level
ocemSetDebug 1

# Name the slaves (shown in log lines as [QUAD01/PS0])
ocemSetName 0, "QUAD01"
ocemSetName 1, "QUAD02"

# Load database for each power supply
dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=SYS:MAG,R=QUAD01,ADDR=0,IMAX=50,VMAX=15,SET_TOLERANCE=0.5,ZERO_TOLERANCE=0.2,SET_TIMEOUT_S=60,CURRENT_THRESHOLD=-1,VOLTAGE_THRESHOLD=-1,POLL_STATE=1,POLL_ANALOG=1")

dbLoadRecords("$(TOP)/db/OCEM_E642.db", "P=SYS:MAG,R=QUAD02,ADDR=1,IMAX=100,VMAX=25,SET_TOLERANCE=1.0,ZERO_TOLERANCE=0.5,SET_TIMEOUT_S=120,CURRENT_THRESHOLD=5,VOLTAGE_THRESHOLD=1,POLL_STATE=1,POLL_ANALOG=1")

iocInit()
```

### Network Setup

The IOC typically connects via Ethernet to a serial terminal server (like MOXA NPort):

```
IOC ←→ Ethernet ←→ Terminal Server ←→ RS485 ←→ Power Supplies
```

Configure the terminal server for:
- **Baud rate**: 9600
- **Data bits**: 8
- **Parity**: None  
- **Stop bits**: 1
- **Flow control**: None
- **Protocol**: Raw TCP

## Key PV Examples

For a power supply configured with `P=SYS:MAG` and `R=QUAD01`:

### Main Control Interface
```bash
# Set current to 50 Amperes (handles polarity automatically)
caput SYS:MAG:QUAD01:CURRENT_SP 50.0

# Set current to -25 Amperes (negative polarity)
caput SYS:MAG:QUAD01:CURRENT_SP -25.0

# Turn ON the power supply
caput SYS:MAG:QUAD01:STATE_SP 1

# Put in STANDBY
caput SYS:MAG:QUAD01:STATE_SP 2

# Reset alarms
caput SYS:MAG:QUAD01:RESET_ALARMS.PROC 1
```

### Status Monitoring
```bash
# Read current readback
caget SYS:MAG:QUAD01:CURRENT_RB

# Read channel state (OFF/ON/STANDBY/FAULT/EXT_INTLK/CONN_FAULT/etc.)
caget SYS:MAG:QUAD01:STATE_RB

# Read UNIMAG state machine state (INIT/OK/GOING_TO_SET/etc.)
caget SYS:MAG:QUAD01:SM_STATE_RB

# Check if state machine is busy
caget SYS:MAG:QUAD01:BUSY_RB

# Read human-readable status message
caget SYS:MAG:QUAD01:STATUS_MSG

# Check for alarms
caget SYS:MAG:QUAD01:HAS_ALARM
caget SYS:MAG:QUAD01:Alarms
```

### Configuration at Runtime
```bash
# Adjust setpoint tolerance
caput SYS:MAG:QUAD01:set_tolerance 0.5

# Adjust timeout
caput SYS:MAG:QUAD01:set_timeout_s 120
```

## Safety Features

1. **Automatic Zero Before Polarity Change**: The UNIMAG state machine automatically ramps current to zero before changing polarity to prevent magnet damage.

2. **Progress-Based Timeout**: The setpoint timeout resets whenever the power supply makes measurable progress toward the target, preventing premature timeouts during slow ramps.

3. **Tolerance Checking**: Separate tolerances for setpoint verification and zero current detection prevent oscillation around target values.

4. **Initialization Verification**: Each power supply must successfully provide all required parameters (VER, PRG, STA, COR, TEN) before being considered operational.

5. **Communication Error Handling**: Automatic retry logic with configurable timeouts and maximum retry counts.

## Troubleshooting

### Common Issues

1. **PS Init Failed**: Check network connectivity and RS485 wiring
2. **Setpoint Not Reached**: Verify current limits and ramping parameters  
3. **Communication Timeouts**: Increase initialization timeout or check terminal server configuration
4. **Slow Response**: Adjust polling rates and poll ratios for your application

### Debug Commands

Enable detailed logging to diagnose issues:
```bash
# IOC shell commands (runtime)
ocemSetDebug 3          # Maximum verbosity
ocemSetDebug 0          # Back to errors only
```

Check the IOC log for detailed protocol traces and state machine transitions.

## Files

- **drvOCEM.c**: Main driver implementation with UNIMAG state machine
- **devOCEM.c**: EPICS device support for records
- **drvOCEM.h**: Driver header with state definitions
- **OCEM_E642.db**: Database template with all PV definitions
- **docs/poll_balance.txt**: Detailed poll ratio configuration guide
