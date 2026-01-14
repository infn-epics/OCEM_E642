Summary of Changes
Driver Changes (Adaptive Polling)
drvOCEM.h:

Added hasPendingCommand and lastCommandTime fields to OCEM_Slave struct
Replaced ocemPollingPeriod with three configurable rates:
idlePollingPeriod (default: 1.0s) - slow polling when no commands active
activePollingPeriod (default: 0.1s) - fast polling during command activity
commandActiveTimeout (default: 5.0s) - how long to stay in active mode after last command
Added ocem_setPollingRates() function declaration
drvOCEM.c:

Added hasActiveCommands() function to detect if any slave has pending commands
Modified queue_command() to mark slaves with pending commands and timestamp
Updated ocem_polling() thread to use adaptive rates based on command activity
Added ocemSetPollingRates IOC shell command for runtime configuration
devOCEM.c:

Updated setPollingPeriod to use idlePollingPeriod instead of old variable
Startup Script
testBtf.cmd:

Added ocemSetPollingRates 1.0, 0.1, 5.0 command with documentation
New OPI Files (Improved v2)
OCEM_serial_v2.bob:

Modern design with grouped sections
Status & Readings panel with units (A, V)
Raw Protocol Values (Debug) section
State Control with colored buttons
Quick Set & Go current control
20-minute strip chart with grid
OCEM_unimag_v2.bob:

Large current display (32pt font)
LED status indicator in title bar
State-colored status display (green=ON, yellow=STANDBY, gray=OFF)
Alarm indicator with color rules
Quick set buttons (0, ±10, ±50, ±100 A)
Compact 10-minute trend chart
Launcher_v2.bob:

Overview table with all 4 devices
Live status, current readback, and LED indicators per device
Buttons to open UNIMAG or Serial Debug for each device
Polling rate information displayed
Polling Behavior
Idle Mode: 1.0s polling - used when no commands are being processed (just monitoring)
Active Mode: 0.1s polling - triggered when commands are queued, provides fast feedback
Auto-switch: Returns to idle mode 5 seconds after last command completes
