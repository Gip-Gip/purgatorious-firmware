The job of the watchdog is to start all the other executables and ensure
they all stay healthy.

# Program Operations

## 1: Program Initialization

### 1.1: Initialize URAP slave
Remove any broken sockets and start the URAP slave. The slave has one
register which is used to signal to other vPLCs that an ESTOP is in effect.

### 1.2: Create logfile for watchdog

### 1.3: Start vPLCs
Starts all vPLCs in the /opt/firmware/active/ directory and stores their
executable paths, socket paths, child process handles and inchashes in a
vector.

#### 1.3.1: Wait for vPLCs to init

## 2: Execution Loop

### 2.1: Check for frozen vPLCs
The primary function of the watchdog is to restart frozen vPLCs and 
sound the alarm if one fails to respond.

#### 2.1.1: Spawn threads to check in on vPLCs
Multitask and check in on all of the vPLCs, making sure the process still
exists and has showed signs of life by incrementing it's inchash.

#### 2.1.2: Delay to ensure all vPLCs have time to update and respond

#### 2.1.3: Check if the threads have finished
If any of the threads are not finished, that means that the process hasn't
responded yet and must be restarted

#### 2.1.4: Check to see if any of the threads threw an error
If a thread threw an error, something's wrong and the vPLC must be
restarted.

#### 2.2: Check the URAP slave for errors
If the URAP slave has encountered any errors, be sure to log them and
sound the alarms.
