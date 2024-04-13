Controls all heater zones

# Program Operation

## 1: Program Initialization
### 1.1: Connect to watchdog
It is important that a connection to the watchdog is established early on so
estops flagged by other vPLCs can be read
### 1.2: Aquire GPIO
Aquire all the gpio pins for the SSRs
### 1.3: Initalize URAP secondary
### 1.4: Connect to other vPLCs
Operation is dependant on getting temperatures from the I2C vPLC and power usage
from the Screw vPLC
### 1.5: Initialize mutable variables
There are plenty of mutable variables that need to persist through each cycles,
including PLC structs, power averages, etc.
## 2: Execution loop 
### 2.1: Calculate instant to resume loop
It is necissary, to keep consistancy, to calculate the instant when the next
loop iteration is to start
### 2.2: Aquire lock on registers
While locked the URAP secondary cannot access the registers
### 2.3: Increment the inchash
Incrementing the inchash tells the watchdog that we are alive
### 2.4a: Peform normal operations if estop is not active
#### 2.4a.1: Retrieve values from other vPLCs
We need fresh temperature and power consumption values
#### 2.4a.2: Update target temperature from register values
#### 2.4a.3: Reset/clear integral term if outside of the Ki range
To prevent a massive initial overshoot, the I term is disabled unless the current
temperature is within a specific range 
#### 2.4a.4: Update Kpid values if functional pid is enabled
#### 2.4a.5: Update Kpid values from registers if the reload pid flag is not zero
#### 2.4a.6: Toggle fans on or off
Fans are turned on if we are above the set temperature - 10 degrees, disabled
otherwise
#### 2.4a.7a: Update power values from registers if power override is not zero
#### 2.4a.7b: Otherwise, update power values from PID calcualtions
#### 2.4a.8: Ensure power values are safe and not negative
The PWM can act weirdly if power values are somehow negative, so ensure they never
go negative
#### 2.4a.9: Calculate current usage
##### 2.4a.9.1: Limit power so that total extruder current doesn't exceed limit
Due to the extruder being operated on a 60 amp residential breaker, ensure that
the power limit is never reached
##### 2.4a.9.2: Write corrected power consumption to register
#### 2.4a.10: Average out power in between PWM periods
To fight derivative kickback, we simply average out the calcualted power over
a pwm period
#### 2.4a.11: Control heater PWM
##### 2.4a.11.1a: If start of a new period, calculate on and off duration
Basically, if power is at 100%, the heater stays on for the whole period. If
at 50%, the heater stays on for half the period. Etc. etc.
###### 2.4a.11.1.1: If power override is disabled, write average power to power registers.
The power registers are also used to report the power % of each zone. If
power override is disabled, we should write the average power so the GUI can
display it.
###### 2.4a.11.1.2: Reset power average
###### 2.4a.11.1.3a: If the duration the heater is to be on is greater than the minimum period time, turn on the heater
###### 2.4a.11.1.3b: Otherwise, ensure the heater is off
##### 2.4a.11.1b: Otherwise, if it is time to turn off the heater, turn off the heater.
#### 2.4a.12: Drop lock on registers so secondary can access them
### 2.4b: Otherwise, perform estop functions
#### 2.4b.1: Set target temperature and power registers to zero
We don't want to immediately start heating up once the estop is cleared
#### 2.4b.2: Drop lock on registers so secondary can access them
#### 2.4b.3: Turn on all fans and turn off all heaters so extruder can cool down
#### 2.4b.4: Reopen lost urap connections
Sometimes an estop could be caused by a vPLC crashing, so we need to reopen
connections if that's the case.
### 2.5: Sleep until the loop restarts
