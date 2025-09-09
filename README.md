# Automated Table Saw Control System - Stage 1

ESP32-S3 based control system for an automated table saw that performs precision wood cutting operations with integrated transfer arm coordination.

## Overview

This system controls Stage 1 of an automated table saw that:
- Feeds wood pieces into the cutting area
- Performs precise cutting operations with a rotating blade
- Coordinates with a transfer arm system for material handling
- Manages multiple pneumatic clamps for secure wood positioning
- Provides comprehensive error handling and safety systems

## Hardware Components

### Motors
- **Cut Motor (Stepper)**: Controls the cutting blade movement
  - 400 steps/revolution with 20T pulley and 6mm 2GT belt
  - Travel distance: 9.1 inches (configurable via `CUT_TRAVEL_DISTANCE`)
  - Homing switch: Active HIGH with pulldown resistor

- **Feed Motor (Stepper)**: Controls wood feeding mechanism
  - 400 steps/revolution with 20T pulley and 6mm 2GT belt
  - Travel distance: 3.4 inches (configurable via `FEED_TRAVEL_DISTANCE`)
  - Homing sensor: Active LOW with pullup resistor

- **Rotation Servo**: Controls wood piece rotation for angled cuts
  - 24V servo with 270° rotation range
  - Home position: 2° (safety position)
  - Active position: Configurable via `ROTATION_SERVO_ACTIVE_POSITION`

### Pneumatic Clamps
- **Feed Clamp**: Secures wood during feeding operations
- **2x4 Secure Clamp**: Secures 2x4 lumber during cutting
- **Rotation Clamp**: Holds cut pieces for rotation operations

### Sensors & Switches
- **Homing Switches**: Limit switches for motor positioning
- **Wood Detection Sensors**: 
  - `_2x4_PRESENT_SENSOR`: Detects presence of 2x4 lumber
  - `WOOD_SUCTION_CONFIRM_SENSOR`: Confirms wood is grabbed by transfer arm
  - `FIRST_CUT_OR_WOOD_FWD_ONE`: Determines feed operation type
- **Control Switches**:
  - `RELOAD_SWITCH`: Manual reload mode activation
  - `START_CYCLE_SWITCH`: Initiates cutting cycles
  - `MANUAL_FEED_SWITCH`: Manual wood feeding control

### Status LEDs
- **Red LED**: Error/fault indication
- **Yellow LED**: Warning/caution indication  
- **Green LED**: Ready/operation OK indication
- **Blue LED**: Process active indication

## State Machine Overview

The system operates through a comprehensive state machine with the following states:

### 1. STARTUP State
**Purpose**: System initialization and IP address display
- Turns on blue LED to indicate startup/homing
- Displays IP address on serial monitor
- Transitions to HOMING state after 1-second delay

### 2. HOMING State
**Purpose**: Initialize all motors to known positions
- **Step 1**: Blinks blue LED during homing process
- **Step 2**: Homes cut motor (blocking operation with retry on failure)
  - **Cut Motor Homing**: Moves in **negative (-)** direction until home switch triggers (HIGH)
  - **Home Position**: Sets position to **0 steps** when switch activates
- **Step 3**: Retracts feed clamp and homes feed motor
  - **Feed Motor Homing**: Moves in **positive (+)** direction until home sensor triggers (LOW)
  - **Home Position**: Sets position to **3.4 inches (3400 steps)** - 0.5 inches offset from sensor
- **Step 4**: Moves feed motor to travel distance and re-extends feed clamp
- **Step 5**: Sets `isHomed` flag to true
- **Step 6**: Turns off blue LED, turns on green LED
- **Step 7**: Skips servo homing for safety (prevents ramming stuck wood)
- **Step 8**: Transitions to IDLE state

### 3. IDLE State
**Purpose**: Wait for user input and manage system modes
- **Reload Mode Handling**: 
  - When reload switch is ON: Retracts all clamps, turns on blue LED
  - When reload switch is OFF: Re-extends 2x4 secure clamp, turns off blue LED
- **Feed Operations**:
  - Manual feed switch + `FIRST_CUT_OR_WOOD_FWD_ONE` HIGH → `FEED_FIRST_CUT`
  - Manual feed switch + `FIRST_CUT_OR_WOOD_FWD_ONE` LOW → `FEED_WOOD_FWD_ONE`
- **Cutting Cycle Start**:
  - Start cycle switch rising edge OR continuous mode active
  - Must have no wood suction error and start switch must be safe
  - Extends clamps, configures cut motor, transitions to CUTTING

### 4. FEED_FIRST_CUT State
**Purpose**: Two-stage wood feeding for first cut operations
- **First Run**:
  1. Retract feed clamp
  2. Move feed motor to -1.2 inches
  3. Extend feed clamp, retract 2x4 secure clamp
  4. Wait 200ms
  5. Move to travel distance
- **Second Run**:
  1. Retract feed clamp
  2. Move feed motor to -1.2 inches again
  3. Extend feed clamp, retract 2x4 secure clamp
  4. Wait 200ms
  5. Move to travel distance minus 1.4 inches
- **Completion**: Check start cycle switch and transition to CUTTING or IDLE

### 5. FEED_WOOD_FWD_ONE State
**Purpose**: Single-stage wood feeding for forward operations
1. Retract feed clamp
2. Move feed motor to home position
3. Extend feed clamp, retract 2x4 secure clamp
4. Wait 200ms
5. Move to travel distance
6. Check start cycle switch and transition to CUTTING or IDLE

### 6. CUTTING State
**Purpose**: Execute the main cutting operation with safety checks
- **Step 0 (Initialization)**:
  - Extends 2x4 secure clamp and feed clamp
  - Homes rotation servo only if wood is properly grabbed (safety check)
  - Configures cut motor for cutting speed
  - Starts cut motor movement
- **Step 1 (Suction Check)**:
  - Monitors cut motor position
  - At `SUCTION_SENSOR_CHECK_DISTANCE`, verifies wood suction sensor
  - If no suction detected: Stops motors, returns cut motor home, transitions to SUCTION_ERROR
  - If suction OK: Continues to Step 2
- **Step 2 (Cutting Process)**:
  - Activates rotation clamp at `ROTATION_CLAMP_ACTIVATION_POSITION`
  - Activates rotation servo at `ROTATION_SERVO_ACTIVATION_POSITION`
  - Sends transfer arm signal at `TA_SIGNAL_ACTIVATION_POSITION`
  - Monitors cut motor completion
  - On completion: Transitions to appropriate RETURNING state based on wood detection

### 7. RETURNING_YES_2x4 State
**Purpose**: Return sequence when wood is detected (2x4 present)
- **Simultaneous Operations**:
  - Cut motor returns to home position
  - Feed motor executes multi-step return sequence
- **Feed Motor Return Sequence**:
  1. Move to home position
  2. Extend feed clamp
  3. Move to travel distance
  4. Retract feed clamp
  5. Move to home position
  6. Extend feed clamp
  7. Move to travel distance
- **Cut Motor Homing**: Includes recovery mechanisms for failed homing attempts
- **Completion**: Transitions to IDLE or next cutting cycle based on continuous mode

### 8. RETURNING_NO_2x4 State
**Purpose**: Return sequence when no wood is detected
- **Multi-Step Process**:
  1. Move cut motor home and retract 2x4 clamp
  2. Wait for cut motor home, extend feed clamp
  3. Wait for feed motor home, retract feed clamp
  4. Move feed motor to 2.0 inches
  5. Extend feed clamp
  6. **Attention Sequence**: 9 rapid feed clamp extension/retraction movements
  7. Move feed motor to home
  8. Retract feed clamp
  9. Move feed motor to final position
  10. Verify cut home position
- **Completion**: Transitions to IDLE state

### 9. ERROR States
**Purpose**: Handle various error conditions with user acknowledgment

#### SUCTION_ERROR State
- Automatically homes cut motor for safety
- Blinks red LED at defined interval
- Waits for start cycle switch rising edge
- Resets continuous mode and transitions to HOMING

#### Cut_Motor_Homing_Error State
- Blinks red and yellow LEDs alternately
- Stops all motors
- Waits for reload switch acknowledgment
- Transitions to ERROR_RESET

#### ERROR_RESET State
- Handles general error recovery
- Implemented in StateManager.cpp

## Pin Assignments

### Motor Control
- Cut Motor Step: Pin 25
- Cut Motor Direction: Pin 26
- Feed Motor Step: Pin 27
- Feed Motor Direction: Pin 14

### Servo Control
- Rotation Servo: Pin 12

### Sensors & Switches
- Cut Motor Home Switch: Pin 32 (Active HIGH, pulldown)
- Feed Motor Home Sensor: Pin 33 (Active LOW, pullup)
- Reload Switch: Pin 34 (Active HIGH, pulldown)
- Start Cycle Switch: Pin 35 (Active HIGH, pulldown)
- Manual Feed Switch: Pin 36 (Active HIGH, pulldown)
- First Cut/Wood Fwd One: Pin 37 (Active LOW, pullup)
- 2x4 Present Sensor: Pin 38 (Active LOW, pullup)
- Wood Suction Confirm: Pin 39 (Active LOW, pullup)

### Pneumatic Clamps
- Feed Clamp: Pin 40 (HIGH = extend)
- 2x4 Secure Clamp: Pin 41 (HIGH = extend)
- Rotation Clamp: Pin 42 (HIGH = extend)

### Status LEDs
- Red LED: Pin 43
- Yellow LED: Pin 44
- Green LED: Pin 45
- Blue LED: Pin 46

### Communication
- Transfer Arm Signal: Pin 47

## Safety Features

1. **Servo Safety**: Rotation servo is not automatically homed on startup to prevent ramming stuck wood into the blade
2. **Suction Verification**: System verifies wood is properly grabbed before starting cut cycle
3. **Motor Homing Recovery**: Includes incremental movement recovery for failed homing attempts
4. **Electrical Interference Prevention**: 100ms delay between feed clamp extension and cut motor homing
5. **Error Acknowledgment**: All errors require user acknowledgment before system reset
6. **Switch Safety**: Start cycle switch must be cycled after errors to prevent accidental restarts

## Configuration

All timing, speed, and distance parameters are configurable through the `Config/` directory:
- `Config.h`: System configuration constants
- `Pins_Definitions.h`: Hardware pin assignments
- `States_Config.h`: State-specific timing and position constants

## OTA Updates

The system supports Over-The-Air (OTA) updates via WiFi:
- IP address displayed on startup
- Upload protocol: ESPOTA
- Upload port: 192.168.1.214 (configurable)

## Dependencies

- **Platform**: ESP32-S3 DevKitC-1
- **Framework**: Arduino
- **Libraries**:
  - Bounce2 (v2.71): Switch debouncing
  - FastAccelStepper (v0.30.0): Stepper motor control
  - ESP32Servo (v3.0.6): Servo control

## Operation Modes

1. **Manual Mode**: Single cutting cycles initiated by start switch
2. **Continuous Mode**: Automatic cycling between cuts
3. **Reload Mode**: Manual material loading with all clamps retracted
4. **Error Mode**: System halt with LED indication and user acknowledgment required

## State Transitions

```
STARTUP → HOMING → IDLE
IDLE → FEED_FIRST_CUT → CUTTING → RETURNING_YES_2x4 → IDLE
IDLE → FEED_WOOD_FWD_ONE → CUTTING → RETURNING_NO_2x4 → IDLE
IDLE → CUTTING → RETURNING_YES_2x4/RETURNING_NO_2x4 → IDLE
Any State → ERROR States → ERROR_RESET → HOMING → IDLE
```

This system provides a robust, safety-focused automated table saw control solution with comprehensive error handling and user-friendly operation modes.