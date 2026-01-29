# State Machine Documentation

This document provides an extremely highly detailed and comprehensive explanation of the Finite State Machine (FSM) controlling the wood cutting and processing system. The system is architected using a function-based state manager that coordinates the operations of stepper motors, pneumatic clamps, sensors, and user interface elements (LEDs, switches).

## 1. State Manager Overview (`StateManager.cpp` & `StateManager.h`)

The **State Manager** is the central nervous system of the firmware. It is responsible for:
-   Storing the `currentState` and `previousState`.
-   Executing the specific logic for the current state in the main loop.
-   Handling state transitions, including calling `onEnter` and `onExit` lifecycle functions.
-   Managing global flags and hardware accessors.
-   Handling common operations that run regardless of the state (e.g., updating switch states, monitoring sensors).

### Core Components
-   **`executeStateMachine()`**: Called repeatedly from the main loop. It routes execution to the specific `execute...` function corresponding to the `currentState`. It also handles global error LED blinking.
-   **`changeState(SystemState newState)`**: The gatekeeper for transitions. It:
    1.  Calls `onExit...` for the old state.
    2.  Updates `previousState` and `currentState`.
    3.  Notifies the dashboard/logging system.
    4.  Calls `onEnter...` for the new state.
-   **`handleCommonOperations()`**: Updates all debounced switches (`Bounce2` objects), monitors the Transfer Arm (TA) signal timing, handles Rotation Clamp retraction timers, and performs safety checks (e.g., stopping the cut motor if it hits the home sensor unexpectedly during return).

---

## 2. Detailed State Breakdowns

### 2.1. STARTUP State (`00_STARTUP.cpp`)
**Purpose**: The initial state upon power-up or reset. It establishes initial feedback and network confirmation.

*   **Entry Actions**: None.
*   **Main Logic**:
    1.  **Visual Feedback**: Turns on the **Blue LED** to indicate system startup/homing mode.
    2.  **Network Info**: Prints the WiFi IP address to the Serial Monitor.
    3.  **Delay**: Waits for 1 second to ensure the message is readable and hardware settles.
    4.  **Transition**: Automatically changes state to **HOMING**.
*   **Exit Actions**: None.

### 2.2. HOMING State (`01_HOMING.cpp`)
**Purpose**: Calibrates the stepper motors to their zero positions. This is critical for accurate movement.

*   **Entry Actions**: Resets internal tracking flags (`cutMotorHomed`, `feedMotorHomed`, `feedHomingPhaseInitiated`).
*   **Main Logic**:
    1.  **Indicator**: Blinks the **Blue LED** continuously (500ms interval) to indicate active homing.
    2.  **Cut Motor Homing (Blocking)**:
        -   Calls `homeCutMotorBlocking()`.
        -   Moves the Cut Motor towards the home sensor until triggered.
        -   Sets position to 0.
        -   *Note*: If this fails, it may retry or stall (dependent on `General_Functions` implementation).
    3.  **Feed Motor Homing (Non-Blocking)**:
        -   **Phase 1**: Retracts the Feed Clamp to allow movement.
        -   **Phase 2**: Calls `homeFeedMotorNonBlocking()`.
        -   **Phase 3**: Once homed, it extends the Feed Clamp again to secure the material.
    4.  **Completion**:
        -   Sets global `isHomed` flag to true.
        -   Turns Blue LED **OFF**, Green LED **ON** (System Ready).
        -   *Safety Note*: Does **not** home the rotation servo here to prevent collision with potential stuck wood.
    5.  **Transition**: Automatically changes state to **IDLE**.
*   **Exit Actions**: None.

### 2.3. IDLE State (`02_IDLE.cpp`)
**Purpose**: The "resting" state where the machine waits for user input. It monitors switches and sensors to determine the next action.

*   **Entry Actions**:
    -   **Clamp Logic**:
        -   If entering from `RELOAD` mode: Retracts 2x4 Secure Clamp.
        -   If entering from a "No Wood" condition: Keeps 2x4 Secure Clamp extended (to prevent dropping small pieces).
        -   Otherwise (Normal): **Extends** 2x4 Secure Clamp.
    -   **Retracts** Feed Clamp and Rotation Clamp.
    -   Resets "Coming from No Wood" flag if it was set.
*   **Main Logic**:
    1.  **Reload Check**: Checks `Reload Switch`. If HIGH, transitions to **RELOAD** state.
    2.  **Feed First Cut Check**:
        -   Checks `Pushwood Forward Switch` (Manual Feed).
        -   If pressed AND `FIRST_CUT_OR_WOOD_FWD_ONE` sensor is **HIGH**: Transitions to **FEED_FIRST_CUT**.
        -   If pressed AND `FIRST_CUT_OR_WOOD_FWD_ONE` sensor is **LOW**: Transitions to **FEED_WOOD_FWD_ONE**.
    3.  **Start Cycle Check**:
        -   Checks `Start Cycle Switch` (Rising Edge) OR (`Continuous Mode` AND not currently cycling).
        -   Verifies safety conditions: `Wood Suction Error` is false, `Start Switch Safe` is true.
        -   **If conditions met**:
            -   Updates LEDs: Green OFF, Yellow ON.
            -   Sets `cuttingCycleInProgress = true`.
            -   Extends all clamps (Feed and Secure).
            -   Configures Cut Motor for cutting speed.
            -   Transitions to **CUTTING**.
*   **Exit Actions**: None.

### 2.4. CUTTING State (`03_CUTTING.cpp`)
**Purpose**: Performs the actual cutting operation. It orchestrates the Cut Motor, Rotation Servo, Rotation Clamp, and Transfer Arm signal based on position.

*   **Entry Actions**: Resets internal step counter, starts cycle timer, stops reload timer.
*   **Main Logic** (Divided into Steps):
    -   **Step 0 (Preparation)**:
        -   Checks for OTA updates.
        -   **Servo Return**: Waits for Rotation Servo to return home (if it was active).
        -   **Clamping**: Ensures 2x4 Secure Clamp and Feed Clamp are extended.
        -   **Suction Check**:
            -   Monitors Suction Sensor. If wood is not grabbed by Transfer Arm (Sensor LOW) after timeout, triggers **SUCTION_ERROR**.
            -   If Sensor HIGH (Grabbed), allows proceeding.
        -   **Motor Config**: Sets speed based on wood presence (Fast if wood present, Slow if not).
        -   **Action**: Commands Cut Motor to move to "Cut" position. Advances to Step 1.
    -   **Step 1 (Monitoring)**:
        -   Updates LED status based on Wood Present Sensor (Yellow if wood, Wave pattern if no wood).
        -   Advances to Step 2.
    -   **Step 2 (Execution)**:
        -   **Position Triggers**: Monitors Cut Motor position.
            -   At `ROTATION_CLAMP_ACTIVATION_DISTANCE`: Extends Rotation Clamp.
            -   At `ROTATION_SERVO_ACTIVATION_DISTANCE`: Activates Rotation Servo (rotates wood).
            -   At `TA_SIGNAL_ACTIVATION_DISTANCE`: Sends signal to Transfer Arm.
        -   **Completion Check**: When Cut Motor stops (reaches target):
            -   Starts Cut Motor return configuration.
            -   Checks Wood Present Sensor.
            -   **Transition**:
                -   If Wood Present: **RETURNING_YES_2x4**.
                -   If No Wood: **RETURNING_NO_2x4**.
*   **Exit Actions**: Resets internal step counter.

### 2.5. RETURNING_YES_2x4 State (`04_RETURNING_Yes_2x4.cpp`)
**Purpose**: Optimized return sequence when wood is present. It performs simultaneous operations: returning the Cut Motor while advancing the Feed Motor for the next cut.

*   **Entry Actions**:
    -   Increments "Yes Wood" counter.
    -   Sets flag `cutMotorInReturningYes2x4Return` to true (enables safety check in `StateManager`).
    -   Initializes sub-step counters.
*   **Main Logic** (Complex State Machine within a State):
    -   **Feed Motor Sequence**:
        1.  Retracts Feed Clamp.
        2.  Moves Feed Motor to Home (Zero).
        3.  Extends Feed Clamp, Retracts Secure Clamp.
        4.  **Wait**: 200ms delay.
        5.  **Safety**: Verify Cut Motor is Home.
        6.  Moves Feed Motor to `FEED_TRAVEL_DISTANCE` (advancing wood).
    -   **Cut Motor Sequence**:
        1.  Waits for Cut Motor to stop moving (it was commanded to return at end of CUTTING).
        2.  **Homing Verification**: Checks Homing Sensor 3 times.
        3.  **Recovery**: If not home, attempts incremental moves until found (or Errors out).
        4.  Sets Position to 0.
    -   **Synchronization**:
        -   Waits for *both* sequences to complete.
        -   Extends Secure Clamp.
    -   **Completion**:
        -   Increments Cycle Counter.
        -   **Transition**:
            -   If `Continuous Mode` active: **CUTTING**.
            -   Else: **IDLE**.
*   **Exit Actions**: Resets internal step counters.

### 2.6. RETURNING_NO_2x4 State (`05_RETURNING_No_2x4.cpp`)
**Purpose**: Special return sequence when the sensor didn't detect wood (e.g., end of board). It uses a "pecking" or multi-stage feed motion to try and clear the remnant or prepare for a fresh board.

*   **Entry Actions**: Resets "Yes Wood" counter, configures motors for return, resets LED patterns.
*   **Main Logic** (8-Step Sequence):
    1.  **Initialize**: Retracts 2x4 Secure Clamp.
    2.  **Wait**: Waits for Cut Motor to return home.
    3.  **Move 1**: Feed Motor moves negative (-1.2") with Clamp Extended.
    4.  **Move 2**: Feed Motor moves positive (to Home) with Clamp Retracted.
    5.  **Move 3**: Feed Motor moves negative (to Final Position) with Clamp Extended.
    6.  **Final Check**:
        -   Retracts Feed Clamp.
        -   Checks 2x4 Present Sensor.
        -   **If Sensor Clear (HIGH)**:
            -   Extends Secure Clamp (after delay).
            -   Sets `ComingFromNoWoodWithSensorsClear` flag.
            -   Disables `StartSwitchSafe` (forces manual reset).
            -   **Transition**: **IDLE**.
        -   **If Sensor Blocked**: Waits until clear.
*   **Exit Actions**: Resets step counters.

### 2.7. RELOAD State (`06_RELOAD.cpp`)
**Purpose**: Safety state for manually loading new lumber.

*   **Entry Actions**:
    -   Retracts Feed Clamp.
    -   Retracts 2x4 Secure Clamp.
    -   Turns **Blue LED** ON.
    -   Resets No-Wood flags.
*   **Main Logic**:
    -   Continuously checks `Reload Switch`.
    -   If Switch goes LOW (OFF):
        -   **Transition**: **IDLE**.
*   **Exit Actions**: Sets `isReloadMode` to false, turns Blue LED OFF.

### 2.8. FEED_WOOD_FWD_ONE State (`07_FEED_WOOD_FWD_ONE.cpp`)
**Purpose**: Manually advances the wood by one "cut length" (Travel Distance). Triggered from IDLE.

*   **Entry Actions**: Initializes steps.
*   **Main Logic**:
    1.  Retracts Feed Clamp.
    2.  Moves Feed Motor to Zero (Home).
    3.  Extends Feed Clamp, Retracts Secure Clamp.
    4.  Waits 200ms.
    5.  Moves Feed Motor to `FEED_TRAVEL_DISTANCE`.
    6.  **Transition**:
        -   If `Start Cycle Switch` HIGH: **CUTTING**.
        -   Else: **IDLE**.
*   **Exit Actions**: Resets steps.

### 2.9. FEED_FIRST_CUT State (`08_FEED_FIRST_CUT.cpp`)
**Purpose**: A specific double-feed sequence used to align a fresh 2x4 board for the very first cut (squaring the end).

*   **Entry Actions**: Initializes steps.
*   **Main Logic** (Two-Stage Feed):
    -   **Run 1**:
        -   Retract Feed Clamp.
        -   Move to Start (-1.2").
        -   Extend Feed Clamp, Retract Secure Clamp.
        -   Wait 200ms.
        -   Move to End (3.4").
    -   **Run 2**:
        -   Retract Feed Clamp.
        -   Move to Start (-1.2").
        -   Extend Feed Clamp, Retract Secure Clamp.
        -   Wait 200ms.
        -   Move to End (2.0").
    -   **Completion**:
        -   Sets `StartSwitchSafe` to true.
        -   **Transition**: Checks Start Switch -> **CUTTING** or **IDLE**.
*   **Exit Actions**: Resets steps.

---

## 3. Error States (Detailed Analysis)

These states handle system faults and require user intervention.

-   **ERROR**: Generic error state.
    -   **Indicator**: Blinks Red and Yellow LEDs (alternating).
    -   **Recovery**: Waits for `Reload Switch` to be toggled (Rising Edge).
    -   **Logic**: Stops all motors immediately.
-   **ERROR_RESET**: Transition state.
    -   **Action**: Calls `handleErrorResetState()` which typically clears flags, homes motors if necessary, and transitions to IDLE or HOMING depending on severity.
-   **SUCTION_ERROR** (`Suction_Error.cpp`):
    -   **Cause**: Transfer Arm Suction Sensor (Pin 39) failed to detect wood during CUTTING.
    -   **Behavior**:
        1.  Wait for sensor to clear (go HIGH) then wait 3 seconds.
        2.  Automatically Home Cut Motor (10s timeout).
        3.  Blink Red LED.
    -   **Recovery**: User must toggle `Start Cycle Switch`. System transitions to **HOMING**.
-   **Cut_Motor_Homing_Error**:
    -   **Cause**: Cut Motor failed to trigger home sensor during homing sequence or return.
    -   **Behavior**: Stops all motors. Blinks error code.
    -   **Recovery**: Acknowledge via `Reload Switch` -> **ERROR_RESET**.

---

## 4. Hardware Interface & Configuration

### 4.1. Pin Definitions (`Pin_Def.cpp`)
The system interacts with the ESP32-S3 via specific GPIO assignments:

| Component | Pin | Logic Level / Type | Note |
| :--- | :--- | :--- | :--- |
| **Cut Motor** | Step: 12, Dir: 11 | Stepper Driver | |
| **Feed Motor** | Step: 17, Dir: 18 | Stepper Driver | |
| **Rotation Servo** | 14 | PWM | |
| **Cut Home Switch** | 3 | Active HIGH | Input Pulldown |
| **Feed Home Sensor** | 45 | Active LOW | Input Pullup |
| **Reload Switch** | 6 | Active HIGH | Input Pulldown |
| **Start Switch** | 5 | Active HIGH | Input Pulldown |
| **Manual Feed Sw** | 16 | Active HIGH | Input Pulldown |
| **First Cut/Fwd** | 10 | Sensor | LOW = Fwd One, HIGH = First Cut |
| **2x4 Present** | 4 | Active LOW | Input Pullup |
| **Suction Sensor** | 39 | Active HIGH | HIGH = Grabbed (Confusing naming in code, but Logic is HIGH=Grabbed) |
| **Feed Clamp** | 36 | Active LOW | LOW = Extended (Inversed) |
| **Secure Clamp** | 48 | Active LOW | LOW = Extended (Inversed) |
| **Rotation Clamp** | 42 | Active HIGH | HIGH = Extended |
| **TA Signal** | 8 | Digital Out | To Transfer Arm |

### 4.2. Motor Configuration (`Motor_Config.cpp`)

**Cut Motor (NEMA 23)**
-   **Steps Per Inch**: 500.0
-   **Normal Speed**: 1.28 in/sec
-   **No Wood Speed**: 1.1 in/sec
-   **Return Speed**: 30.0 in/sec (Fast return)
-   **Homing Speed**: 2.6 in/sec

**Feed Motor**
-   **Steps Per Inch**: 1000.0
-   **Normal Speed**: 22,000 steps/sec
-   **Return Speed**: 40,000 steps/sec (High speed return)
-   **Homing Speed**: 1,500 steps/sec

### 4.3. Special Functional Modes

**Minis Mode vs. 3 Inch Mode**
-   **Config Mode 1 (Minis)**:
    -   Adds **50ms** delay to Rotation Clamp retraction.
    -   Adds **500ms** non-blocking delay before sending Transfer Arm (TA) signal.
-   **Default Mode (3 Inch)**:
    -   Standard timing.
    -   Immediate TA signal transmission.

**LED Wave Pattern**
-   Used during "No Wood" conditions in Cutting/Returning states.
-   Sequentially lights up Red -> Yellow -> Green -> Blue LEDs.
-   Interval: 200ms per step.
-   Duration: 250ms per LED.

---

## 5. State Connection & Flow Diagram

```text
       [POWER ON]
           │
           v
      [00_STARTUP]
           │
           v
      [01_HOMING]
           │
           v
      [02_IDLE] <───────────────────────────────────────────────┐
           │   ^                                                │
           │   │ (Reload Switch OFF)                            │
           │   └────────────── [06_RELOAD] <────────────────┐   │
           │                                                │   │
           ├── (Reload Switch ON) ──────────────────────────┘   │
           │                                                    │
           ├── (Manual Feed + Sensor HIGH) ──> [08_FEED_FIRST_CUT]
           │                                         │
           │                                         │ (Start Switch?)
           │                                         ├─ YES ─┐
           │                                         └─ NO ──│──┐
           │                                                 │  │
           ├── (Manual Feed + Sensor LOW) ───> [07_FEED_FWD_ONE]│
           │                                         │          │
           │                                         │ (Start Switch?)
           │                                         ├─ YES ─┐  │
           │                                         └─ NO ──│──┘
           │                                                 │
           ├── (Start Switch / Continuous) ──────────────────┘
           │
           v
      [03_CUTTING]
           │
           │ (Checks Wood Sensor at end of cut)
           │
           ├── [YES WOOD] ──> [04_RETURNING_YES_2x4]
           │                        │
           │                        │ (Continuous Mode?)
           │                        ├─ YES ──────────────┐
           │                        └─ NO ───────────────│──┐
           │                                             │  │
           └── [NO WOOD] ───> [05_RETURNING_NO_2x4]      │  │
                                    │                    │  │
                                    └────────────────────┘  │
                                                            │
           (All Paths return here if not Continuous)        │
                                                            │
           v <──────────────────────────────────────────────┘
      [02_IDLE]
```
