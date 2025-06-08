#include "StateMachine/03_CUTTING.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"

//* ************************************************************************
//* ************************** CUTTING STATE *******************************
//* ************************************************************************
// Handles the wood cutting operation.
// This state manages a multi-step cutting process.
// It includes logic for normal cutting, deciding between a YES_WOOD Sequence and a NO_WOOD Sequence, and error handling.

void CuttingState::onEnter(StateManager& stateManager) {
    // Reset all step counters when entering cutting state
    resetSteps();
}

void CuttingState::onExit(StateManager& stateManager) {
    // Reset all step counters when exiting cutting state
    resetSteps();
}

void CuttingState::execute(StateManager& stateManager) {
    // Throttle state logging to every 2 seconds
    static unsigned long lastStateLogTime = 0;
    if (millis() - lastStateLogTime >= 2000) {
        //serial.println("Current State: CUTTING");
        lastStateLogTime = millis();
    }
    
    if (homePositionErrorDetected) {
        handleHomePositionError(stateManager);
        return;
    }
    
    // Handle signal timing independently of motor movements
    if (signalActive && millis() - signalStartTime >= 2000) { // Original was 2000ms
        signalActive = false;
    }

    switch (cuttingStep) {
        case 0: 
            handleCuttingStep0(stateManager);
            break;
        case 1: 
            handleCuttingStep1(stateManager);
            break;
        case 2: 
            handleCuttingStep2(stateManager);
            break;
        case 3: 
            handleCuttingStep3(stateManager);
            break;
        case 4: 
            handleCuttingStep4(stateManager);
            break;
        case 5: 
            handleCuttingStep5(stateManager);
            break;
        case 8:
            handleCuttingStep8_FeedMotorHomingSequence(stateManager);
            break;
        case 9:
            handleCuttingStep9_SuctionErrorRecovery(stateManager);
            break;
    }
}

void CuttingState::handleCuttingStep0(StateManager& stateManager) {
    //serial.println("Cutting Step 0: Starting cut motion."); 
    
    // Debug motor before configuration
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    if (cutMotor) {
        Serial.print("Step 0 - Motor before config - Running: ");
        Serial.print(cutMotor->isRunning() ? "YES" : "NO");
        Serial.print(", Position: ");
        //serial.println(cutMotor->getCurrentPosition());
    } else {
        //serial.println("Step 0 - ERROR: cutMotor is NULL!");
    }
        
    extend2x4SecureClamp();
    extendFeedClamp();

    // Configure and move motor
    configureCutMotorForCutting();
    moveCutMotorToCut();
    
    // Debug motor after move command
    if (cutMotor) {
        Serial.print("Step 0 - Motor after moveTo command - Running: ");
        Serial.print(cutMotor->isRunning() ? "YES" : "NO");
        Serial.print(", Position: ");
        Serial.print(cutMotor->getCurrentPosition());
        Serial.print(", Target: ");
        Serial.print(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
        //serial.println(" steps");
    }
    
    rotationClampActivatedThisCycle = false; // Reset for this cut cycle
    cuttingStep = 1;
    //serial.println("Step 0 complete - advancing to Step 1");
}

void CuttingState::handleCuttingStep1(StateManager& stateManager) {
    // CUTTING (Step 1): Check WAS_WOOD_SUCTIONED_SENSOR, start cut motor, monitor position for servo activation
    extern const int WOOD_SUCTION_CONFIRM_SENSOR; // From main.cpp
    
    if (stepStartTime == 0) {
        stepStartTime = millis();
        //serial.println("Cutting Step 1: Checking suction sensor then starting cut motion.");
        
        // Debug motor status at start of step 1
        FastAccelStepper* cutMotor = stateManager.getCutMotor();
        if (cutMotor) {
            Serial.print("Step 1 start - Motor Running: ");
            Serial.print(cutMotor->isRunning() ? "YES" : "NO");
            Serial.print(", Position: ");
            //serial.println(cutMotor->getCurrentPosition());
        }
    }

    // Check suction sensor after brief delay to ensure it's stabilized
    if (millis() - stepStartTime >= 1000) {
        if (digitalRead(WOOD_SUCTION_CONFIRM_SENSOR) == LOW) { // LOW means NO SUCTION (Error condition)
            //serial.println("Cutting Step 1: WAS_WOOD_SUCTIONED_SENSOR is LOW (No Suction). Error detected. Returning cut motor home before manual reset.");
            
            // Stop feed motor immediately but return cut motor home safely
            FastAccelStepper* cutMotor = stateManager.getCutMotor();
            FastAccelStepper* feedMotor = stateManager.getFeedMotor();
            
            if (feedMotor && feedMotor->isRunning()) {
                feedMotor->forceStop();
                //serial.println("Feed motor stopped due to suction error.");
            }
            
            // Configure cut motor for safe return home
            if (cutMotor) {
                configureCutMotorForReturn();
                moveCutMotorToHome();
                //serial.println("Cut motor returning home due to suction error.");
            }
            
            // Set cutting cycle flag to false to prevent continuous operation
            stateManager.setCuttingCycleInProgress(false);
            
            // Transition to suction error recovery step
            cuttingStep = 9; // New step for suction error recovery
            stepStartTime = 0; // Reset step timer
            return;
        } else {
            //serial.println("Cutting Step 1: WAS_WOOD_SUCTIONED_SENSOR is HIGH (Suction OK). Starting cut motor toward cut position.");
            
            // Debug: Double-check motor configuration and movement
            FastAccelStepper* cutMotor = stateManager.getCutMotor();
            if (cutMotor) {
                Serial.print("Step 1 - Before reconfiguring - Motor Running: ");
                Serial.print(cutMotor->isRunning() ? "YES" : "NO");
                Serial.print(", Position: ");
                //serial.println(cutMotor->getCurrentPosition());
                
                configureCutMotorForCutting();
                moveCutMotorToCut();
                
                // Small delay to let motor start
                delay(50);
                
                Serial.print("Step 1 - After moveCutMotorToCut - Motor Running: ");
                Serial.print(cutMotor->isRunning() ? "YES" : "NO");
                Serial.print(", Position: ");
                Serial.print(cutMotor->getCurrentPosition());
                Serial.print(", Target: ");
                Serial.print(CUT_TRAVEL_DISTANCE * CUT_MOTOR_STEPS_PER_INCH);
                //serial.println(" steps");
            } else {
                //serial.println("Step 1 - ERROR: cutMotor is NULL!");
            }
            
            cuttingStep = 2;
            stepStartTime = 0; // Reset for next step
            //serial.println("Step 1 complete - advancing to Step 2");
        }
    }
}

void CuttingState::handleCuttingStep2(StateManager& stateManager) {
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const int _2x4_PRESENT_SENSOR; // From main.cpp
    extern const float ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES; // From main.cpp
    extern const float ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES; // From main.cpp
    // CUT_TRAVEL_DISTANCE and CUT_MOTOR_STEPS_PER_INCH are already declared in General_Functions.h
    
    // Debug logging every 100ms to track motor state
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 100) {
        if (cutMotor) {
            long currentPosition = cutMotor->getCurrentPosition();
            float currentPositionInches = (float)currentPosition / CUT_MOTOR_STEPS_PER_INCH;
            Serial.print("Step 2 - Motor Running: ");
            Serial.print(cutMotor->isRunning() ? "YES" : "NO");
            Serial.print(", Position: ");
            Serial.print(currentPositionInches, 3);
            Serial.print(" inches (");
            Serial.print(currentPosition);
            Serial.print(" steps), Target: ");
            Serial.print(CUT_TRAVEL_DISTANCE);
            //serial.println(" inches");
        } else {
            //serial.println("Step 2 - ERROR: cutMotor is NULL!");
        }
        lastDebugTime = millis();
    }
    
    // Debug logging for motor position every 500ms (detailed logging)
    static unsigned long lastDetailedDebugTime = 0;
    if (cutMotor && cutMotor->isRunning() && millis() - lastDetailedDebugTime >= 500) {
        long currentPosition = cutMotor->getCurrentPosition();
        float currentPositionInches = (float)currentPosition / CUT_MOTOR_STEPS_PER_INCH;
        Serial.print("Cut motor position: ");
        Serial.print(currentPositionInches);
        Serial.print(" inches (");
        Serial.print(currentPosition);
        Serial.print(" steps). Clamp activated: ");
        //serial.println(rotationClampActivatedThisCycle ? "YES" : "NO");
        lastDetailedDebugTime = millis();
    }
    
    // Early Rotation Clamp Activation (matching old catcher clamp logic)
    if (!rotationClampActivatedThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= ((CUT_TRAVEL_DISTANCE - ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH)) {
        Serial.print("*** ACTIVATING ROTATION CLAMP *** Position: ");
        Serial.print((float)cutMotor->getCurrentPosition() / CUT_MOTOR_STEPS_PER_INCH);
        Serial.print(" inches, Activation threshold: ");
        Serial.print(CUT_TRAVEL_DISTANCE - ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES);
        //serial.println(" inches");
        extendRotationClamp(); // FIXED: Extend clamp when reaching activation position (matching old catcher clamp logic)
        rotationClampActivatedThisCycle = true;
        Serial.print("Cutting Step 2: Cut motor reached ");
        Serial.print(CUT_TRAVEL_DISTANCE - ROTATION_CLAMP_EARLY_ACTIVATION_OFFSET_INCHES);
        //serial.println(" inches - extending rotation clamp early.");
    }
    
    // Early Rotation Servo Activation (matching old catcher servo logic)
    if (!rotationServoActivatedThisCycle && cutMotor &&
        cutMotor->getCurrentPosition() >= ((CUT_TRAVEL_DISTANCE - ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES) * CUT_MOTOR_STEPS_PER_INCH)) {
        //serial.println("Cutting Step 2: Activating Rotation Servo (early).");
        activateRotationServo();
        rotationServoActivatedThisCycle = true;
        Serial.print("Cutting Step 2: Cut motor reached ");
        Serial.print(CUT_TRAVEL_DISTANCE - ROTATION_SERVO_EARLY_ACTIVATION_OFFSET_INCHES);
        //serial.println(" inches - activating rotation servo early.");
    }
    
    // Check if motor finished moving to cut position
    if (cutMotor && !cutMotor->isRunning()) {
        //serial.println("Cutting Step 2: Cut fully complete."); 
        sendSignalToTA(); // Signal to Transfer Arm (this also activates servo if not already active)
        configureCutMotorForReturn();

        int sensorValue = digitalRead(_2x4_PRESENT_SENSOR);
        bool no2x4Detected = (sensorValue == HIGH);
        
        if (no2x4Detected) {
            //serial.println("Cutting Step 2: RETURNING_NO_2x4 state - 2x4 sensor reads HIGH. Transitioning to RETURNING_NO_2x4 state.");
            stateManager.changeState(RETURNING_NO_2x4);
        } else {
            //serial.println("Cutting Step 2: RETURNING_YES_2x4 state - 2x4 sensor reads LOW. Transitioning to RETURNING_YES_2x4 state.");
            stateManager.changeState(RETURNING_YES_2x4);
        }
    }
}

void CuttingState::handleCuttingStep3(StateManager& stateManager) {
    //serial.println("Cutting Step 3: (Should be bypassed for wood path) Initial position move complete.");
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    if (feedMotor && !feedMotor->isRunning()) {
        retract2x4SecureClamp();
        //serial.println("Feed clamp and 2x4 secure clamp retracted.");

        configureFeedMotorForReturn();
        moveFeedMotorToHome();
        //serial.println("Feed motor moving to home (0).");
    }
}

void CuttingState::handleCuttingStep4(StateManager& stateManager) {
    //serial.println("Cutting Step 4: (Logic moved to Step 7 for wood path) Feed motor at home (0).");
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    extern const float CUT_MOTOR_INCREMENTAL_MOVE_INCHES; // From main.cpp
    extern const float CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES; // From main.cpp
    // CUT_MOTOR_STEPS_PER_INCH, FEED_TRAVEL_DISTANCE are already declared in General_Functions.h
    
    if (feedMotor && !feedMotor->isRunning()) {
        retract2x4SecureClamp();
        //serial.println("Feed clamp retracted.");

        if (cutMotor && !cutMotor->isRunning()) {
            //serial.println("Cut motor also at home. Checking cut motor position switch.");
            bool sensorDetectedHome = false;
            for (int i = 0; i < 3; i++) {
                delay(30);
                stateManager.getCutHomingSwitch()->update();
                Serial.print("Cut position switch read attempt "); Serial.print(i+1); Serial.print(": "); //serial.println(stateManager.getCutHomingSwitch()->read());
                if (stateManager.getCutHomingSwitch()->read() == HIGH) {
                    sensorDetectedHome = true;
                    if (cutMotor) cutMotor->setCurrentPosition(0); // Recalibrate to 0 when switch is hit
                    //serial.println("Cut motor position switch detected HIGH. Position recalibrated to 0.");
                    break;
                }
            }
            if (!sensorDetectedHome) {
                //serial.println("ERROR: Cut motor position switch did not detect home after return attempt.");
                if (cutMotorIncrementalMoveTotalInches < CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES) {
                    Serial.print("Attempting incremental move. Total moved: ");
                    Serial.print(cutMotorIncrementalMoveTotalInches);
                    //serial.println(" inches.");
                    cutMotor->move(-CUT_MOTOR_INCREMENTAL_MOVE_INCHES * CUT_MOTOR_STEPS_PER_INCH);
                    cutMotorIncrementalMoveTotalInches += CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
                    // Stay in cuttingStep 4 to re-check sensor after move
                } else {
                    //serial.println("ERROR: Cut motor position switch did not detect home after MAX incremental moves!");
                    stopCutMotor();
                    stopFeedMotor();
                    extend2x4SecureClamp();
                    turnRedLedOn();
                    turnYellowLedOff();
                    stateManager.changeState(ERROR);
                    stateManager.setErrorStartTime(millis());
                    resetSteps();
                    cutMotorIncrementalMoveTotalInches = 0.0; // Reset for next attempt
                    //serial.println("Transitioning to ERROR state due to cut motor homing failure after cut.");
                }
            } else {
                //serial.println("Cut motor position switch confirmed home. Moving feed motor to final position.");
                cutMotorIncrementalMoveTotalInches = 0.0; // Reset on success
                moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                cuttingStep = 5; 
            }
        }
    }
}

void CuttingState::handleCuttingStep5(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    if (feedMotor && !feedMotor->isRunning()) {
        //serial.println("Cutting Step 5: Feed motor at final position. Starting end-of-cycle feed motor homing sequence."); 
        
        //! ************************************************************************
        //! STEP 1: RETRACT FEED CLAMP AND START FEED MOTOR HOMING SEQUENCE
        //! ************************************************************************
        retract2x4SecureClamp();
        //serial.println("Feed clamp retracted. Starting feed motor homing sequence...");
        
        // Transition to new step 8 for feed motor homing sequence
        cuttingStep = 8;
        cuttingSubStep8 = 0; // Initialize homing substep
        //serial.println("Transitioning to feed motor homing sequence (Step 8)."); 
    }
}

void CuttingState::handleCuttingStep8_FeedMotorHomingSequence(StateManager& stateManager) {
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    extern const float FEED_MOTOR_HOMING_SPEED; // From main.cpp
    // FEED_TRAVEL_DISTANCE and FEED_MOTOR_STEPS_PER_INCH are already declared in General_Functions.h
    
    // Non-blocking feed motor homing sequence
    switch (cuttingSubStep8) {
        case 0: // Start homing - move toward home switch
            //serial.println("Feed Motor Homing Step 8.0: Moving toward home switch.");
            if (feedMotor) {
                feedMotor->setSpeedInHz((uint32_t)FEED_MOTOR_HOMING_SPEED);
                feedMotor->moveTo(10000 * FEED_MOTOR_STEPS_PER_INCH); // Large positive move toward switch
            }
            cuttingSubStep8 = 1;
            break;
            
        case 1: // Wait for home switch to trigger
            stateManager.getFeedHomingSwitch()->update();
            if (stateManager.getFeedHomingSwitch()->read() == HIGH) {
                //serial.println("Feed Motor Homing Step 8.1: Home switch triggered. Stopping motor.");
                if (feedMotor) {
                    feedMotor->stopMove();
                    feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH);
                }
                //serial.println("Feed motor hit home switch.");
                cuttingSubStep8 = 2;
            }
            break;
            
        case 2: // Wait for motor to stop, then move to -0.2 inch from switch
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("Feed Motor Homing Step 8.2: Moving to -0.2 inch from home switch to establish working zero.");
                feedMotor->moveTo(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH - 0.1 * FEED_MOTOR_STEPS_PER_INCH);
                cuttingSubStep8 = 3;
            }
            break;
            
        case 3: // Wait for positioning move to complete, then set new zero
            if (feedMotor && !feedMotor->isRunning()) {
                //serial.println("Feed Motor Homing Step 8.3: Setting new working zero position.");
                feedMotor->setCurrentPosition(FEED_TRAVEL_DISTANCE * FEED_MOTOR_STEPS_PER_INCH); // Set this position as the new zero
                //serial.println("Feed motor homed: 0.2 inch from switch set as position 0.");
                
                configureFeedMotorForNormalOperation();
                cuttingSubStep8 = 4;
            }
            break;
            
        case 4: // Homing complete - check for continuous mode or finish cycle
            //serial.println("Feed Motor Homing Step 8.4: Homing sequence complete.");
            extend2x4SecureClamp();
            //serial.println("2x4 secure clamp extended."); 
            turnYellowLedOff();
            stateManager.setCuttingCycleInProgress(false);
            
            // Check if start cycle switch is active for continuous operation
            if (stateManager.getStartCycleSwitch()->read() == HIGH && stateManager.getStartSwitchSafe()) {
                //serial.println("Start cycle switch is active - continuing with another cut cycle.");
                // Prepare for next cycle
                extend2x4SecureClamp();
                extendRotationClamp(); // Extend rotation clamp for next cutting cycle
                configureCutMotorForCutting(); // Ensure cut motor is set to proper cutting speed
                turnYellowLedOn();
                stateManager.setCuttingCycleInProgress(true);
                stateManager.changeState(CUTTING);
                resetSteps();
                //serial.println("Transitioning to CUTTING state for continuous operation.");
            } else {
                //serial.println("Cycle complete. Transitioning to IDLE state.");
                stateManager.changeState(IDLE);
                resetSteps();
            }
            break;
    }
}

void CuttingState::handleCuttingStep9_SuctionErrorRecovery(StateManager& stateManager) {
    // CUTTING (Step 9): Suction Error Recovery - Wait for cut motor to return home, then transition to error hold
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    
    if (cutMotor && !cutMotor->isRunning()) {
        //serial.println("Cutting Step 9: Cut motor returned home after suction error. Checking home sensor.");
        
        // Check cut motor home sensor
        bool sensorDetectedHome = false;
        for (int i = 0; i < 3; i++) {
            delay(30);
            stateManager.getCutHomingSwitch()->update();
            Serial.print("Cut position switch read attempt "); 
            Serial.print(i+1); 
            Serial.print(": "); 
            //serial.println(stateManager.getCutHomingSwitch()->read());
            
            if (stateManager.getCutHomingSwitch()->read() == HIGH) {
                sensorDetectedHome = true;
                if (cutMotor) cutMotor->setCurrentPosition(0); // Recalibrate to 0 when switch is hit
                //serial.println("Cut motor position switch detected HIGH after suction error recovery.");
                break;
            }
        }
        
        if (sensorDetectedHome) {
            //serial.println("Cut motor successfully returned home after suction error. Transitioning to SUCTION_ERROR for manual reset.");
            stateManager.changeState(SUCTION_ERROR);
            resetSteps();
        } else {
            //serial.println("WARNING: Cut motor home sensor not detected after suction error recovery. Proceeding to SUCTION_ERROR anyway.");
            stateManager.changeState(SUCTION_ERROR);
            resetSteps();
        }
    } else if (cutMotor) {
        // Motor still running - provide status update
        static unsigned long lastStatusTime = 0;
        if (millis() - lastStatusTime >= 1000) {
            //serial.println("Cutting Step 9: Cut motor returning home after suction error...");
            lastStatusTime = millis();
        }
    }
}

void CuttingState::handleHomePositionError(StateManager& stateManager) {
    //serial.println("Home position error detected during cutting operation."); 
    unsigned long lastErrorBlinkTime = stateManager.getLastErrorBlinkTime();
    bool errorBlinkState = stateManager.getErrorBlinkState();
    
    if (millis() - lastErrorBlinkTime > 100) { 
        errorBlinkState = !errorBlinkState;
        stateManager.setErrorBlinkState(errorBlinkState);
        if(errorBlinkState) turnRedLedOn(); else turnRedLedOff();
        if(!errorBlinkState) turnYellowLedOn(); else turnYellowLedOff();
        stateManager.setLastErrorBlinkTime(millis());
    }
    
    FastAccelStepper* cutMotor = stateManager.getCutMotor();
    FastAccelStepper* feedMotor = stateManager.getFeedMotor();
    if (cutMotor) cutMotor->forceStopAndNewPosition(cutMotor->getCurrentPosition());
    if (feedMotor) feedMotor->forceStopAndNewPosition(feedMotor->getCurrentPosition());
    
    extend2x4SecureClamp();
    
    if (stateManager.getReloadSwitch()->rose()) {
        homePositionErrorDetected = false;
        stateManager.changeState(ERROR_RESET);
        stateManager.setErrorAcknowledged(true);
        //serial.println("Home position error acknowledged by reload switch."); 
    }
}

void CuttingState::resetSteps() {
    cuttingStep = 0;
    signalStartTime = 0;
    signalActive = false;
    homePositionErrorDetected = false;
    rotationClampActivatedThisCycle = false;
    rotationServoActivatedThisCycle = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
    cuttingSubStep8 = 0; // Reset position motor homing substep
} 