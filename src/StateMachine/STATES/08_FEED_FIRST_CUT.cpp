#include "StateMachine/08_FEED_FIRST_CUT.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include <math.h>

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================
namespace FeedFirstCutConfig {
    constexpr float TOTAL_WOOD_MOVEMENT_INCHES = 5.0;    // Total wood movement goal
    constexpr float BALL_SCREW_MAX_TRAVEL_INCHES = 4.0;  // Maximum ball screw travel
    constexpr float PASS_OVERLAP_INCHES = 0.1;           // Overlap between passes
    constexpr unsigned long CLAMP_OPERATION_DELAY_MS = 300; // Delay after clamp operations
    
    // Calculate required passes based on constraints
    constexpr int REQUIRED_PASSES = static_cast<int>(
        ceil((TOTAL_WOOD_MOVEMENT_INCHES + PASS_OVERLAP_INCHES) / 
             (BALL_SCREW_MAX_TRAVEL_INCHES - PASS_OVERLAP_INCHES))
    );
}

// ============================================================================
// STATE MACHINE ENUMS
// ============================================================================
enum class FeedFirstCutState {
    INITIALIZING,
    EXECUTING_PASSES,
    COMPLETED
};

enum class PassExecutionStep {
    RETRACT_CLAMPS,
    MOVE_AWAY_FROM_HOME,
    EXTEND_CLAMPS,
    WAIT_FOR_CLAMP_DELAY,
    PUSH_WOOD_FORWARD
};

// ============================================================================
// STATE VARIABLES
// ============================================================================
namespace {
    FeedFirstCutState currentState = FeedFirstCutState::INITIALIZING;
    PassExecutionStep currentPassStep = PassExecutionStep::RETRACT_CLAMPS;
    
    int currentPassNumber = 0;
    unsigned long stepStartTimestamp = 0;
    bool isFirstPass = true;
    bool passStepInitialized = false;
    
    // Logging control
    unsigned long lastLogTimestamp = 0;
    constexpr unsigned long LOG_INTERVAL_MS = 1000;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================
namespace {
    void logState(const char* message) {
        Serial.print("FeedFirstCut: "); Serial.println(message);
    }
    
    void logStateWithValue(const char* message, int value) {
        Serial.print("FeedFirstCut: "); Serial.print(message); Serial.println(value);
    }
    
    void logStateWithFloat(const char* message, float value) {
        Serial.print("FeedFirstCut: "); Serial.print(message); Serial.println(value, 1);
    }
    
    void logStateTransition(const char* from, const char* to) {
        Serial.print("FeedFirstCut: State transition: "); Serial.print(from); 
        Serial.print(" -> "); Serial.println(to);
    }
    
    bool shouldLog() {
        unsigned long currentTime = millis();
        if (currentTime - lastLogTimestamp >= FeedFirstCutConfig::LOG_INTERVAL_MS) {
            lastLogTimestamp = currentTime;
            return true;
        }
        return false;
    }
    
    void resetPassStepVariables() {
        currentPassStep = PassExecutionStep::RETRACT_CLAMPS;
        passStepInitialized = false;
        stepStartTimestamp = 0;
    }
}

// ============================================================================
// PASS CALCULATION FUNCTIONS
// ============================================================================
namespace {
    struct PassCoordinates {
        float startPosition;
        float endPosition;
    };
    
    PassCoordinates calculatePassCoordinates(int passNumber) {
        PassCoordinates coords;
        
        if (passNumber == 1) {
            // First pass: 0" to (max travel - overlap)
            coords.startPosition = 0.0f;
            coords.endPosition = FeedFirstCutConfig::BALL_SCREW_MAX_TRAVEL_INCHES - 
                               FeedFirstCutConfig::PASS_OVERLAP_INCHES;
            logState("First pass - moving wood from 0 to 3.9 inches");
        } else {
            // Subsequent passes: continue from previous position
            float previousEnd = FeedFirstCutConfig::BALL_SCREW_MAX_TRAVEL_INCHES - 
                              FeedFirstCutConfig::PASS_OVERLAP_INCHES;
            coords.startPosition = previousEnd - FeedFirstCutConfig::PASS_OVERLAP_INCHES;
            coords.endPosition = FeedFirstCutConfig::TOTAL_WOOD_MOVEMENT_INCHES;
            logState("Second pass - moving wood from 3.8 to 5.0 inches");
        }
        
        logStateWithValue("Pass ", passNumber);
        logStateWithFloat(" - Start: ", coords.startPosition);
        logStateWithFloat(" End: ", coords.endPosition);
        
        return coords;
    }
}

// ============================================================================
// PASS EXECUTION FUNCTIONS
// ============================================================================
namespace {
    void executePassSequence(const PassCoordinates& coords) {
        if (!passStepInitialized) {
            resetPassStepVariables();
            logState("Starting new pass sequence");
        }
        
        FastAccelStepper* feedMotor = getFeedMotor();
        if (!feedMotor) return;
        
        switch (currentPassStep) {
            case PassExecutionStep::RETRACT_CLAMPS:
                logState("Step 1 - Retracting feed clamp only");
                retractFeedClamp();
                currentPassStep = PassExecutionStep::MOVE_AWAY_FROM_HOME;
                break;
                
            case PassExecutionStep::MOVE_AWAY_FROM_HOME:
                if (!feedMotor->isRunning()) {
                    logState("Step 2 - Moving away from home sensor");
                    float targetPosition = FeedFirstCutConfig::BALL_SCREW_MAX_TRAVEL_INCHES - 
                                        FeedFirstCutConfig::PASS_OVERLAP_INCHES;
                    moveFeedMotorToPosition(targetPosition);
                    currentPassStep = PassExecutionStep::EXTEND_CLAMPS;
                }
                break;
                
            case PassExecutionStep::EXTEND_CLAMPS:
                if (!feedMotor->isRunning()) {
                    logState("Step 3 - Extending feed clamp");
                    extendFeedClamp();
                    retract2x4SecureClamp();
                    stepStartTimestamp = millis();
                    currentPassStep = PassExecutionStep::WAIT_FOR_CLAMP_DELAY;
                }
                break;
                
            case PassExecutionStep::WAIT_FOR_CLAMP_DELAY:
                if (millis() - stepStartTimestamp >= FeedFirstCutConfig::CLAMP_OPERATION_DELAY_MS) {
                    logState("Step 4 - Delay complete, moving to push wood");
                    currentPassStep = PassExecutionStep::PUSH_WOOD_FORWARD;
                }
                break;
                
            case PassExecutionStep::PUSH_WOOD_FORWARD:
                if (!feedMotor->isRunning()) {
                    logStateWithFloat("Step 5 - Moving forward to push wood from ", coords.startPosition);
                    logStateWithFloat(" to ", coords.endPosition);
                    
                    float woodPushDistance = coords.endPosition - coords.startPosition;
                    float targetPosition = FeedFirstCutConfig::BALL_SCREW_MAX_TRAVEL_INCHES - 
                                        FeedFirstCutConfig::PASS_OVERLAP_INCHES - woodPushDistance;
                    
                    moveFeedMotorToPosition(targetPosition);
                    logStateWithFloat("Wood push distance: ", woodPushDistance);
                    
                    // Reset for next pass
                    passStepInitialized = false;
                }
                break;
        }
    }
}

// ============================================================================
// MAIN STATE EXECUTION
// ============================================================================
void executeFeedFirstCutState() {
    static bool isFirstExecution = true;
    
    if (isFirstExecution) {
        logState("executeFeedFirstCutState() called for first time!");
        isFirstExecution = false;
    }
    
    if (shouldLog()) {
        Serial.print("FeedFirstCut: executeFeedFirstCutState() called, currentState: ");
        Serial.println(static_cast<int>(currentState));
    }
    
    executeFeedFirstCutStep();
}

void executeFeedFirstCutStep() {
    FastAccelStepper* feedMotor = getFeedMotor();
    if (!feedMotor) return;
    
    switch (currentState) {
        case FeedFirstCutState::INITIALIZING:
            currentPassNumber = 1;
            currentState = FeedFirstCutState::EXECUTING_PASSES;
            logStateTransition("INITIALIZING", "EXECUTING_PASSES");
            executePassSequence(calculatePassCoordinates(currentPassNumber));
            break;
            
        case FeedFirstCutState::EXECUTING_PASSES:
            if (!feedMotor->isRunning()) {
                if (currentPassNumber < FeedFirstCutConfig::REQUIRED_PASSES) {
                    currentPassNumber++;
                    logStateWithValue("Pass ", currentPassNumber - 1);
                    logState(" complete, starting next pass");
                    executePassSequence(calculatePassCoordinates(currentPassNumber));
                } else {
                    logState("All passes complete, transitioning to COMPLETED");
                    currentState = FeedFirstCutState::COMPLETED;
                }
            }
            break;
            
        case FeedFirstCutState::COMPLETED:
            if (!feedMotor->isRunning()) {
                handleStateCompletion();
            }
            break;
    }
}

// ============================================================================
// STATE COMPLETION HANDLING
// ============================================================================
namespace {
    void handleStateCompletion() {
        if (getStartCycleSwitch()->read() == HIGH) {
            logState("Start cycle switch HIGH - transitioning to CUTTING state");
            changeState(CUTTING);
            setCuttingCycleInProgress(true);
            configureCutMotorForCutting();
            turnYellowLedOn();
            extendFeedClamp();
        } else {
            logState("Start cycle switch LOW - transitioning to IDLE state");
            changeState(IDLE);
        }
    }
}

// ============================================================================
// STATE ENTRY/EXIT FUNCTIONS
// ============================================================================
void onEnterFeedFirstCutState() {
    logState("onEnterFeedFirstCutState() called!");
    
    // Initialize state variables
    currentState = FeedFirstCutState::INITIALIZING;
    currentPassNumber = 0;
    stepStartTimestamp = 0;
    isFirstPass = true;
    resetPassStepVariables();
    
    logState("State variables initialized");
    logStateWithFloat("Target distance: ", FeedFirstCutConfig::TOTAL_WOOD_MOVEMENT_INCHES);
    logStateWithValue("Total passes needed: ", FeedFirstCutConfig::REQUIRED_PASSES);
    logState("State entry complete");
}

void onExitFeedFirstCutState() {
    currentState = FeedFirstCutState::INITIALIZING;
    currentPassNumber = 0;
    stepStartTimestamp = 0;
    isFirstPass = false;
    resetPassStepVariables();
    logState("Sequence complete");
}

// ============================================================================
// LEGACY SUPPORT FUNCTIONS
// ============================================================================
void testSerialOutput() {
    logState("File loaded successfully!");
}

// Legacy function names for backward compatibility
void executeFeedPass() {
    PassCoordinates coords = calculatePassCoordinates(currentPassNumber);
    executePassSequence(coords);
}

void executeSinglePass(float startPos, float endPos) {
    PassCoordinates coords{startPos, endPos};
    executePassSequence(coords);
}
