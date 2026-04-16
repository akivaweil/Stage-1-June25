#include "StateMachine/05_RETURNING_No_2x4.h"
#include "StateMachine/StateManager.h"
#include "StateMachine/FUNCTIONS/General_Functions.h"
#include "Config/Pin_Def.h"
#include "Config/config.h"
#include "WebSocketDashboard/websocket_dashboard.h"

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ❌ RETURNING NO 2X4 STATE — Config                                   ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
const float FEED_MOTOR_SPEED_MULTIPLIER      = 1.05;
const float FEED_MOTOR_TRAVEL_PLUS_OFFSET    = 0.15;  // Added to FEED_TRAVEL_DISTANCE on enter
const float FEED_MOTOR_2ND_POSITION          = -1.2;  // First backward target
const float FEED_MOTOR_HOME_POSITION         = 1.1;   // Forward mid-point
const float FEED_MOTOR_FINAL_POSITION        = -1.2;  // Final resting position
const unsigned long INITIAL_FEED_DELAY_MS        = 200; // Wait before moving feed motor at enter
const unsigned long AFTER_EXTEND_CLAMP_DELAY_MS  = 150; // Wait after extending feed clamp in step 1
const unsigned long AFTER_RETRACT_2X4_DELAY_MS   = 100; // Wait after retracting 2x4 secure clamp
const unsigned long AFTER_0_8_DELAY_MS           = 150; // Wait after feed motor reaches 0.8"
const unsigned long AFTER_FINAL_DELAY_MS         = 150; // Wait after feed motor reaches final pos
const unsigned long CLAMP_SETTLE_DELAY_MS        = 200; // Wait after clamp extend/retract before motor moves
const unsigned long SENSOR_CLEAR_DELAY_MS        = 300; // Wait after sensor clears before extending 2x4
unsigned long ROTATION_CLAMP_NO2X4_EXTRA_DELAY_MS = 350; // Extra delay for rotation clamp in NO_2x4 scenario (used by StateManager)

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ❌ RETURNING NO 2X4 — Step Enumeration                               ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝
// STEP_RETRACT_FEED_CLAMP       (0): Retract feed clamp
// STEP_WAIT_CUT_MOTOR_HOME      (1): Wait for cut motor to finish returning home
// STEP_INITIALIZE               (2): Start 150ms initial delay
// STEP_WAIT_INITIAL_DELAY       (3): 150ms → move feed to FEED_TRAVEL + 0.1"
// STEP_WAIT_FEED_REACH_TRAVEL   (4): Feed reaches FEED_TRAVEL + 0.1" → extend feed clamp, 150ms timer
// STEP_WAIT_AFTER_EXTEND_CLAMP  (5): 150ms → retract 2x4 secure clamp, 100ms timer
// STEP_WAIT_AFTER_RETRACT_2X4   (6): 100ms → move feed to -1.2" (clamp extended)
// STEP_WAIT_FEED_AT_NEG_1_2     (7): Feed reaches -1.2" → confirm clamp extended
// STEP_RETRACT_CLAMP_MOVE_0_8   (8): Retract feed clamp → move feed to 0.8"
// STEP_WAIT_FEED_AT_0_8         (9): Feed reaches 0.8" → retract clamp, 150ms timer
// STEP_WAIT_AFTER_0_8          (10): 150ms → extend clamp, move feed to -1.2"
// STEP_WAIT_FEED_FINAL         (11): Feed reaches -1.2" → extend clamp, 150ms timer, start reload timer
// STEP_WAIT_AFTER_FINAL        (12): 150ms → advance
// STEP_FINAL_COMPLETION        (13): Retract clamp → poll sensor → 300ms → extend 2x4 → IDLE

enum ReturningNo2x4Step {
    STEP_RETRACT_FEED_CLAMP      = 0,
    STEP_WAIT_CUT_MOTOR_HOME     = 1,
    STEP_INITIALIZE              = 2,
    STEP_WAIT_INITIAL_DELAY      = 3,
    STEP_WAIT_FEED_REACH_TRAVEL  = 4,
    STEP_WAIT_AFTER_EXTEND_CLAMP = 5,
    STEP_WAIT_AFTER_RETRACT_2X4  = 6,
    STEP_WAIT_FEED_AT_NEG_1_2    = 7,
    STEP_RETRACT_CLAMP_MOVE_0_8  = 8,
    STEP_WAIT_FEED_AT_0_8        = 9,
    STEP_WAIT_AFTER_0_8          = 10,
    STEP_WAIT_FEED_FINAL         = 11,
    STEP_WAIT_AFTER_FINAL        = 12,
    STEP_FINAL_COMPLETION        = 13
};

// Static state tracking
static int    returningNo2x4Step     = 0;
static unsigned long stepTimerStart    = 0;
static unsigned long stepTimerDuration = 0;
static bool   sensorClearedTimerActive = false;
static unsigned long sensorClearedTime = 0;

static void startStepTimer(unsigned long durationMs) {
    stepTimerStart    = millis();
    stepTimerDuration = durationMs;
}

static bool isStepTimerDone() {
    return millis() - stepTimerStart >= stepTimerDuration;
}

//╔═══╗ ════════════════════════════════════════════════════════════════ ╔═══╗
//║ ❌ RETURNING NO 2X4 STATE                                            ║
//╚═══╝ ════════════════════════════════════════════════════════════════ ╚═══╝

void executeReturningNo2x4State() {
    handleReturningNo2x4Sequence();
}

void onEnterReturningNo2x4State() {
    resetConsecutiveYeswoodCount();
    configureCutMotorForReturn();         // Cut motor already running from CUTTING state
    configureFeedMotorForNormalOperation();
    resetNoWoodLedWavePattern();

    returningNo2x4Step      = STEP_RETRACT_FEED_CLAMP;
    sensorClearedTimerActive = false;
    sensorClearedTime        = 0;
}

void onExitReturningNo2x4State() {
    resetReturningNo2x4Steps();
}

void handleReturningNo2x4Sequence() {
    handleNoWoodLedWavePattern();

    extern float FEED_TRAVEL_DISTANCE;
    FastAccelStepper* feedMotor = getFeedMotor();

    switch (returningNo2x4Step) {

        //! ************************************************************************
        //! STEP 0: Retract feed clamp
        //! ************************************************************************
        case STEP_RETRACT_FEED_CLAMP:
            retractFeedClamp();
            returningNo2x4Step = STEP_WAIT_CUT_MOTOR_HOME;
            break;

        //! ************************************************************************
        //! STEP 1: Wait for cut motor to finish returning home
        //! ************************************************************************
        case STEP_WAIT_CUT_MOTOR_HOME: {
            FastAccelStepper* cutMotor = getCutMotor();
            if (!cutMotor || !cutMotor->isRunning()) {
                returningNo2x4Step = STEP_INITIALIZE;
            }
            break;
        }

        //! ************************************************************************
        //! STEP 2: Start 150ms delay before moving feed motor
        //! ************************************************************************
        case STEP_INITIALIZE:
            startStepTimer(INITIAL_FEED_DELAY_MS);
            returningNo2x4Step = STEP_WAIT_INITIAL_DELAY;
            break;

        //! ************************************************************************
        //! STEP 3: 150ms elapsed → move feed motor to FEED_TRAVEL_DISTANCE + 0.1"
        //! ************************************************************************
        case STEP_WAIT_INITIAL_DELAY:
            if (isStepTimerDone()) {
                configureFeedMotorForSlowOperation(FEED_MOTOR_SPEED_MULTIPLIER);
                retractFeedClamp();
                delay(CLAMP_SETTLE_DELAY_MS);
                moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE + FEED_MOTOR_TRAVEL_PLUS_OFFSET);
                returningNo2x4Step = STEP_WAIT_FEED_REACH_TRAVEL;
            }
            break;

        //! ************************************************************************
        //! STEP 4: Feed motor reaches FEED_TRAVEL + 0.1" → extend feed clamp → 150ms timer
        //! ************************************************************************
        case STEP_WAIT_FEED_REACH_TRAVEL:
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                startStepTimer(AFTER_EXTEND_CLAMP_DELAY_MS);
                returningNo2x4Step = STEP_WAIT_AFTER_EXTEND_CLAMP;
            }
            break;

        //! ************************************************************************
        //! STEP 5: 150ms elapsed → retract 2x4 secure clamp → 100ms timer
        //! ************************************************************************
        case STEP_WAIT_AFTER_EXTEND_CLAMP:
            if (isStepTimerDone()) {
                retract2x4SecureClamp();
                startStepTimer(AFTER_RETRACT_2X4_DELAY_MS);
                returningNo2x4Step = STEP_WAIT_AFTER_RETRACT_2X4;
            }
            break;

        //! ************************************************************************
        //! STEP 6: 100ms elapsed → extend feed clamp, move feed motor to -1.2"
        //! ************************************************************************
        case STEP_WAIT_AFTER_RETRACT_2X4:
            if (isStepTimerDone()) {
                configureFeedMotorForSlowOperation(FEED_MOTOR_SPEED_MULTIPLIER);
                extendFeedClamp();
                delay(CLAMP_SETTLE_DELAY_MS);
                moveFeedMotorToPosition(FEED_MOTOR_2ND_POSITION);
                returningNo2x4Step = STEP_WAIT_FEED_AT_NEG_1_2;
            }
            break;

        //! ************************************************************************
        //! STEP 7: Feed motor reaches -1.2" → confirm feed clamp extended
        //! ************************************************************************
        case STEP_WAIT_FEED_AT_NEG_1_2:
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                returningNo2x4Step = STEP_RETRACT_CLAMP_MOVE_0_8;
            }
            break;

        //! ************************************************************************
        //! STEP 8: Retract feed clamp → move feed motor to 0.8"
        //! ************************************************************************
        case STEP_RETRACT_CLAMP_MOVE_0_8:
            configureFeedMotorForSlowOperation(FEED_MOTOR_SPEED_MULTIPLIER);
            retractFeedClamp();
            delay(CLAMP_SETTLE_DELAY_MS);
            moveFeedMotorToPosition(FEED_MOTOR_HOME_POSITION);
            returningNo2x4Step = STEP_WAIT_FEED_AT_0_8;
            break;

        //! ************************************************************************
        //! STEP 9: Feed motor reaches 0.8" → retract feed clamp → 150ms timer
        //! ************************************************************************
        case STEP_WAIT_FEED_AT_0_8:
            if (feedMotor && !feedMotor->isRunning()) {
                retractFeedClamp();
                startStepTimer(AFTER_0_8_DELAY_MS);
                returningNo2x4Step = STEP_WAIT_AFTER_0_8;
            }
            break;

        //! ************************************************************************
        //! STEP 10: 150ms elapsed → extend feed clamp, move feed motor to -1.2"
        //! ************************************************************************
        case STEP_WAIT_AFTER_0_8:
            if (isStepTimerDone()) {
                configureFeedMotorForSlowOperation(FEED_MOTOR_SPEED_MULTIPLIER);
                extendFeedClamp();
                delay(CLAMP_SETTLE_DELAY_MS);
                moveFeedMotorToPosition(FEED_MOTOR_FINAL_POSITION);
                returningNo2x4Step = STEP_WAIT_FEED_FINAL;
            }
            break;

        //! ************************************************************************
        //! STEP 11: Feed motor reaches -1.2" → extend clamp, start reload timer, 150ms timer
        //! ************************************************************************
        case STEP_WAIT_FEED_FINAL:
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                startReloadTimer();
                startStepTimer(AFTER_FINAL_DELAY_MS);
                returningNo2x4Step = STEP_WAIT_AFTER_FINAL;
            }
            break;

        //! ************************************************************************
        //! STEP 12: 150ms elapsed → advance to final completion
        //! ************************************************************************
        case STEP_WAIT_AFTER_FINAL:
            if (isStepTimerDone()) {
                returningNo2x4Step = STEP_FINAL_COMPLETION;
            }
            break;

        //! ************************************************************************
        //! STEP 13: Retract feed clamp → poll wood sensor → 300ms → extend 2x4 → IDLE
        //! ************************************************************************
        case STEP_FINAL_COMPLETION:
            retractFeedClamp();

            if (!sensorClearedTimerActive) {
                // Wait for wood sensor to read clear (HIGH = no wood present)
                if (getWoodPresentSensorBounce()->read() == HIGH) {
                    sensorClearedTime        = millis();
                    sensorClearedTimerActive = true;
                }
            } else {
                if (millis() - sensorClearedTime >= SENSOR_CLEAR_DELAY_MS) {
                    extend2x4SecureClamp();
                    setComingFromNoWoodWithSensorsClear(true);
                    resetReturningNo2x4Steps();
                    incrementCuttingCycleCounter();
                    setCuttingCycleInProgress(false);

                    if (getStartCycleSwitch()->read() == HIGH) {
                        setStartSwitchSafe(false);
                    }

                    changeState(IDLE);
                }
            }
            break;
    }
}

void resetReturningNo2x4Steps() {
    returningNo2x4Step       = 0;
    stepTimerStart           = 0;
    stepTimerDuration        = 0;
    sensorClearedTimerActive = false;
    sensorClearedTime        = 0;
}
