#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include "StateMachine/04_YESWOOD.h"
#include "StateMachine/03_CUTTING.h"  // for isWoodPresent()
#include "StateMachine/StateManager.h"
#include "StateMachine/General_Functions.h"
#include "Config/Pins_Definitions.h"
#include "Config/Config.h"
#include "WebDashboard/WebDashboard.h"

// YESWOOD STATE
// Handles the simultaneous return sequence when wood sensor detects lumber.
// Manages cut motor return to home while feed motor executes multi-step return sequence.
// Includes final feed wood movement to configured distance before transitioning to next cycle or IDLE.
//
// Feed clamp extension occurs immediately after feed motor completion.

// Static variables for YESWOOD state tracking
static int yeswoodSubStep = 0;
static int feedMotorReturnSubStep = 0; // For initial feed motor return sequence
static unsigned long stepStartTime = 0; // For 200ms delay

// Cut motor homing recovery timing
static unsigned long cutMotorHomingAttemptStartTime = 0;
static bool cutMotorHomingAttemptInProgress = false;
static float cutMotorIncrementalMoveTotalInches = 0.0;

// FEED PULLBACK (YESWOOD-only)
// At YESWOOD entry, briefly pull the wood back via the feed motor while the
// top (secure) clamp is retracted. The feed clamp stays extended so its grip
// drags the wood with the motor. The forward feed stroke later in YESWOOD
// adds FEED_PULLBACK_FEED_COMPENSATION so the wood lands at the same final
// position as a non-pullback cycle.
//
// 0 = idle/done; 1..6 = active phases (see handleYeswoodPullback).
static int yeswoodPullbackStep = 0;
static unsigned long yeswoodPullbackTimer = 0;

// Time given for the top-clamp solenoid to physically release before any
// motor motion. Matches the system's standard cylinder-action delay.
static const unsigned long YESWOOD_TOP_CLAMP_RELEASE_DELAY_MS = 150;

// After the pullback completes and the top clamp re-extends, wait this long
// before commanding the cut motor home — gives the top clamp time to fully
// engage so the wood is held secure during the return.
static const unsigned long YESWOOD_CUT_MOTOR_RETURN_DELAY_MS = 150;

// POST-FORWARD FEED PULLBACK PREP (parallel to CUTTING step 0)
// Runs autonomously (driven by tickYeswoodPullbackPrep, called by
// executeStateMachine after the state dispatch each loop). Armed at the end
// of YESWOOD's continuous-mode branch right before changeState(CUTTING).
// Sequence: retract feed clamp → wait 150ms → feed motor 0.5" toward 0
// (away from home) → wait for motion → extend feed clamp. The wood stays
// held by the top clamp (which CUTTING step 0 extends one-shot at entry)
// during the brief feed-clamp retract window.
//
// 0 = idle/done; 1..5 = active phases.
static int yeswoodPullbackPrepStep = 0;
static unsigned long yeswoodPullbackPrepTimer = 0;
static const float YESWOOD_FEED_BACKSTEP_INCHES = 1.5f;
// Hold the feed clamp retracted for this long (after CUTTING step 0 has
// done its one-shot clamp extension) before issuing the 0.5" backstep, so
// the solenoid is fully released and the wood doesn't get dragged back.
static const unsigned long YESWOOD_FEED_BACKSTEP_CLAMP_DELAY_MS = 300;

// Settle delay after extending the feed clamp before moving the feed motor forward.
// Lets the clamp fully grip before the wood is pushed to FEED_TRAVEL_DISTANCE.
static const unsigned long FEED_CLAMP_EXTEND_SETTLE_MS = 300;

// Distance-from-home (inches, cut motor) at which we extend the feed clamp and
// start the settle timer — early, so the 300ms grip-settle overlaps the cut
// motor's final approach. The forward feed-motor move still gates on the home
// switch, so wood doesn't push forward until the cut is truly home.
static const float CUT_MOTOR_FEED_CLAMP_PREP_INCHES = 3.0f;

// Distance-from-home (inches, cut motor) at which we retract the top clamp so
// its release delay overlaps the cut motor's last bit of travel. Kept tight so
// the wood is effectively held by the top clamp "the entire way home."
static const float CUT_MOTOR_TOP_CLAMP_RETRACT_PREP_INCHES = 0.25f;

// Shorter release delay used on the YESWOOD return-to-forward handoff. The
// general YESWOOD_TOP_CLAMP_RELEASE_DELAY_MS (150ms) stays in place for the
// pullback sequence at YESWOOD entry, which is more sensitive.
static const unsigned long YESWOOD_RETURN_TOP_CLAMP_RELEASE_MS = 100;

// Hold the feed clamp extended this many ms after the top (secure) clamp is
// re-extended at the end of YESWOOD before transitioning to IDLE. Gives the
// top clamp time to physically engage the wood before the feed clamp lets go.
// Applied ONLY on the YESWOOD->IDLE branch — does not affect continuous-mode
// (run-cycle switch on) timing.
static const unsigned long YESWOOD_TO_IDLE_FEED_CLAMP_HOLD_MS = 400;

// Number of cut-home-switch reads when verifying the cut motor reached home
// after its return, and the per-read settle delay between them.
static const int CUT_HOME_VERIFICATION_ATTEMPTS = 3;
static const unsigned long CUT_HOME_VERIFICATION_READ_DELAY_MS = 5;

// Reset the consecutive-yeswood counter once it reaches this many cycles.
static const int CONSECUTIVE_YESWOOD_RESET_THRESHOLD = 3;

void handleYeswoodState() {
    handleYeswoodSequence();
}

void onEnterYeswoodState() {
    // STEP 1: START CUT MOTOR RETURN (TOP CLAMP REMAINS EXTENDED)

    // Increment consecutive yeswood counter
    incrementConsecutiveYeswoodCount();

    // Enable cut motor homing sensor monitoring during return
    extern bool cutMotorInYeswoodReturn;
    cutMotorInYeswoodReturn = true;

    // Cut motor already started in CUTTING state
    // Top clamp remains extended during first feed motor movement

    // Initialize step tracking
    yeswoodSubStep = 0;
    feedMotorReturnSubStep = 0;
    stepStartTime = 0;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
    cutMotorIncrementalMoveTotalInches = 0.0;

    // TEMP: pullback disabled regardless of dashboard FEED_PULLBACK_DISTANCE.
    // Forward-move compensation at YESWOOD step 9 is also dropped to keep
    // the wood landing in the same final position. Restore both together.
    yeswoodPullbackStep  = 0;
    yeswoodPullbackTimer = 0;

    // The pullback FSM normally kicked off the cut motor return at the end of
    // its sequence (after re-extending the top clamp). With pullback skipped,
    // the top clamp was never retracted in CUTTING/YESWOOD entry, so we can
    // start the cut motor return immediately.
    startCutMotorReturnSequence();
}

void onExitYeswoodState() {
    resetYeswoodSteps();
}

// Main Sequence Handler
// Manages the complete YESWOOD sequence through multiple substeps

// Forward declaration so handleYeswoodSequence can gate on it.
static void handleYeswoodPullback();

void handleYeswoodSequence() {
    // Run the wood-pullback sequence first, gating the rest until it completes.
    if (yeswoodPullbackStep != 0) {
        handleYeswoodPullback();
        return;
    }

    FastAccelStepper* feedMotor = getFeedMotor();
    FastAccelStepper* cutMotor = getCutMotor();
    extern bool cutMotorInYeswoodReturn;

    switch (yeswoodSubStep) {
        case 0: // Execute feed motor return sequence (without homing)
            handleFeedMotorReturnSequence();
            break;

        case 1: // Wait for feed motor to complete forward travel, then proceed to cut motor wait
            if (feedMotor && !feedMotor->isRunning()) {
                yeswoodSubStep = 2;
            }
            break;

        case 2: // Wait for cut motor completion
            // Wait for cut motor to complete return home, then execute homing sequence
            if (cutMotor && !cutMotor->isRunning() && !cutMotorHomingAttemptInProgress) {
                // STEP 3: CUT MOTOR RETURN COMPLETE - START HOMING VERIFICATION SEQUENCE
                cutMotorInYeswoodReturn = false;
                
                bool sensorDetectedHome = false;
                
                // Execute homing verification sequence - 3-attempt verification
                for (int i = 0; i < CUT_HOME_VERIFICATION_ATTEMPTS; i++) {
                    delay(CUT_HOME_VERIFICATION_READ_DELAY_MS);
                    getCutHomingSwitch()->update();
                    bool sensorReading = getCutHomingSwitch()->read();
                    
                    if (sensorReading == HIGH) {
                        sensorDetectedHome = true;
                        break;
                    }
                }
                
                if (sensorDetectedHome) {
                // ************************************************************************
                // STEP 4: HOMING VERIFIED - SET POSITION TO 0 AND PROCEED TO COMPLETION
                // ************************************************************************
                    if (cutMotor) cutMotor->setCurrentPosition(0);
                    cutMotorIncrementalMoveTotalInches = 0.0; // Reset on success
                    
                    // Feed motor already at travel distance from return sequence, proceed to completion
                    yeswoodSubStep = 3;
                } else {
                    // Home switch not detected - try incremental move recovery
                    
                    if (cutMotorIncrementalMoveTotalInches < CUT_MOTOR_MAX_INCREMENTAL_MOVE_INCHES) {
                        if (cutMotor) {
                            cutMotor->move(-CUT_MOTOR_INCREMENTAL_MOVE_INCHES * CUT_MOTOR_STEPS_PER_INCH);
                            cutMotorIncrementalMoveTotalInches += CUT_MOTOR_INCREMENTAL_MOVE_INCHES;
                        }
                        // Stay in same step to re-check sensor after move
                    } else {
                        // Max incremental moves exceeded - transition to error
                        Serial.println("[Stage1] FAULT: cut motor home not detected after max incremental moves");
                        onErrorOccurred("Cut motor home switch not detected after max moves");
                        if (cutMotor) cutMotor->forceStop();
                        if (feedMotor) feedMotor->forceStop();
                        showRedLed();
                        turnYellowLedOff();
                        changeState(STATE_ERROR);
                        setErrorStartTime(millis());
                        resetYeswoodSteps();
                        return;
                    }
                }
            }
            break;
            
        case 3: // Feed motor already at travel distance - extend top clamp
            // ************************************************************************
            // STEP 4: EXTEND TOP CLAMP AFTER FEED WOOD MOVEMENT COMPLETE
            // ************************************************************************
            extendTopClamp();
            yeswoodSubStep = 4;
            break;
            
        case 4: // Complete sequence - check for continuous operation or return to IDLE
            if (feedMotor && !feedMotor->isRunning()) {
                // ************************************************************************
                // STEP 5: SEQUENCE COMPLETE - CHECK FOR CONTINUOUS OPERATION OR RETURN TO IDLE
                // ************************************************************************
                turnYellowLedOff();
                incrementCuttingCycleCounter();
                setCuttingCycleInProgress(false);
                
                // Reset consecutive yeswood counter only when it reaches the threshold
                if (getConsecutiveYeswoodCount() >= CONSECUTIVE_YESWOOD_RESET_THRESHOLD) {
                    resetConsecutiveYeswoodCount();
                }
                
                
                // Check for continuous operation mode
                if (getStartCycleSwitch()->read() == HIGH && getStartSwitchSafe()) {
                    extendFeedClamp();
                    configureCutMotorForCutting();
                    showYellowLed();
                    setCuttingCycleInProgress(true);
                    // TEMPORARILY DISABLED: post-forward 1.5" backstep prep.
                    // To re-enable, restore: yeswoodPullbackPrepStep = 1;
                    // yeswoodPullbackPrepStep  = 1;
                    yeswoodPullbackPrepTimer = 0;
                    changeState(STATE_CUTTING);
                    resetYeswoodSteps();
                } else {
                    // Going to IDLE: keep feed clamp extended an extra 300ms so the
                    // top (secure) clamp has a chance to physically extend before
                    // IDLE entry retracts the feed clamp. Continuous mode above is
                    // intentionally NOT delayed.
                    stepStartTime = millis();
                    yeswoodSubStep = 5;
                }
            }
            break;

        case 5: // Hold feed clamp extended after top-clamp re-extension before IDLE transition
            if (millis() - stepStartTime >= YESWOOD_TO_IDLE_FEED_CLAMP_HOLD_MS) {
                changeState(STATE_IDLE);
                resetYeswoodSteps();
            }
            break;
    }
}

// Feed Motor Return Sequence
// Handles the feed motor return sequence during simultaneous operation (no homing)

void handleFeedMotorReturnSequence() {
    FastAccelStepper* feedMotor = getFeedMotor();
    
    switch (feedMotorReturnSubStep) {
        case 0: // Retract feed clamp first
            // ************************************************************************
            // STEP 6: RETRACT FEED CLAMP
            // ************************************************************************
            retractFeedClamp();
            feedMotorReturnSubStep = 1;
            break;
            
        case 1: // Move feed motor to position 0 (pulled-back / load end — opposite of physical home)
            if (feedMotor && !feedMotor->isRunning()) {
                // ************************************************************************
                // STEP 7: MOVE FEED MOTOR TO POSITION 0 (PULLED-BACK END)
                // ************************************************************************
                configureFeedMotorForNormalOperation();
                moveFeedMotorToZero();
                feedMotorReturnSubStep = 2;
            }
            break;

        case 2: // Wait for pull-back move AND cut motor within prep distance of home
            if (feedMotor && !feedMotor->isRunning()) {
                // Extend the feed clamp and start the settle timer early — when
                // the cut motor is within CUT_MOTOR_FEED_CLAMP_PREP_INCHES of
                // home — so the 300ms grip-settle overlaps the cut motor's final
                // travel. Top clamp stays extended for the full return; it only
                // retracts in case 3 when the cut motor actually reaches home.
                FastAccelStepper* cutMotor = getCutMotor();
                long prepThresholdSteps =
                    (long)(CUT_MOTOR_FEED_CLAMP_PREP_INCHES * CUT_MOTOR_STEPS_PER_INCH);
                if (cutMotor && cutMotor->getCurrentPosition() <= prepThresholdSteps) {
                    // ************************************************************************
                    // STEP 8: CUT MOTOR NEAR HOME - EXTEND FEED CLAMP (TOP CLAMP STAYS)
                    // ************************************************************************
                    extendFeedClamp();
                    stepStartTime = millis();
                    feedMotorReturnSubStep = 3;
                }
            }
            break;

        case 3: { // Retract top clamp ~0.25" before home so its release overlaps the last bit of travel
            // Safety check: feed clamp must have had time to settle before we
            // hand off the wood by releasing the top clamp.
            bool minDelayMet = (millis() - stepStartTime >= FEED_CLAMP_EXTEND_SETTLE_MS);

            FastAccelStepper* cutMotor = getCutMotor();
            long topClampRetractSteps =
                (long)(CUT_MOTOR_TOP_CLAMP_RETRACT_PREP_INCHES * CUT_MOTOR_STEPS_PER_INCH);
            bool cutMotorAtRetractPoint =
                cutMotor && cutMotor->getCurrentPosition() <= topClampRetractSteps;

            if (minDelayMet && cutMotorAtRetractPoint) {
                // ************************************************************************
                // STEP 9: 0.25" FROM HOME - RETRACT TOP CLAMP (FEED CLAMP NOW HOLDS WOOD)
                // ************************************************************************
                retractTopClamp();
                stepStartTime = millis();
                feedMotorReturnSubStep = 4;
            }
            break;
        }

        case 4: // Wait for top-clamp release AND cut motor home before pushing the wood forward
            getCutHomingSwitch()->update();
            if (millis() - stepStartTime >= YESWOOD_RETURN_TOP_CLAMP_RELEASE_MS &&
                getCutHomingSwitch()->read() == HIGH &&
                feedMotor && !feedMotor->isRunning()) {
                // ************************************************************************
                // STEP 10: TOP CLAMP RELEASED - MOVE TO TRAVEL DISTANCE
                // ************************************************************************
                // TEMP: pullback disabled at YESWOOD entry, so no compensation
                // needed here. Restore '+ FEED_PULLBACK_FEED_COMPENSATION' when
                // the pullback is re-enabled in onEnterYeswoodState.
                moveFeedMotorToPosition(FEED_TRAVEL_DISTANCE);
                yeswoodSubStep = 1;
            }
            break;
    }
}

// Wood Pullback Sequence
// At YESWOOD entry, retract the top clamp, pull the feed motor (and the wood
// it's gripping via the still-extended feed clamp) back by FEED_PULLBACK_DISTANCE,
// then re-extend the top clamp. Runs once per YESWOOD entry; gates the main
// sequence until it finishes.

static void handleYeswoodPullback() {
    FastAccelStepper* feedMotor = getFeedMotor();

    switch (yeswoodPullbackStep) {
        case 1: // Retract top clamp so the wood can be dragged back
            retractTopClamp();
            yeswoodPullbackTimer = millis();
            yeswoodPullbackStep = 2;
            break;

        case 2: // Wait for solenoid release before any motion
            if (millis() - yeswoodPullbackTimer >= YESWOOD_TOP_CLAMP_RELEASE_DELAY_MS) {
                if (feedMotor) {
                    float currentInches =
                        (float)feedMotor->getCurrentPosition() / FEED_MOTOR_STEPS_PER_INCH;
                    moveFeedMotorToPosition(currentInches - FEED_PULLBACK_DISTANCE);
                }
                yeswoodPullbackStep = 3;
            }
            break;

        case 3: // Wait for the pullback move to complete
            if (feedMotor && !feedMotor->isRunning()) {
                yeswoodPullbackStep = 4;
            }
            break;

        case 4: // Re-extend top clamp, then start the cut-motor-return delay
            extendTopClamp();
            yeswoodPullbackTimer = millis();
            yeswoodPullbackStep = 5;
            break;

        case 5: // Wait 150 ms after top-clamp re-extend before starting cut motor return
            if (millis() - yeswoodPullbackTimer >= YESWOOD_CUT_MOTOR_RETURN_DELAY_MS) {
                // Now command the cut motor home — deferred from CUTTING so the
                // wood is securely held by the top clamp before any cut-side motion.
                startCutMotorReturnSequence();
                yeswoodPullbackStep = 0; // done; main YESWOOD FSM takes over
            }
            break;
    }
}

// Post-forward Feed Pullback Prep (parallel)
// Armed at the end of YESWOOD's continuous-mode branch and ticked from
// executeStateMachine() each loop iteration. Runs concurrently with CUTTING
// step 0 so the cut cycle isn't slowed down by this housekeeping.

void tickYeswoodPullbackPrep() {
    if (yeswoodPullbackPrepStep == 0) return;

    FastAccelStepper* feedMotor = getFeedMotor();

    switch (yeswoodPullbackPrepStep) {
        case 1: // Wait until CUTTING step 0 has done its one-shot clamp extension.
                // If we retracted before that fired, step 0 would override and
                // re-extend the clamp, dragging the wood when we move the motor.
            if (getCurrentState() == STATE_CUTTING && cuttingClampsExtended()) {
                retractFeedClamp();
                yeswoodPullbackPrepTimer = millis();
                yeswoodPullbackPrepStep = 2;
            }
            break;

        case 2: // Hold retracted long enough for the solenoid to fully release,
                // then move the feed motor away from home (toward 0 / load end)
            if (millis() - yeswoodPullbackPrepTimer >= YESWOOD_FEED_BACKSTEP_CLAMP_DELAY_MS) {
                if (feedMotor && !feedMotor->isRunning()) {
                    // Decrease position = away from home (matches the proven
                    // pattern used by handleYeswoodPullback at YESWOOD entry).
                    float currentInches =
                        (float)feedMotor->getCurrentPosition() / FEED_MOTOR_STEPS_PER_INCH;
                    moveFeedMotorToPosition(currentInches - YESWOOD_FEED_BACKSTEP_INCHES);
                    yeswoodPullbackPrepStep = 3;
                }
            }
            break;

        case 3: // Wait for the backstep to complete, then re-extend the feed clamp
            if (feedMotor && !feedMotor->isRunning()) {
                extendFeedClamp();
                yeswoodPullbackPrepTimer = millis();
                yeswoodPullbackPrepStep = 4;
            }
            break;

        case 4: // Brief settle delay so the clamp grips before anything else acts
            if (millis() - yeswoodPullbackPrepTimer >= YESWOOD_FEED_BACKSTEP_CLAMP_DELAY_MS) {
                yeswoodPullbackPrepStep = 0; // done
            }
            break;
    }
}

// Utility Functions

void resetYeswoodSteps() {
    yeswoodSubStep = 0;
    feedMotorReturnSubStep = 0;
    stepStartTime = 0;
    cutMotorHomingAttemptStartTime = 0;
    cutMotorHomingAttemptInProgress = false;
    cutMotorIncrementalMoveTotalInches = 0.0;
    yeswoodPullbackStep = 0;
    yeswoodPullbackTimer = 0;
}