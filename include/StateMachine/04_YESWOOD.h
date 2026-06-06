#pragma once

#include "StateMachine/General_Functions.h"

// Yeswood State
// Handles the simultaneous return sequence when wood sensor detects lumber.
// Manages cut motor return to home while feed motor executes multi-step return sequence.
// Includes final feed wood movement to 3.4 inches before transitioning to next cycle or IDLE.

// Function declarations for YESWOOD state
void handleYeswoodState();
void onEnterYeswoodState();
void onExitYeswoodState();

// Helper function declarations for YESWOOD sequence
void handleYeswoodSequence();
void handleFeedMotorReturnSequence();
void handleFeedWoodMovement();

// Reset all step counters
void resetYeswoodSteps();

// Autonomous mini-FSM that runs in parallel with the YESWOOD→CUTTING
// transition (continuous mode). Called every loop iteration by
// executeStateMachine() so it ticks regardless of currentState.
// Briefly retracts the feed clamp after the YESWOOD forward stroke, moves
// the feed motor 0.5" backward (away from home), and re-extends the clamp.
// Designed to overlap with CUTTING step 0's setup; the wood is held by the
// top clamp during the retract window.
void tickYeswoodPullbackPrep();
