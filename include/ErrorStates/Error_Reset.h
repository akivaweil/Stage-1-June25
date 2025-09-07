#ifndef ERROR_RESET_H
#define ERROR_RESET_H

//* ************************************************************************
//* ************************** ERROR RESET **********************************
//* ************************************************************************
// Error timing constants for different error types
const unsigned long STANDARD_ERROR_BLINK_INTERVAL = 250;     // Standard error blink rate (250ms)
const unsigned long SUCTION_ERROR_BLINK_INTERVAL = 500;     // Suction error blink rate (1000ms)

// Function declaration
void handleErrorResetState();

#endif 