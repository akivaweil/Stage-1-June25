#include <Arduino.h>
#include "StateMachine/General_Functions.h"

// CUT-RATE STATS
// Records the timestamp of every completed cut in a ring buffer and reports the
// average cuts-per-minute over rolling 1/3/5/15-minute windows for the dashboard.
// Entries older than the longest window are pruned lazily as the buffer wraps.

static const uint32_t CUT_WINDOW_1_MS  = 1UL  * 60UL * 1000UL;
static const uint32_t CUT_WINDOW_3_MS  = 3UL  * 60UL * 1000UL;
static const uint32_t CUT_WINDOW_5_MS  = 5UL  * 60UL * 1000UL;
static const uint32_t CUT_WINDOW_15_MS = 15UL * 60UL * 1000UL;

// Capacity comfortably exceeds the most cuts possible in 15 min (a cut every
// ~2 s = 450); the oldest entries simply fall out of every window once wrapped.
static const size_t CUT_STATS_CAPACITY = 512;

static uint32_t cutTimes[CUT_STATS_CAPACITY];
static size_t   cutHead = 0;   // index of the next write
static size_t   cutFill = 0;   // valid entries, capped at CUT_STATS_CAPACITY

// Call once each time a cut physically completes.
void recordCut() {
    cutTimes[cutHead] = millis();
    cutHead = (cutHead + 1) % CUT_STATS_CAPACITY;
    if (cutFill < CUT_STATS_CAPACITY) cutFill++;
}

// Average cuts/min for each window. Walks newest→oldest and stops at the first
// entry beyond 15 min since older entries can only be older still (unsigned
// millis() subtraction stays correct across rollover for sub-49-day ages).
void getCutRates(float& perMin1, float& perMin3, float& perMin5, float& perMin15) {
    const uint32_t now = millis();
    uint16_t c1 = 0, c3 = 0, c5 = 0, c15 = 0;
    for (size_t i = 0; i < cutFill; i++) {
        const size_t idx = (cutHead + CUT_STATS_CAPACITY - 1 - i) % CUT_STATS_CAPACITY;
        const uint32_t age = now - cutTimes[idx];
        if (age > CUT_WINDOW_15_MS) break;
        c15++;
        if (age <= CUT_WINDOW_5_MS) c5++;
        if (age <= CUT_WINDOW_3_MS) c3++;
        if (age <= CUT_WINDOW_1_MS) c1++;
    }
    perMin1  = c1;            // 1-min window: count already == per-minute rate
    perMin3  = c3  / 3.0f;
    perMin5  = c5  / 5.0f;
    perMin15 = c15 / 15.0f;
}
