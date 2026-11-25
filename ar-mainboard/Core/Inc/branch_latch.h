#ifndef BRANCH_LATCH_H
#define BRANCH_LATCH_H
#include <stdbool.h>
#include <stdint.h>

/* Digital L2/R2 only (1 = black). `high` is the OR of both pins at `at`.
 * `rose` also catches a pulse that ended between scans. If set, first_rise
 * is the earliest serviced rising edge since the preceding observation. */
typedef struct {
  uint32_t at, first_rise;
  bool high, rose;
} BranchSample;

#define BRANCH_REARM_MS 75u
typedef struct {
  bool latched, timing_clear;
  uint32_t clear_since;
} BranchLatch;

static inline void BranchLatch_Start(BranchLatch *b, bool either_high) {
  *b = (BranchLatch){.latched = either_high}; /* do not count a held starting mark */
}

/* One entry per physical crossing, not one per pin or per raw IRQ. Evaluate
 * the quiet interval at the FIRST new edge, not when a later scan reads it:
 * a pulse at 74 ms must not become a new crossing at the 80 ms scan. */
static inline bool BranchLatch_Entered(BranchLatch *b, BranchSample s) {
  if (s.high || s.rose) {
    /* A sampled high without an IRQ has no known start time. Accept it if
     * armed, but never use it as proof of a completed clear interval. */
    const bool rearmed_at_rise = b->timing_clear && s.rose &&
        (uint32_t)(s.first_rise - b->clear_since) >= BRANCH_REARM_MS;
    const bool entered = !b->latched || rearmed_at_rise;
    b->latched = true;
    b->timing_clear = !s.high; /* short pulse: already back to both low */
    b->clear_since = s.at;
    return entered;
  }
  if (!b->latched) return false;
  if (!b->timing_clear) {
    b->timing_clear = true;
    b->clear_since = s.at;
  } else if ((uint32_t)(s.at - b->clear_since) >= BRANCH_REARM_MS) {
    b->latched = false;
    b->timing_clear = false;
  }
  return false;
}
#endif
