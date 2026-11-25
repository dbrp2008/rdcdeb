/* Optional second-crossing right turn. Decisions only: the board owns both motors. */
#include "function2.h"
#include "function1.h" /* F1_LEGACY */
#include "line_drive.h"

#define F2_LINE_GAP_MS 75u /* original analog clearance for pivot exit, not BR rearm */

typedef enum { F2_IDLE, F2_FOLLOW, F2_ADVANCE, F2_TURN_PENDING, F2_TURN_GAP, F2_TURN_LINE, F2_DONE } F2_State;

typedef struct {
  F2_State state;
  unsigned branches;
  BranchLatch crossing;
  bool turned, timing_line_gap, saw_centre_white;
  uint32_t line_clear_since;
  const char *status;
  float pivot_deg;   /* encoder angle of the pivot so far (< 0 = unknown) */
  bool failed;
  unsigned timed;    /* F2_Tick: 0 = not timing, 1 = advancing, 2 = turning (PENDING/GAP/LINE as one) */
  uint32_t timed_at; /* ...since when */
} F2_App;

static F2_App app;

void F2_Init(void) {
  app = (F2_App){.state = F2_IDLE, .status = "F2 READY", .pivot_deg = -1.0f};
}

void F2_Start(bool either_side_high) {
  app = (F2_App){.state = F2_FOLLOW, .status = "F2 FOLLOW", .pivot_deg = -1.0f};
  BranchLatch_Start(&app.crossing, either_side_high);
}

void F2_Stop(const char *reason) {
  app.state = F2_DONE;
  app.status = reason;
}

void F2_Update(BranchSample side, unsigned filtered_line) {
  if (app.state == F2_IDLE || app.state == F2_DONE) return;
  const bool entered = BranchLatch_Entered(&app.crossing, side);
  if (app.state == F2_FOLLOW) {
    if (entered) {
      ++app.branches;
      if (app.branches == F2_TURN_AT_BRANCH && !app.turned) {
        app.turned = true; /* only once per run */
#if !F1_LEGACY
        if (F2_T_OUT_ADVANCE_MM > 0.0f) {
          app.state = F2_ADVANCE; /* past the T first, then pivot (F2_Advanced) */
          app.status = "F2 PAST THE T";
        } else
#endif
        {
          app.state = F2_TURN_PENDING;
          app.status = "F2 TURN RIGHT";
        }
      } else {
        app.status = app.branches == 1u ? "F2 BRANCH 1" : "F2 FOLLOW";
      }
    }
    return; /* the crossing scan predates the pivot command */
  }
  if (app.state != F2_TURN_GAP && app.state != F2_TURN_LINE) return;

  /* During the commanded pivot, require BOTH junction clearance and a fresh
   * centre-white -> centre-black sequence. Consume but never queue side hits. */
  const bool centre_black = (filtered_line & LINE_CENTRE) != 0u;
  if (!centre_black) app.saw_centre_white = true;
  if (app.state == F2_TURN_GAP) {
    if (Line_MultipleBlack(filtered_line)) {
      app.timing_line_gap = false;
      return;
    }
    if (!app.timing_line_gap) {
      app.timing_line_gap = true;
      app.line_clear_since = side.at;
    }
    if ((uint32_t)(side.at - app.line_clear_since) < F2_LINE_GAP_MS) return;
    app.state = F2_TURN_LINE;
    app.status = "F2 FIND LINE";
  }
  /* Centre may return on the same scan that completes the 75 ms gap. */
#if !F1_LEGACY
  if (app.pivot_deg >= 0.0f && app.pivot_deg < F2_TURN_MIN_DEG) return; /* too early: not the branch yet */
#endif
  if (app.saw_centre_white && centre_black) {
    app.state = F2_FOLLOW;
    app.status = "F2 FOLLOW";
  }
}

bool F2_Advancing(void) { return app.state == F2_ADVANCE; }
void F2_Advanced(float mm) {
  if (app.state == F2_ADVANCE && mm >= F2_T_OUT_ADVANCE_MM) {
    app.state = F2_TURN_PENDING;
    app.status = "F2 TURN RIGHT";
  }
}

void F2_PivotAngle(float deg) {
#if !F1_LEGACY
  if (!F2_Turning() || app.state == F2_ADVANCE) return;
  app.pivot_deg = deg;
  if (deg >= 0.0f && deg > F2_TURN_MAX_DEG) { /* no tape where the branch should be: stop rather than spin */
    app.state = F2_DONE;
    app.failed = true;
    app.status = "F2 TURN NO TAPE";
  }
#else
  (void)deg;
#endif
}
bool F2_Failed(void) { return app.failed; }

/* Time backups: the advance ends on encoder distance and the pivot's "no tape" stop on encoder angle - a dead encoder
 * reaches neither, so the robot would drive or spin forever. Called every drive tick while F2 runs. */
void F2_Tick(uint32_t now) {
  const unsigned group = app.state == F2_ADVANCE ? 1u :
                         (app.state == F2_TURN_PENDING || app.state == F2_TURN_GAP || app.state == F2_TURN_LINE) ? 2u : 0u;
  if (group != app.timed) { app.timed = group; app.timed_at = now; return; }
  if (group == 0u) return;
  const uint32_t limit = group == 1u ? F2_ADVANCE_MAX_MS : F2_TURN_MAX_MS;
  if ((uint32_t)(now - app.timed_at) >= limit) {
    app.state = F2_DONE;
    app.failed = true;
    app.status = group == 1u ? "F2 ADV TIMEOUT" : "F2 TURN TIMEOUT";
  }
}

void F2_PivotCommanded(void) {
  if (app.state == F2_TURN_PENDING) app.state = F2_TURN_GAP;
}

unsigned F2_BranchCount(void) { return app.branches; }
bool F2_Turning(void) {
  return app.state == F2_ADVANCE || app.state == F2_TURN_PENDING || app.state == F2_TURN_GAP || app.state == F2_TURN_LINE;
}
bool F2_Done(void) { return app.state == F2_DONE; }
const char *F2_Status(void) { return app.status; }
