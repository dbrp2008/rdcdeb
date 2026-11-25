/* F3 chooses a visible right exit, not a numbered crossing or a timed angle. */
#include "function3.h"
#include <stdbool.h>

enum { LEFT = 1u, CENTRE = 2u, RIGHT = 4u, ALL = LEFT | CENTRE | RIGHT };
typedef enum { FOLLOW, LEAVE_JUNCTION, ALIGN_BRANCH } TurnPhase;
static struct {
  TurnPhase phase;
  F3_Command command;
  unsigned turns;
  uint32_t turn_at, centre_at, wait_at;
  bool confirming_centre, waiting;
} f3;

void F3_Reset(void) {
  f3.phase = FOLLOW;
  f3.command = F3_WAIT_TAPE;
  f3.turns = 0u;
  f3.confirming_centre = f3.waiting = false;
}
unsigned F3_RightTurns(void) { return f3.turns; }
static bool terminal(F3_Command cmd) {
  return cmd == F3_LOST_LINE || cmd == F3_UNSTABLE_LINE || cmd == F3_TURN_TIMEOUT;
}
F3_Command F3_CommandAt(uint32_t now) {
  if (!terminal(f3.command)) {
    if (f3.phase != FOLLOW && (uint32_t)(now - f3.turn_at) >= F3_TURN_LIMIT_MS)
      f3.command = F3_TURN_TIMEOUT;
    else if (f3.waiting && (uint32_t)(now - f3.wait_at) >= F3_FILTER_WAIT_MS)
      f3.command = F3_UNSTABLE_LINE;
  }
  return f3.command;
}

void F3_Scan(uint32_t now, unsigned raw, unsigned filtered) {
  if (terminal(F3_CommandAt(now))) return; /* never resume a terminal command */
  raw &= ALL; filtered &= ALL;
  const unsigned tape = raw & filtered;
  if (!raw) { f3.command = F3_LOST_LINE; return; } /* even if the filter still says black */
  if (!tape) {
    /* Raw tape can move to another sensor before its average catches up.
     * Wait with both wheels stopped, never drive on the stale side. */
    if (!f3.waiting) { f3.waiting = true; f3.wait_at = now; }
    f3.confirming_centre = false;
    f3.command = F3_WAIT_TAPE;
    return;
  }
  f3.waiting = false;

  /* WWB alone is ordinary right correction. WBB, BWB and BBB can indicate
   * a right branch. Check on every scan: BBW can widen to BBB at one fork. */
  const bool right_branch = (tape & RIGHT) && (tape & (CENTRE | LEFT));
  if (right_branch) {
    if (f3.phase == FOLLOW) { ++f3.turns; f3.turn_at = now; }
    f3.phase = LEAVE_JUNCTION;
    f3.confirming_centre = false;
  }
  if (f3.phase == LEAVE_JUNCTION) {
    /* Seeing the old centre line is not proof of a completed right turn. */
    if (raw != RIGHT || filtered != RIGHT) { f3.command = F3_PIVOT_RIGHT; return; }
    f3.phase = ALIGN_BRANCH;
  }
  if (f3.phase == ALIGN_BRANCH) {
    /* Track the separated right-hand tape back to centre. A recurring broad
     * patch before confirmation keeps the same turn and original deadline. */
    if (raw == CENTRE && filtered == CENTRE) {
      if (!f3.confirming_centre) { f3.confirming_centre = true; f3.centre_at = now; }
      if ((uint32_t)(now - f3.centre_at) >= F3_CENTRE_CONFIRM_MS) {
        f3.phase = FOLLOW;
        f3.confirming_centre = false;
      }
    } else {
      f3.confirming_centre = false;
    }
  }
  /* No right branch: take the centre line if visible, otherwise track the
   * remaining side. These are forward arcs, distinct from the junction pivot. */
  f3.command = (tape & CENTRE) ? F3_STRAIGHT : (tape & RIGHT) ? F3_STEER_RIGHT : F3_STEER_LEFT;
}
