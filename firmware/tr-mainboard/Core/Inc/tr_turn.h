#ifndef TR_TURN_H
#define TR_TURN_H
#include <stdbool.h>
#include <math.h>
#include <stdint.h>

#define TR_TURN_TIMEOUT_MS 8000U
#define TR_TURN_NO_PROGRESS_MS 600U
#define TR_TURN_TOL_DEG 0.7f
#define TR_TURN_DECEL_DEG 40.0f
#define TR_TURN_MIN_FRAC 0.15f
#define TR_TURN_MAX_FRAC 0.50f
#define TR_TURN_WRONG_WAY_DEG 5.0f

typedef enum {
  TR_TURN_IDLE, TR_TURN_RUN, TR_TURN_DONE, TR_TURN_ABORT,
  TR_TURN_NOFB, TR_TURN_REJECT, TR_TURN_WRONG_WAY
} TRTurnState;
typedef struct {
  TRTurnState state;
  float target_deg, travelled_deg, last_heading_deg;
  uint32_t start_tick;
} TRTurn;

static inline void tr_turn_start(TRTurn *turn, float target_deg, float heading,
                                 uint32_t now, bool allowed)
{
  if (!allowed) { turn->state = TR_TURN_REJECT; return; }
  turn->state = TR_TURN_RUN;
  turn->target_deg = target_deg;
  turn->travelled_deg = 0.0f;
  turn->last_heading_deg = heading;
  turn->start_tick = now;
}

/* Returns the desired sign and fraction of rotation. Wrap-safe accumulated
 * heading permits 180 degree turns without discontinuities at +/-360. */
static inline float tr_turn_step(TRTurn *turn, uint32_t now, float heading,
                                 bool safe_to_continue)
{
  if (turn->state != TR_TURN_RUN) return 0.0f;
  if (!safe_to_continue) { turn->state = TR_TURN_ABORT; return 0.0f; }
  turn->travelled_deg += remainderf(heading - turn->last_heading_deg, 360.0f);
  turn->last_heading_deg = heading;
  const float direction = turn->target_deg > 0.0f ? 1.0f : -1.0f;
  const float remaining = turn->target_deg - turn->travelled_deg;
  if (fabsf(remaining) <= TR_TURN_TOL_DEG || remaining * direction <= 0.0f) {
    turn->state = TR_TURN_DONE;
    return 0.0f;
  }
  if (turn->travelled_deg * direction < -TR_TURN_WRONG_WAY_DEG) {
    turn->state = TR_TURN_WRONG_WAY;
    return 0.0f;
  }
  if ((uint32_t)(now - turn->start_tick) > TR_TURN_TIMEOUT_MS) {
    turn->state = TR_TURN_ABORT;
    return 0.0f;
  }
  if ((uint32_t)(now - turn->start_tick) > TR_TURN_NO_PROGRESS_MS &&
      fabsf(turn->travelled_deg) < 1.0f) {
    turn->state = TR_TURN_NOFB;
    return 0.0f;
  }
  float fraction = fabsf(remaining) / TR_TURN_DECEL_DEG;
  if (fraction > TR_TURN_MAX_FRAC) fraction = TR_TURN_MAX_FRAC;
  if (fraction < TR_TURN_MIN_FRAC) fraction = TR_TURN_MIN_FRAC;
  return direction * fraction;
}
#endif
