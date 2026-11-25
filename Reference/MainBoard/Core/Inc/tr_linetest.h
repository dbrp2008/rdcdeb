#ifndef TR_LINETEST_H
#define TR_LINETEST_H

/**
 * TR line-following test (pure logic; the glue is tr_auto_app.c, which shares the AUTO page).
 *
 * Follows a line with the two IRs that straddle it and turns 90 degrees at each junction, WITHOUT any field distances.
 * A drift and a junction look the same to two straddling IRs (one of them sees black), so it tells them apart by probing:
 *
 *   LEAVE   after the countdown: drive straight (heading held), ignoring the line, until it has crossed something dark (the
 *           start box's fill and/or its black outline) and BOTH IRs then read white together for TRL_CLEAR_MS - so the box,
 *           its outline (even crossed at an angle), or starting half over it never looks like a line or junction.
 *           (If one IR stays black for TRL_LEAVE_LINE_MM it started off-centre with the line under that IR: follow it.)
 *   FOLLOW  drive forward slowly (TRL_SPEED), heading held, both IRs on white
 *   CHECK   one IR sees black: keep creeping forward TRL_CHECK_MM first. The other IR going black too (for TRL_BOTH_MM) = a CROSSING met at a
 *           slight angle (the start box outline, any crossing on the field): drive on. The first IR clearing = it has passed
 *           over (the back edge of a crossing): drive on. Still only that IR black -> PROBE.
 *   CROSS   both IRs black (a crossing, or a branch on one side while the other IR touches the main line): drive on and
 *           watch which IR clears first. The other one clearing within TRL_CROSS_MM = a crossing: carry on. The other one
 *           staying black = it is on the main line, so the IR that cleared was on a BRANCH: a JUNCTION on that side.
 *   PROBE   one IR still sees black: stop moving forward and slide sideways toward that IR
 *             - the IR sees white again within TRL_PROBE_MS  -> it was the line drifting: re-centre the line, count a DRIFT,
 *               back to FOLLOW. The free gap between the line and the IRs is measured, not assumed: the first time, it keeps
 *               sliding until the OTHER IR just touches the line (that distance = the gap) and slides back half of it; after
 *               that it slides half the remembered gap. (A fixed re-centre slide bounced the TR between the IRs: 40 corrections.)
 *             - the other IR ALSO sees black -> a branch is under one of them: CROSS decides which
 *             - the IR is still black after TRL_PROBE_MS     -> a branch going off to that side: a JUNCTION (backup)
 *   ADVANCE drive TRA_ADVANCE_MM (IR-to-turning-centre, a robot dimension) so the wheels are over the junction
 *   TURN    90 degrees toward the branch (left IR -> left), then FOLLOW the new line
 *   both IRs black = a crossing: drive straight on.   Both IRs white = the line is between them: drive on.
 *   Two straddling IRs cannot see where a line ENDS, so the test stops when the front ultrasonic sees something within
 *   TRL_STOP_MM (a wall / object at the end of the line), on B2, on a joystick movement, or after TRL_TIME_LIMIT_MS.
 *
 * The screen shows the number of drift corrections and junctions. Stops after TRL_TIME_LIMIT_MS, on B2, or a stick.
 */

#include "tr_auto.h"

#ifndef TRL_SPEED
#define TRL_SPEED 0.2f /* forward ratio while following: slow, so it can stop on a 20 mm line */
#endif
#ifndef TRL_PROBE_SPEED
#define TRL_PROBE_SPEED 0.15f /* sideways ratio while probing (slow: less overshoot when it stops) */
#endif
#ifndef TRL_CHECK_MM
#define TRL_CHECK_MM 8 /* CHECK: creep this far before deciding: covers a crossing met up to ~7 degrees off-square (IRs 50 mm
                          apart), and (with ~1 mm braking at TRL_CHECK_SPEED) stays inside even a 20 mm branch tape */
#endif
#ifndef TRL_CHECK_SPEED
#define TRL_CHECK_SPEED 0.1f /* CHECK: creep at half speed so it stops within ~1 mm */
#endif
#ifndef TRL_BOTH_MM
#define TRL_BOTH_MM 2 /* CHECK: both IRs black continuously for this much creep = a crossing (one tick is often edge flicker) */
#endif
#ifndef TRL_CROSS_MM
#define TRL_CROSS_MM 15 /* CROSS: the second IR must clear within this much after the first (crossing up to ~10 degrees off-square) */
#endif
#ifndef TRL_CROSS_CLEAR_MM
#define TRL_CROSS_CLEAR_MM 8 /* CROSS: an IR counts as clear after this much solid white (~8 control ticks at TRL_SPEED: flicker-proof) */
#endif
#ifndef TRL_CROSS_MAX_MM
#define TRL_CROSS_MAX_MM 80 /* CROSS: both black for longer than this: give up and follow on */
#endif
#ifndef TRL_PROBE_MS
#define TRL_PROBE_MS 400U /* still black after sliding this long toward it = a junction */
#endif
/* Braking: after the sideways command stops, the TR still slides v^2 / 2a (the drive's 600 rpm/s limit on 152 mm wheels).
 * Slides end that much early, and once the gap is known the slide is slowed so the braking distance stays under a quarter
 * of the gap (it cannot stop in the middle of a gap smaller than twice its braking distance). Never slower than TRL_PROBE_MIN. */
#ifndef TRL_PROBE_MIN
#define TRL_PROBE_MIN 0.06f
#endif
#ifndef TRL_CLEAR_MS
#define TRL_CLEAR_MS 100U /* LEAVE: both IRs on white this long = clear of the start box, start following */
#endif
#ifndef TRL_LEAVE_LINE_MM
#define TRL_LEAVE_LINE_MM 80 /* LEAVE: the SAME single IR black for this much forward travel = it started off-centre with the line under
                                that IR (the box outline is ~20 mm wide, so crossing it, even at an angle, is much shorter): start following */
#endif
#ifndef TRL_LEAVE_MAX_MM
#define TRL_LEAVE_MAX_MM 300 /* LEAVE: never saw anything dark in this far (started with the IRs past the outline): start following */
#endif
#ifndef TRL_LEAVE_MAX_MS
#define TRL_LEAVE_MAX_MS 4000U /* LEAVE: start following anyway after this long */
#endif
#ifndef TRL_FLICKER_MM
#define TRL_FLICKER_MM 3 /* the IR reads black again within this much of where it read white: a flicker at the tape edge, keep probing */
#endif
#ifndef TRL_HDG_TRIM_DEG
#define TRL_HDG_TRIM_DEG 0.5f /* each drift correction turns the held heading this much toward the line (the line corrects the heading) */
#endif
#ifndef TRL_HDG_TRIM_MAX_DEG
#define TRL_HDG_TRIM_MAX_DEG 6.0f /* ...but never more than this from the heading the leg started with */
#endif
#ifndef TRL_GAP_MIN_MM
#define TRL_GAP_MIN_MM 4 /* a measured gap smaller than this is another line touching the IR, not the real gap: ignored */
#endif
#ifndef TRL_GAP_MAX_MM
#define TRL_GAP_MAX_MM 40 /* never slide further than this while measuring the gap */
#endif
#ifndef TRL_STOP_MM
#define TRL_STOP_MM 150 /* front ultrasonic this close (2 readings in a row) = stop: wall or object ahead */
#endif
#ifndef TRL_TIME_LIMIT_MS
#define TRL_TIME_LIMIT_MS 30000U
#endif

typedef enum { TRL_IDLE = 0, TRL_COUNTDOWN, TRL_LEAVE, TRL_FOLLOW, TRL_CHECK, TRL_CROSS, TRL_PROBE, TRL_ADV, TRL_TURN, TRL_END, TRL_TIMEUP, TRL_FAULT, TRL_FOUND } TRL_State; /* TRL_END = stopped by the ultrasonic; TRL_FOUND = search mode found a junction */

typedef struct {
  TRL_State st;
  float speed;                 /* forward ratio while following (TRL_SPEED for the test) */
  bool search;                 /* search mode (used by the 20 s routine): no ADV/TURN, no US stop, no time limit; a junction -> TRL_FOUND */
  float junc_mm;               /* search mode: forward position (in->dist_mm) where the IRs were over the branch */
  uint32_t t0, since, countdown_end;
  float base_mm, hdg_target;
  float hdg_ref;               /* heading at the start of the leg (start / after a turn); the trim stays within +-TRL_HDG_TRIM_MAX_DEG of it */
  int8_t side;                 /* -1 left, +1 right: where the probe / junction is */
  uint32_t clear_since;        /* LEAVE: both IRs white since (0 = not now) */
  bool leave_dark;             /* LEAVE: an IR has seen the dark box / its outline */
  float seen_mm, both_mm;      /* CHECK: creep distance where the IR last read black / where both IRs went black (-1 = not now) */
  float check_mm;              /* CHECK / PROBE: forward position where the IR first saw black */
  float cross_mm;              /* CROSS: where both went black */
  float lb_left, lb_right;     /* CROSS: where each IR last read black */
  float single_mm;             /* LEAVE: forward position where the current single-IR-black pattern began (side = which IR) */
  bool turn_requested, turn_seen_run;
  uint8_t phase;               /* PROBE: 0 slide toward the black IR, 1 re-centre (or measure the gap), 2 slide back half the gap */
  float slide_base;            /* sideways position where the current slide began */
  float gap_mm;                /* free sideways gap between touching one IR and touching the other; 0 = not measured yet */
  uint8_t us_close;            /* fresh ultrasonic readings in a row within TRL_STOP_MM */
  uint16_t drifts, junctions;  /* what the test saw */
} TRLine;

void TRLine_Init(TRLine *a);
void TRLine_Start(TRLine *a, uint32_t now);
void TRLine_Abort(TRLine *a);
/* Search mode for the autonomous routine: follow the line from here (heading held at heading_deg, forward ratio speed) with the
 * same drift / crossing / junction logic, and stop in TRL_FOUND at the first junction: a->side = -1 left / +1 right,
 * a->junc_mm = where the IRs were over the branch. Call TRLine_Step every tick until then. */
void TRLine_Search(TRLine *a, const TRAuto_In *in, float heading_deg, float speed);
bool TRLine_IsRunning(const TRLine *a);
void TRLine_Step(TRLine *a, const TRAuto_In *in, TRAuto_Out *out);
const char *TRLine_StateText(TRL_State s);

#endif
