#ifndef FUNCTION1_H
#define FUNCTION1_H
#include <stdbool.h>
#include <stdint.h>
#include "line_drive.h"
#include "branch_latch.h"

/* F1 is a single-instance, foreground controller. No HAL, pin, motor or TFT
 * access. Times are uint32_t milliseconds; IR masks use bits 0/1/2 = L/C/R. */
/* F1_LEGACY 1 = the original behaviour exactly (their tests pin it). 0 (default) = the fixes:
 *   - steering: 110 / 011 (line between the centre and a side sensor) steer GENTLY; 100 / 001 steer hard (as before).
 *     Line lost (000) right after it slipped out to one side: keep turning back that way (slower) for F1_LOST_TURN_MS;
 *     lost while it was centred (a gap in the tape): straight on, as before.
 *   - the 101 "hold left" latch releases on ANY scan with the centre black (010 / 110 / 011 / 111) or after
 *     F1_SPLIT_MAX_MS, instead of only on an exact 010 (it could keep turning left with the line on the RIGHT).
 *   - home: after the target hold, F1_RETRACE: the board replays the recorded outbound wheel commands backwards on the
 *     encoders (retrace.h) instead of reverse / spin / find the line / follow it back / wait for 111 + white + US < 55.
 *   - no "armed three-black finish" on the next run (with the retrace home it stopped the next run at its first crossing). */
#ifndef F1_LEGACY
#define F1_LEGACY 0
#endif
/* (not legacy) HOME for F1: TURN 180 and follow the line home forward (the IRs lead: stable), instead of the encoder
 * retrace (F2 keeps the retrace: its branch turn would have to be undone). After the target hold: reverse F1_BACK_MM
 * (both wheels held to equal pulses by the board), pivot CCW - line tape before F1_PIVOT_MIN_DEG is ignored (the line it
 * left), the centre IR on tape inside [MIN, MAX] ends the turn AT the line (no extra turning), no tape by F1_PIVOT_MAX_DEG
 * = stop "TURN NO LINE" - then follow the line home. The finish is the original one (the start-zone outline 111 seen,
 * then white, and US1 < DRIVE_US1_FINISH_MM to the zone's wall), but the outline only counts within F1_HOME_EARLY_MM of
 * the distance it drove out (crossings earlier on the route are ignored), and it stops at the latest F1_HOME_LATE_MM past
 * that distance. The board feeds the encoders with F1_Odometry. */
#ifndef F1_RETURN_TURN
#define F1_RETURN_TURN 1
#endif
#ifndef F1_BACK_MM
#define F1_BACK_MM 100.0f
#endif
#ifndef F1_PIVOT_MIN_DEG
#define F1_PIVOT_MIN_DEG 140.0f
#endif
#ifndef F1_PIVOT_MAX_DEG
#define F1_PIVOT_MAX_DEG 220.0f
#endif
#ifndef F1_HOME_EARLY_MM
#define F1_HOME_EARLY_MM 500.0f
#endif
#ifndef F1_HOME_LATE_MM
#define F1_HOME_LATE_MM 300.0f
#endif
#ifndef F2_RETURN_TURN
#define F2_RETURN_TURN 1 /* F2 home: turn 180 like F1, follow the vertical line back, turn LEFT at the T, follow home (0 = retrace) */
#endif
#ifndef F2_T_EARLY_MM
#define F2_T_EARLY_MM 300.0f /* F2 home: the T counts from this much before the expected distance (key -> T, measured going out) */
#endif
#ifndef F2_T_LATE_MM
#define F2_T_LATE_MM 250.0f /* ...and if not found this far past it: stop ("HOME NO JUNCTION") */
#endif
#ifndef F2_T_ADVANCE_MM
/* F2 home at the T: drive this far straight past the bottom line, THEN pivot left in place (F1's turn). Turning as soon as
 * the IRs reach the line cannot work: the IRs' circle only touches the line there and swings away from it. After
 * advancing s the IRs' circle crosses the line for ANY IR-to-axle distance d between s/2 and far more than the AR has
 * (pickup at acos(1 - s/d): d 100 mm -> 78 deg, 130 -> 67, 160 -> 60, 200 -> 53) */
#define F2_T_ADVANCE_MM 80.0f
#endif
#ifndef F2_T_TURN_MIN_DEG
#define F2_T_TURN_MIN_DEG 35.0f /* ...tape before this is ignored */
#endif
#ifndef F2_T_TURN_MAX_DEG
#define F2_T_TURN_MAX_DEG 130.0f /* ...and with none by this it stops ("HOME T NO TAPE") */
#endif
/* Time backups for every encoder-gated move (a dead or miscounting encoder must stop the run, not run it away). Sized for
 * a slow AR: 80 mm in 3 s = 27 mm/s (the sim drives ~225 mm/s); a 130 deg pivot in 6 s = 22 deg/s (sim ~60). */
#ifndef F2_T_ADVANCE_MAX_MS
#define F2_T_ADVANCE_MAX_MS 3000u /* F2 home: past the bottom T (F2_T_ADVANCE_MM) in this long, or stop "HOME T TIMEOUT" */
#endif
#ifndef F2_T_TURN_MAX_MS
#define F2_T_TURN_MAX_MS 6000u /* F2 home: the left pivot onto the bottom line, or stop "HOME T TIMEOUT" */
#endif
#ifndef F1_RETURN_EXTRA_MS
#define F1_RETURN_EXTRA_MS 5000u /* home by turning: the return may take 2 x the outbound time + this, or stop "HOME BY TIME" */
#endif
#ifndef F1_STALL_MS
#define F1_STALL_MS 500u /* home: backup if US1 misses the zone's wall - near home, wheels moved < F1_STALL_MM in this long */
#endif
#ifndef F1_STALL_MM
#define F1_STALL_MM 5.0f
#endif
#ifndef F1_OUTLINE_ALL5
#define F1_OUTLINE_ALL5 1 /* home: the start-zone outline = all 3 analog AND an outer digital IR seen black (both may not) */
#endif
#ifndef F1_OUTLINE_SPAN_MM
#define F1_OUTLINE_SPAN_MM 60.0f /* home: every IR black within this much travel = the outline (they need not trip together) */
#endif
#ifndef F1_HEADING_AVG_MM
#define F1_HEADING_AVG_MM 250.0f /* following home: the line's direction = the heading averaged over this much travel */
#endif
#ifndef F1_ZONE_SHIFT_MM
#define F1_ZONE_SHIFT_MM 0.0f /* OFF: it rotated left in the zone into the barrier beside it (field). Was 23: shift LEFT of the line, then straight on. The AR ended square, 1-2.5 cm
                                  * over the zone's right edge with < 4 cm spare on the left: ~2.5 cm centred it on the floor,
                                  * +0.5 cm more for margin = ~3 cm. 23 here shifts ~30 mm in the sim (it overshoots ~7 mm while
                                  * straightening; 18 gave ~25). 0 = off. + = further left */
#endif
#ifndef F1_ZONE_SHIFT_DEG
#define F1_ZONE_SHIFT_DEG 8.0f /* ...by heading this far left of the line's direction until shifted */
#endif
#ifndef F1_STRAIGHT_K
#define F1_STRAIGHT_K 1.5f /* straight on (outline / into the zone): % duty difference per degree off the line's direction */
#endif
#ifndef F1_TRACK_MM
#define F1_TRACK_MM 233.0f /* between the wheels, tuned on this AR (246 centre to centre): the pivot angle = both wheels' arcs / this */
#endif
/* (not legacy) F3: the top-right key, by the right wall (SIM ONLY - test_f3_home.c on the photo's map). Out: the bottom line,
 * KEEP RIGHT from F3_KEEP_FROM_MM onto the U (sharper steering: the U's radius ~200 mm is tighter than the line follower
 * turns), round it (F3_U_DONE_DEG clockwise), east along its top arm still keeping right (the diagonal joins on the left),
 * at the V (centre + right IR off the tape): advance F3_CORNER_ADVANCE_MM, pivot LEFT onto the short up-right leg (window
 * F3_V_*), follow it keeping left, at its top bend (centre + left IR off): advance, pivot RIGHT (F3_B_*), east to the key:
 * US1 only counts on this last leg (the box by the bend). Home: back off F3_BACK_MM, pivot CCW straight onto the up-right
 * leg (F3_PIVOT_*: the bend is too close behind to find the east leg at 180), down it keeping left, at the V advance and
 * pivot RIGHT onto the U's top arm (F3_HV_*: the arm ~45 deg, the diagonal ~70 - the first tape wins), keep left round the
 * U onto the bottom line, then home like F1 (outline, zone shift, wall). Every leg has a distance limit and every
 * advance / pivot a time limit: a missed corner stops the run with a message, it does not wander off. */
#ifndef F3_KEEP_FROM_MM
#define F3_KEEP_FROM_MM 300.0f   /* keep right (onto the U) from this far out; the U leaves the bottom line ~500 mm out */
#endif
#ifndef F3_U_DONE_DEG
#define F3_U_DONE_DEG 170.0f     /* round the U: turned this far clockwise = on its top arm */
#endif
#ifndef F3_U_MAX_MM
#define F3_U_MAX_MM 1400.0f      /* keep right this far without getting round the U: stop "F3 NO U" */
#endif
#ifndef F3_ARM_MM
#define F3_ARM_MM 200.0f         /* top arm: the V counts once the outer-left IR saw the diagonal, or after this */
#endif
#ifndef F3_LEG_MAX_MM
#define F3_LEG_MAX_MM 700.0f     /* a straight leg this long without its corner: stop ("F3 NO V", "F3 NO BEND", "HOME NO V") */
#endif
#ifndef F3_CORNER_CONFIRM_MM
#define F3_CORNER_CONFIRM_MM 65.0f /* a corner = its IRs white for this much travel (a wobble off the line is shorter; the advance
                                    * counts from the first white. Sim: 55-80 pass, 30-50 let a wobble fake the home V) */
#endif
#ifndef F3_CORNER_ADVANCE_MM
#define F3_CORNER_ADVANCE_MM 110.0f /* at a corner: this far straight on before the pivot - about the IRs' distance ahead of the
                                    * axle, so the AR pivots OVER the corner and the centre IR meets the next leg nearly along it
                                    * (80 as F2 left it ~17 deg short: the follower then lost the leg in an IR gap) */
#endif
#ifndef F3_V_SIDE_ADVANCE_MM
#define F3_V_SIDE_ADVANCE_MM 252.0f /* out at the V: the outer-left IR (60 mm out) first touches the diagonal (joining at ~27
                                     * deg, 20 mm tape) ~142 mm before the V: follow on to the V, then straight on the rest of
                                     * this (= F3_CORNER_ADVANCE_MM past the V). 10 mm off the line's centre moves it ~20 mm */
#endif
#ifndef F3_BEND_SIDE_MM
#define F3_BEND_SIDE_MM 20.0f      /* out at the bend: the outer-right IR first touches the east leg ~this far past it */
#endif
#ifndef F3_HV_SIDE_MM
#define F3_HV_SIDE_MM 10.0f        /* home at the V: the outer-right IR first touches the diagonal ~this far before it */
#endif
#ifndef F3_V_MIN_DEG
#define F3_V_MIN_DEG 10.0f       /* out, the V's left pivot: the up-right leg at ~55-65 deg (the diagonal ~145); only after white */
#endif
#ifndef F3_V_MAX_DEG
#define F3_V_MAX_DEG 105.0f
#endif
#ifndef F3_UP_MIN_MM
#define F3_UP_MIN_MM 150.0f      /* the up-right leg (~310 mm): its bend counts after this */
#endif
#ifndef F3_B_MIN_DEG
#define F3_B_MIN_DEG 10.0f       /* the bend's right pivot: the east leg at ~55-65 deg (less if it saw the bend early) */
#endif
#ifndef F3_SIDE_CONFIRM_MM
#define F3_SIDE_CONFIRM_MM 6.0f  /* the bend: the outer-right IR high this long (it crosses the 20 mm east leg over ~23 mm) */
#endif
#ifndef F3_ON_EAST_DEG
#define F3_ON_EAST_DEG 35.0f     /* the bend seen with the AR already this far right of the up-right leg (east leg ~61): the
                                  * follower took the bend itself - no pivot, on along the east leg */
#endif
#ifndef F3_BBACK_MIN_DEG
#define F3_BBACK_MIN_DEG 20.0f   /* the bend's pivot found no tape: turn back left, the up-right leg counts after this... */
#endif
#ifndef F3_BBACK_MAX_DEG
#define F3_BBACK_MAX_DEG 170.0f  /* ...none by this: stop "F3 BEND NO TAPE" (one retry per run) */
#endif
#ifndef F3_B_MAX_DEG
#define F3_B_MAX_DEG 105.0f
#endif
#ifndef F3_BACK_MM
#define F3_BACK_MM 60.0f         /* home: back off the key this far, then pivot (sim: 50 / 65 fail a few runs: sensitive) */
#endif
#ifndef F3_PIVOT_MIN_DEG
#define F3_PIVOT_MIN_DEG 150.0f  /* home pivot (CCW): the up-right leg at ~220-260 deg */
#endif
#ifndef F3_PIVOT_MAX_DEG
#define F3_PIVOT_MAX_DEG 290.0f
#endif
#ifndef F3_DOWN_MIN_MM
#define F3_DOWN_MIN_MM 150.0f    /* home, down the up-right leg (~310 mm): the V counts after this (sim: 120 let a wobble early
                                  * on the leg pass for the V; 165 also passes) */
#endif
#ifndef F3_HV_MIN_DEG
#define F3_HV_MIN_DEG 10.0f      /* home, the V's right pivot: the U's top arm at ~55-65 deg (the first tape after white),
                                  * the diagonal ~80-90 */
#endif
#ifndef F3_HV_MAX_DEG
#define F3_HV_MAX_DEG 78.0f 
#endif
#ifndef F3_ARM_AVG_FROM_MM
#define F3_ARM_AVG_FROM_MM 80.0f  /* home, along the U's top arm (~330 mm past the V): its heading averaged from here... */
#endif
#ifndef F3_ARM_AVG_TO_MM
#define F3_ARM_AVG_TO_MM 230.0f   /* ...to here; + 180 = the bottom line's direction into the zone */
#endif
#ifndef F3_HU_DONE_DEG
#define F3_HU_DONE_DEG 150.0f    /* home: turned this far left since the V = round the U, on the bottom line */
#endif
#ifndef F3_SHARP_INSIDE
#define F3_SHARP_INSIDE 3.0f     /* round the U: the wheel duties (before F12_DUTY) with only the outer IR on the tape */
#endif
#ifndef F3_SHARP_OUTSIDE
#define F3_SHARP_OUTSIDE 24.0f
#endif
/* F4 (not legacy): the bottom-left key, below the 8. SIM ONLY - every number below is from the field map (105.png, ~3.6
 * px/cm) and the AR's measured size, NOT field-tested. Out along the bottom line like F1 (its heading averaged = H0, west)
 * until US1 <= F4_STONE_STOP_MM at the F1 stone, stop, then encoder legs held on the heading (no line):
 * right 90, north F4_NORTH_MM, left 90, west past the stone to W, right F4_GAP_DEG, F4_GAP_MM through the gap between the
 * T's bar and the barrier's top (346 mm wide: ~38 mm each side of the 270 mm AR), left F4_GAP_DEG, west across the 1 and
 * the 8 (L+C+R all black = a crossing; the 8's west edge = the first one from F4_S4_EDGE_FROM_MM), advance
 * F4_EDGE_ADV_MM (axle over the west edge = in line with the key), left 90, south to the key (US1 only counts here),
 * hold, back off. Home: reverse north to where it turned, then the same legs backwards, each straight as long as it was
 * driven out (a wheel-size error cancels), at the end south until the bottom line (all three black), advance, pivot left
 * onto it, home like F1 (outline, zone, wall). */
#ifndef F4_STONE_STOP_MM
#define F4_STONE_STOP_MM 250.0f   /* out on the bottom line: US1 this close to the F1 stone = stop (the pivots sweep ~190 mm) */
#endif
#ifndef F4_LINE_OFF_DEG
#define F4_LINE_OFF_DEG 25.0f /* out to the stone: this far off the bottom line's heading = on a curve: stop "F4 OFF LINE" */
#endif
#ifndef F4_STONE_MIN_MM
#define F4_STONE_MIN_MM 1000.0f   /* ...only counts this far out (the stop is ~1430 mm out) */
#endif
#ifndef F4_STONE_MAX_MM
#define F4_STONE_MAX_MM 2100.0f   /* no stone by this far out: stop "F4 NO STONE" */
#endif
#ifndef F4_US_AHEAD_MM
#define F4_US_AHEAD_MM 117.0f     /* US1 ahead of the wheel axle */
#endif
#ifndef F4_CAL_AT_MM
#define F4_CAL_AT_MM 350.0f       /* the turn calibration: this far out on the bottom line (before the U; the start zone's
                                   * outline and the U both out of the IRs' circle), a full turn right on the spot */
#endif
#ifndef F4_CAL_PAST_OUTLINE_MM
#define F4_CAL_PAST_OUTLINE_MM 250.0f /* ...and at least this far past where the IRs crossed the start zone's outline (all
                                       * three black): the IRs' circle (~115 mm round the axle) stays off it */
#endif
#ifndef F4_OUTLINE_MAX_MM
#define F4_OUTLINE_MAX_MM 300.0f  /* ...the outline only counts this near the start */
#endif
#ifndef F4_CAL_BEHIND_MIN_DEG
#define F4_CAL_BEHIND_MIN_DEG 120.0f /* ...the centre IR on tape from this far round = the line behind */
#endif
#ifndef F4_CAL_FRONT_MIN_DEG
#define F4_CAL_FRONT_MIN_DEG 300.0f  /* ...from this far round = the line in front again (the turn ends there) */
#endif
#ifndef F4_CAL_TAPE_MAX_DEG
#define F4_CAL_TAPE_MAX_DEG 40.0f /* ...the line behind wider than this = not the line: "F4 CAL BAD" */
#endif
#ifndef F4_CAL_K_MIN
#define F4_CAL_K_MIN 0.6f        /* ...true / encoder degrees outside [MIN, MAX] = "F4 CAL BAD" */
#endif
#ifndef F4_CAL_K_MAX
#define F4_CAL_K_MAX 1.4f
#endif
#ifndef F4_CAL_MAX_DEG
#define F4_CAL_MAX_DEG 700.0f     /* ...turned this far (encoder) without all four edges: "F4 CAL NO LINE" */
#endif
#ifndef F4_CAL_MAX_MS
#define F4_CAL_MAX_MS 20000u      /* ...or this long: "F4 TURN TIMEOUT" */
#endif
#ifndef F4_CAL_DEADBAND
#define F4_CAL_DEADBAND 0.015f    /* ...a scale within 1 +- this = keep 1 (the measurement scatters ~1 %, the turns take 1.5 %) */
#endif
#ifndef F4_H0_AFTER_CAL_MM
#define F4_H0_AFTER_CAL_MM 100.0f /* the bottom line's heading (H0) is averaged from this far past the calibration */
#endif
#ifndef F4_SETTLE_MS
#define F4_SETTLE_MS 300u         /* stopped at the stone this long (the coast is measured and taken off the west leg) */
#endif
#ifndef F4_NORTH_MM
#define F4_NORTH_MM 266.5f        /* north off the bottom line: the west leg runs at y -266.5 (clear of the stone and circle) */
#endif
#ifndef F4_W_PAST_STONE_MM
#define F4_W_PAST_STONE_MM 316.6f /* the west leg ends this far past the stone's face (W, lined up with the gap) */
#endif
#ifndef F4_W_WALL_MM
#define F4_W_WALL_MM 169.0f       /* the west leg ends with US1 this far from the barrier's east face (x 2103: the gap's
                                   * south-west wall) = at W */
#endif
#ifndef F4_W_WINDOW_MM
#define F4_W_WINDOW_MM 120.0f     /* ...US1 read from this much further out (the barrier's face is the only thing ahead) */
#endif
#ifndef F4_W_FROM_MM
#define F4_W_FROM_MM 100.0f       /* ...from this far into the west leg - WHEREVER the encoders put W (a roughly placed
                                   * stone puts the barrier earlier / later than the stone says) */
#endif
#ifndef F4_W_EXTRA_MM
#define F4_W_EXTRA_MM 300.0f      /* US1 never sees the face: the west leg ends this far past the encoders' W */
#endif
#ifndef F4_US_BLOCK_MM
#define F4_US_BLOCK_MM 70.0f      /* any F4 straight (not the key leg): US1 this near = something ahead: stop "F4 US1 BLOCKED" */
#endif
#ifndef F4_GAP_DEG
#define F4_GAP_DEG 64.1f          /* through the gap: this far right of west (square to the gap) */
#endif
#ifndef F4_GAP_MM
#define F4_GAP_MM 415.0f          /* W -> through the gap (its middle at 300) -> G2 (y -640), 207 mm from both wall ends (the
                                   * pivot sweeps 190) */
#endif
#ifndef F4_EDGE_ADV_MM
#define F4_EDGE_ADV_MM 125.0f     /* a crossing seen at the tape's near side: on this far = the axle over its middle (IRs
                                   * ~115 mm ahead + half the tape) */
#endif
#ifndef F4_S4_EDGE_FROM_MM
#define F4_S4_EDGE_FROM_MM 385.0f /* west from G2 at y -640: the 1 at ~126, the 8's east edge ~281, its west edge ~489 */
#endif
#ifndef F4_S4_MAX_MM
#define F4_S4_MAX_MM 640.0f       /* no west edge by this: stop "F4 NO 8" */
#endif
#ifndef F4_KEY_MAX_MM
#define F4_KEY_MAX_MM 800.0f      /* south to the key (~530 mm): none by this, stop "F4 NO KEY" */
#endif
#ifndef F4_BACK_MM
#define F4_BACK_MM 100.0f         /* home: back off the key this far, then on back north (no 180 by the barrier) */
#endif
#ifndef F4_H4_FROM_MM
#define F4_H4_FROM_MM 70.0f       /* home south to the bottom line (~140 mm): all three black from here (the C crosses
                                   * earlier, at a slant: never all three at once) */
#endif
#ifndef F4_H4_MAX_MM
#define F4_H4_MAX_MM 300.0f       /* none by this: stop "F4 HOME NO LINE" */
#endif
#ifndef F4_P5_MIN_DEG
#define F4_P5_MIN_DEG 45.0f       /* home, the left pivot onto the bottom line: the centre IR on tape after white, in here */
#endif
#ifndef F4_P5_MAX_DEG
#define F4_P5_MAX_DEG 135.0f
#endif
#ifndef F4_PIVOT_LEAD_DEG
#define F4_PIVOT_LEAD_DEG 1.5f    /* a pivot ends this short of its angle (it coasts; the next straight holds the heading) */
#endif
#ifndef F4_PAUSE_MS
#define F4_PAUSE_MS 250u          /* stopped this long before every F4 leg (the encoders' direction comes from the command) */
#endif
#ifndef F4_RISE_SPAN_MM
#define F4_RISE_SPAN_MM 15.0f     /* a crossing: two IRs went black within this much travel of each other */
#endif
#ifndef F4_KEY_STALL_MIN_MM
#define F4_KEY_STALL_MIN_MM 300.0f /* south to the key: stalled after this far = against the key (US1 missed it): hold */
#endif
#ifndef F4_CROSS_GAP_MM
#define F4_CROSS_GAP_MM 40.0f     /* two crossings at least this far apart */
#endif
#ifndef F4_STALL_MS
#define F4_STALL_MS 700u          /* an F4 leg with the encoders still this long (dead encoder, against something): stop */
#endif
#ifndef F1_LOST_TURN_MS
#define F1_LOST_TURN_MS 800u
#endif
#ifndef F1_SPLIT_MAX_MS
#define F1_SPLIT_MAX_MS 500u
#endif

typedef enum {
  F1_IDLE, F1_LAUNCH, F1_OUTBOUND, F1_TARGET_APPROACH, F1_TARGET_HOLD, F1_TARGET_REVERSE,
  F1_PIVOT, F1_ALIGN, F1_RETURN, F1_FINISH_REVERSE,
  F1_RETRACE /* (not legacy) the board drives home on the encoders (retrace.h); F1 only reports the phase */
} F1_Phase;
typedef enum { F1_KEEP_STOPPED, F1_MOVE, F1_STOP } F1_Request;
/* F1_MOVE already contains the final physical-wheel PWM percentages. */
typedef struct { F1_Request request; LineWheels wheels; const char *status; } F1_Command;

void F1_Init(void); /* board reset, including the next-run finish latch */
/* Only after start interlocks; either_side_high is live L2 OR R2 at start.
 * In F1 forward follow (filtered L/C/R), 101 holds left through all other
 * patterns until exactly 010 releases it to straight on that scan. Otherwise
 * 100/001 steer left/right per scan, all others go straight. Steering never
 * overrides a stop, finish, hold, reverse or pivot. F2 remains per-scan. */
void F1_Start(uint32_t now, unsigned filtered, bool either_side_high);
/* Feed each fresh digital observation once; analog scans never advance BR. */
void F1_SideSample(BranchSample side);
/* For F2's shared timing/turnaround only: retain its original stateless following. */
void F1_StartPerScan(uint32_t now, unsigned filtered);
/* (not legacy) F3: the top-right key on F1's run (see F3_* above); the board feeds it exactly as F1 */
void F1_StartF3(uint32_t now, unsigned filtered, bool either_side_high);
/* (not legacy) F4: the bottom-left key below the 8 (see F4_* above, sim only); the board feeds it exactly as F1 */
void F1_StartF4(uint32_t now, unsigned filtered, bool either_side_high);
void F1_Stop(void); /* external stop: retain the count and any next-run latch */

/* Feed each new full-window scan exactly once, before advancing the clock.
 * Only a fresh filtered 111 after CCW alignment arms this return's line end. */
void F1_Scan(uint32_t started, uint32_t completed, unsigned raw, unsigned filtered);
/* Report each new valid ping at result time. True = the 350 ms forward approach
 * just latched, NOT a zero request. Later pings do not extend/cancel it.
 * Apply CommandAt every control tick, not just when pings arrive. */
bool F1_TargetPing(uint32_t observed_at, uint32_t valid_us1_mm);
/* After polls: advance timed manoeuvres. No elapsed-run limit. A lost-tape
 * pivot still times out. Non-NULL is a latched terminal reason. */
const char *F1_Advance(uint32_t now);
/* Check manoeuvre deadlines, BTN2 and IR/US1 freshness before EVERY command.
 * Supply the latest valid US1 millimetres; on fault/stale input stop instead. An armed
 * return on filtered 000 with US1 <55 mm requests a terminal stop, no dwell.
 * F1_KEEP_STOPPED requests zero on BOTH wheels, including first entry to hold. */
F1_Command F1_CommandAt(uint32_t now, uint32_t fresh_us1_mm);

/* (not legacy) Every tick: odo_mm = the distance driven (mean of both wheels' |pulses|, mm, since boot), arc_mm = both
 * wheels' |pulses| added (mm) - the pivot angle comes from it. */
void F1_Odometry(float odo_mm, float arc_mm);
/* (not legacy) Every tick: the heading from the encoders (deg, counter-clockwise +; right wheel minus left wheel / track) */
void F1_Heading(float deg);
float F1_PivotDeg(void); /* the angle of the current home pivot (screen / tests) */
bool F1_F3Bend(int *up_mm, char *kind, int *deg, bool *retried); /* F3's bend trigger + pivot (screen row 8) */
/* F4 (screen row 8): US1 at the stone stop, crossings west to the 8 / east home, the leg (false = not F4) */
bool F1_F4Info(int *us_mm, unsigned *x_out, unsigned *x_home, unsigned *leg, unsigned *k_pct, int *drift);
/* k_pct: the measured turn scale x100 (0 = not yet); drift: its heading off the bottom line's at the stone, degrees */
bool F1_F4OffLine(void); /* F4 is on one of its own legs (off the tape): a missing US1 echo there does not stop it */
float F1_LastPivotDeg(void); /* the last home pivot's measured angle, kept after it ends (0 until a run pivots) */
/* (not legacy) The outer digital IRs' live levels (same front bar): the home outline needs ALL FIVE black (F1_OUTLINE_ALL5) */
void F1_SideLevels(bool left_high, bool right_high);
/* (not legacy) true while F1 backs off before the pivot: the board then holds both wheels to equal pulses (into the zone F1
 * holds the line's heading itself) */
bool F1_EncoderStraight(uint32_t now);
/* (not legacy) F2: every tick, whether F2's outbound branch turn is running (the distances out to / from the T are noted) */
void F1_BranchTurning(bool turning);
/* (not legacy) true for a run that goes home by turning 180 (F1, and F2 with F2_RETURN_TURN): US1 is not needed until the wall */
bool F1_TurnsHome(void);
F1_Phase F1_PhaseAt(uint32_t now);
unsigned F1_BranchCount(void);
bool F1_FinishArmed(void); /* standalone outbound three-black reverse finish, not return line end */
#endif
