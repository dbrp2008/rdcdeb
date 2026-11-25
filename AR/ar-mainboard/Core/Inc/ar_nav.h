#ifndef AR_NAV_H
#define AR_NAV_H

/**
 * Artifact Retriever navigation (pure logic, no HAL): follows the black track lines with THREE front IR sensors
 * (left / centre / right), drives out to a key, pauses for an assumed passive pickup, turns around, follows the line home and
 * stops inside the start zone.
 *
 * Track model (Leviathan/blue ARA, robot starts in the start zone facing along the bottom line):
 *
 *   KEY A  = the key at the end of the VERTICAL line.
 *            out : follow the bottom line, ignore the curved branch on the right, at the CROSS (all three sensors
 *                  on the vertical line) drive forward a little, pivot RIGHT 90 degrees, follow the vertical line,
 *                  stop at the key.
 *            back: turn around at the key, follow the vertical line down to the CROSS, pivot LEFT 90 degrees,
 *                  follow the bottom line to the start zone.
 *
 *   KEY B  = the key on the right wall, reached by the U-curve and the diagonal.
 *            out : follow the bottom line and take the RIGHT-hand branch (the U-curve), then follow the line round
 *                  its corners to the key.
 *            back: turn around at the key and follow the line back to the start zone.
 *
 * Everything that depends on the real track and robot (speeds, distances, timings, sensor spacing) is a tunable
 * constant below. NOT yet tested on the real track: junction behaviour, especially Key B, needs tuning there.
 */

#include <stdbool.h>
#include <stdint.h>

/* ---- Tunables (percent = motor duty, 0..100) ---- */
#ifndef AR_BASE_PCT
#define AR_BASE_PCT 35 /* straight-line speed while following */
#endif
#ifndef AR_PIVOT_PCT
#define AR_PIVOT_PCT 30 /* wheel speed while turning on the spot (25 was tried: no better repeatability, three TRN4 runs spread 8.6 deg) */
#endif
#ifndef AR_COUNTDOWN_MS
#define AR_COUNTDOWN_MS 2000U /* after START: wait this long before moving, to get your hands clear */
#endif
#ifndef AR_CROSS_MS
#define AR_CROSS_MS 15U /* all three sensors must see the line this long to count as the CROSS */
#endif
#ifndef AR_CROSS_ADVANCE_MS
#define AR_CROSS_ADVANCE_MS 250U /* after the CROSS, drive on this long so the wheel axle is over the junction */
#endif
#ifndef AR_PIVOT_MIN_MS
#define AR_PIVOT_MIN_MS 250U /* turn at least this long before looking for the new line */
#endif
#ifndef AR_PIVOT_MAX_MS
#define AR_PIVOT_MAX_MS 3500U /* give up (fault) if the new line is not found */
#endif
#ifndef AR_STOP_MM
#define AR_STOP_MM 100 /* ultrasonic: closer than this means the key is reached. CALIBRATE with the real sensor */
#endif
#ifndef AR_US_CONFIRM
#define AR_US_CONFIRM 2U /* this many fresh readings in a row below AR_STOP_MM */
#endif
#ifndef AR_END_LOST_MS
#define AR_END_LOST_MS 900U /* no line at all this long on a key leg: stop with a lost-line fault */
#endif
#ifndef AR_HOME_LOST_MS
#define AR_HOME_LOST_MS 500U /* no line at all this long on the last leg home: treat as the start zone edge */
#endif
#ifndef AR_LOST_FAULT_MS
#define AR_LOST_FAULT_MS 4000U /* lost the line this long while it should be there: stop (fault) */
#endif
#ifndef AR_GRAB_PAUSE_MS
#define AR_GRAB_PAUSE_MS 500U /* stop at the key this long so the spring hand can snap on */
#endif
#ifndef AR_HOME_ADVANCE_MS
#define AR_HOME_ADVANCE_MS 700U /* drive on this long after the line ends so the whole robot is inside the zone */
#endif
/* ---- Hardcoded route (middle key -> top key -> right key). Distances are along the line centre, in cm, measured
 * on the ARA map with the rulebook 300 mm start zone as the ruler (about +-5 cm). CHECK AGAINST THE REAL FIELD. ---- */
/* Headings of the real lines, in degrees clockwise from north (the vertical line is 0, the horizontal run to the right key is
 * 90). Taken from the map picture, so MEASURE THEM on the real field with a protractor and correct them here. The route's turns
 * are derived: turn onto the diagonal = HDG_DIAG, onto the up-right segment = HDG_UP - HDG_DIAG, onto the horizontal = 90 - HDG_UP. */
/* Right key: 1 (default) = from the diagonal start point turn right 90 deg (east) and drive STRAIGHT to the key (the key is level with
 * that point, about AR_RIGHT_STRAIGHT_NOTE below), ignoring the zig-zag tape; the return mirrors it (reverse, turn left 90). 0 = follow the
 * zig-zag with the legs and headings below (diagonal, up-right segment, short horizontal). Map: 61+15+24 = ~100 cm east, ~0.5 cm north. */
#ifndef AR_RIGHT_STRAIGHT
#define AR_RIGHT_STRAIGHT 1
#endif
#ifndef AR_HDG_DIAG_DEG
#define AR_HDG_DIAG_DEG 116 /* the diagonal, going down and to the right */
#endif
#ifndef AR_HDG_UP_DEG
#define AR_HDG_UP_DEG 27 /* the short segment, going up and to the right */
#endif
/* Angle of each pivot in the TRN4 turn-calibration mode (four turns). Use 90 to check the track width; try 116 to check a long turn. */
#ifndef AR_TRN_DEG
#define AR_TRN_DEG 90
#endif
/* Turning point after the top key (ALL): 1 (default) = reverse until the front US reads AR_TOP_TURN_MM from the top stone, then
 * turn east (the stone stays when the key is taken). AR_TOP_TURN_LEAD_MM ends a little early for the time the readings take.
 * If the US gives nothing usable, it stops after AR_TOP_TURN_MAX_CM by encoder. 0 = reverse a fixed AR_TOP_BACK_CM. */
#ifndef AR_TOP_TURN_US
#define AR_TOP_TURN_US 1
#endif
#ifndef AR_TOP_TURN_MM
#define AR_TOP_TURN_MM 200 /* the old 13 cm reverse ended about 230 mm from the stone; the turn needs to be a bit closer. TUNE. */
#endif
#ifndef AR_TOP_TURN_LEAD_MM
#define AR_TOP_TURN_LEAD_MM 15
#endif
#ifndef AR_TOP_TURN_MAX_CM
#define AR_TOP_TURN_MAX_CM 30
#endif
#ifndef AR_TOP_BACK_CM
#define AR_TOP_BACK_CM 13 /* top key -> where the diagonal leaves the vertical line (reverse this far) */
#endif
#ifndef AR_DIAG_CM
#define AR_DIAG_CM 68 /* the diagonal, down and to the right */
#endif
#ifndef AR_UP_CM
#define AR_UP_CM 34 /* short segment, up and to the right */
#endif
/* Return to the start zone after a MID / TOP / TOP1 run (encoder-only): MID reverses AR_J_TO_SZ_CM east from J; TOP / TOP1
 * reverse (AR_TOP_KEY_FROM_J_CM - AR_TOP_BACK_CM) down to J, pivot right 90 deg (face east) and drive AR_J_TO_SZ_CM east.
 * Set AR_ROUTE_HOME 0 to stop at the key instead. J -> start zone left edge is about 77 cm on the map; the rest goes into the
 * 30 cm zone. TUNE ON THE FIELD. ALL and RGT1 first MIRROR the route back from the right key (reverse the right-key leg exactly,
 * pivot left 63, reverse AR_UP_CM, pivot right 89, reverse AR_DIAG_CM, pivot left 116) and then use the TOP return above. */
#ifndef AR_ROUTE_HOME
#define AR_ROUTE_HOME 1
#endif
#ifndef AR_J_TO_SZ_CM
#define AR_J_TO_SZ_CM 90
#endif
#ifndef AR_SZ_TO_J_CM
#define AR_SZ_TO_J_CM AR_J_TO_SZ_CM /* TOP / RGT from the start zone: drive west this far to junction J. TUNE. */
#endif
/* Junction J by the front US (the middle stone stays on the field and the AR faces it on the bottom line): stop when the US
 * reads AR_J_US_MM, i.e. the reading with the AR's axle on J, facing the middle stone. Used driving west from the start zone (TOP,
 * RGT) and reversing from the middle key (MID, TOP2, ALL). The US is only trusted within +-AR_J_WINDOW_CM of the encoder distance
 * (AR_SZ_TO_J_CM / AR_MID_BACK_CM); with no usable reading at all the encoder distance decides. 0 = encoder only. */
#ifndef AR_J_US
#define AR_J_US 1
#endif
#ifndef AR_J_US_MM
#define AR_J_US_MM 680 /* ESTIMATE: map J -> middle key 81 cm minus about 13 cm from the sensor to the axle. MEASURE: axle on J, read FC. */
#endif
#ifndef AR_J_WINDOW_CM
#define AR_J_WINDOW_CM 25
#endif
#ifndef AR_J_TO_TURN_MAX_CM
#define AR_J_TO_TURN_MAX_CM 71 /* RGT: north from J to the turning point if the US never sees the top stone (map: 84 - 13) */
#endif
#ifndef AR_TOP_KEY_FROM_J_CM
#define AR_TOP_KEY_FROM_J_CM 84
#endif
/* Front US as a backup on the last leg east into the start zone: if > 0, that leg also ends when the front US sees something
 * closer than this many mm (the east wall). 0 = off (encoder distance only). Set it once you know the wall gap. */
#ifndef AR_HOME_WALL_MM
#define AR_HOME_WALL_MM 0
#endif
/* Wheel feedback edges per revolution and wheel diameter give the edges per cm of travel (mean of both wheels).
 * AR_FB_EDGES_PER_REV: the motor library's 114 is quoted "for a 12 V supply"; this AR runs on 24 V, so MEASURE IT: hold B1 on
 * the WHEEL TEST screen for exactly 20 wheel turns and divide the edge count E by 20. */
#ifndef AR_FB_EDGES_PER_REV
#define AR_FB_EDGES_PER_REV 115.0f /* measured on the 24 V AR: about 2300 edges per 20 wheel turns, both wheels */
#endif
#ifndef AR_WHEEL_DIAMETER_MM
#define AR_WHEEL_DIAMETER_MM 65.0f /* measured about 65-66 mm */
#endif
#ifndef AR_EDGES_PER_CM
#define AR_EDGES_PER_CM (AR_FB_EDGES_PER_REV / (3.14159265f * AR_WHEEL_DIAMETER_MM / 10.0f))
#endif
#ifndef AR_PIVOT_DEG_PER_S
#define AR_PIVOT_DEG_PER_S 90 /* PLACEHOLDER pivot rate at AR_PIVOT_PCT; only used to size the minimum pivot time */
#endif
/* 0 (default) = the route uses ONLY the wheel encoders (distance, turn angle, straight-line balance) and the front
 * ultrasonic (key stops); the IRs are not read for it. 1 = also use the IRs for steering, junction J and ending pivots. */
#ifndef AR_ROUTE_USE_IR
#define AR_ROUTE_USE_IR 0
#endif
#ifndef AR_MID_BACK_CM
#define AR_MID_BACK_CM 81 /* encoder-only: middle key stop -> junction J (reverse this far, axle over J) */
#endif
/* Distance between the centres of the two drive wheels. PLACEHOLDER, MEASURE IT. Turn angle = encoders:
 * each wheel travels pi * track * degrees / 360. Tune so a commanded 90 deg turn really is 90 deg. */
#ifndef AR_TRACK_MM
#define AR_TRACK_MM 233 /* 246 mm measured. Photo pairs for TRN4 (four 90 deg turns vs 360): 236 mm = 4.7 deg past, 234 mm = 6.1 deg past (photo noise about +-3). 233 is within about 1 deg per turn, as good as the mat repeats. */
#endif
/* Straight-line balance (encoder only): err = left edges - right edges since the step began (about heading error).
 * Correction (percent points, added to one wheel and taken from the other) = KP * err + KI * integral(err). The
 * integral removes the steady heading offset that a plain P term leaves when one motor is weaker. */
/* Drift trim for wheels of slightly different size: the balance keeps (left edges x (1000 + trim) / 1000) equal to the right
 * edges. If the AR CURVES to the RIGHT on a straight leg, the left wheel covers more ground per edge: use a POSITIVE trim
 * (negative for a curve to the left). From the STR test: trim = 2000 x track x offset / distance^2 (all in cm; offset = how far
 * it ended to the side). e.g. 23.3 cm track, 10 cm offset after 200 cm: 2000 x 23.3 x 10 / 40000 = +12. */
#ifndef AR_DRIFT_TRIM_PERMILLE
#define AR_DRIFT_TRIM_PERMILLE 0
#endif
#ifndef AR_F20_CM
#define AR_F20_CM 20
#endif
#ifndef AR_STR_TEST_CM
#define AR_STR_TEST_CM 200
#endif
#ifndef AR_BALANCE_PCT_PER_EDGE
#define AR_BALANCE_PCT_PER_EDGE 3 /* KP: percent per edge of difference */
#endif
#ifndef AR_BALANCE_I_PCT_PER_EDGE_S
#define AR_BALANCE_I_PCT_PER_EDGE_S 4 /* KI: percent per (edge * second) */
#endif
#ifndef AR_BALANCE_MAX_PCT
#define AR_BALANCE_MAX_PCT 15 /* most one wheel may be sped up / slowed down */
#endif
/* Encoder-only pivots: the last AR_PIVOT_SLOW_ZONE_PCT percent of the arc runs at AR_PIVOT_SLOW_PCT wheel speed (less coast
 * after the stop). AR_PIVOT_EARLY_EDGES stops that many edges early to cancel the remaining coast (0 = off). */
/* Encoder-only straight legs (reversing and fixed-distance driving): the last AR_SLOW_ZONE_CM runs at AR_SLOW_SPEED_PCT, and the
 * leg ends AR_COAST_CM early to cancel the roll after the stop (a 35 % reverse overshot by 2-4 cm). Tune AR_COAST_CM on the field. */
/* Backup for the key stop (encoder-only route): if the wheels turn fewer than AR_STALL_MIN_EDGES edges in AR_STALL_MS while driving
 * to a key, the AR is pushing against the stone (the US may give no echo that close) and counts as arrived. */
#ifndef AR_STALL_MS
#define AR_STALL_MS 600U
#endif
#ifndef AR_STALL_MIN_EDGES
#define AR_STALL_MIN_EDGES 3
#endif
/* Straight legs start gently: the speed ramps from AR_STRAIGHT_START_PCT to the leg speed over AR_STRAIGHT_RAMP_MS, so the AR
 * does not twist as it sets off (one wheel gripping first). A MID run on the field left the start zone on the tape and then
 * ran about 5 deg off. */
#ifndef AR_STRAIGHT_START_PCT
#define AR_STRAIGHT_START_PCT 15
#endif
#ifndef AR_STRAIGHT_RAMP_MS
#define AR_STRAIGHT_RAMP_MS 300U
#endif
#ifndef AR_SLOW_ZONE_CM
#define AR_SLOW_ZONE_CM 10
#endif
#ifndef AR_SLOW_SPEED_PCT
#define AR_SLOW_SPEED_PCT 28 /* was 20: the motors may stall at low duty under load */
#endif
/* Extra: every REVERSING leg ends this many cm early (on top of AR_COAST_CM). */
#ifndef AR_REVERSE_TRIM_CM
#define AR_REVERSE_TRIM_CM 1.0f
#endif
#ifndef AR_COAST_CM
#define AR_COAST_CM 1.0f
#endif
/* Pivots start gently: the wheel speed ramps from AR_PIVOT_START_PCT to the full pivot speed over AR_PIVOT_RAMP_MS, so the two
 * motors spin up together (a hard start shifts the AR sideways when one motor reacts faster). */
#ifndef AR_PIVOT_START_PCT
#define AR_PIVOT_START_PCT 15
#endif
#ifndef AR_PIVOT_RAMP_MS
#define AR_PIVOT_RAMP_MS 200U
#endif
/* Optional: long pivots (above 90 deg) stop short by this many percent of the angle beyond 90. 0 = off. Check with TRN4 and AR_TRN_DEG 116. */
#ifndef AR_PIVOT_LONG_TRIM_PCT
#define AR_PIVOT_LONG_TRIM_PCT 0.0f /* off: it was based on a misread test; set only if turns above 90 deg overshoot more than 90 deg ones */
#endif
#ifndef AR_PIVOT_SLOW_ZONE_PCT
#define AR_PIVOT_SLOW_ZONE_PCT 30
#endif
#ifndef AR_PIVOT_SLOW_PCT
#define AR_PIVOT_SLOW_PCT 25 /* last stretch of a pivot */
#endif
#ifndef AR_PIVOT_EARLY_EDGES
#define AR_PIVOT_EARLY_EDGES 0
#endif
/* Between route steps the AR stands still: at least AR_STEP_SETTLE_MS, then until neither wheel has moved for AR_STILL_MS
 * (it has really stopped rolling), but never longer than AR_STEP_SETTLE_MAX_MS. Pivots add AR_PIVOT_SETTLE_EXTRA_MS to the
 * minimum. The run screen shows the most the AR rolled (edges) and the longest wait (ms) during the last run. */
#ifndef AR_STEP_SETTLE_MS
#define AR_STEP_SETTLE_MS 150U
#endif
#ifndef AR_STILL_MS
#define AR_STILL_MS 80U
#endif
#ifndef AR_STEP_SETTLE_MAX_MS
#define AR_STEP_SETTLE_MAX_MS 600U
#endif
#ifndef AR_PIVOT_SETTLE_EXTRA_MS
#define AR_PIVOT_SETTLE_EXTRA_MS 0U
#endif
#ifndef AR_STEP_MAX_MS
#define AR_STEP_MAX_MS 25000U /* one route step may take at most this long, else fault */
#endif
#ifndef AR_REV_TRIM_PCT
#define AR_REV_TRIM_PCT 85 /* reversing steers gently: the slower wheel runs at this percent of the other */
#endif

#ifndef AR_TURNAROUND_DIR
#define AR_TURNAROUND_DIR (-1) /* turn-around direction at the key: -1 = left, +1 = right */
#endif

typedef struct {
  bool l, c, r; /* true = this sensor sees the black line */
} AR_Line;

/* A / B = line-following key runs. MID / TOP / ALL = the HARDCODED route (see the route block below):
 *   MID = middle key only (stops after reversing back to the junction J)
 *   TOP = middle + top key (stops after reversing 13 cm back down from the top key)
 *   ALL = middle + top + right key, then turn around and follow the line home */
/* Route modes (B2 on the run screen cycles them in this order; A / B are the old line-following runs):
 *   MID  = middle key from the start zone, then home      TOP  = top key from the start zone (west to J, north), then home
 *   RGT  = right key from the start zone (west to J, north to the turning point, east), then home
 *   TOP2 = middle key and top key in one run, then home    ALL  = all three keys, then home */
typedef enum {
  AR_KEY_A = 0, AR_KEY_B = 1, AR_KEY_MID = 2, AR_KEY_TOP = 3, AR_KEY_RGT = 4, AR_KEY_TOP2 = 5, AR_KEY_ALL = 6,
  AR_KEY_TOP1 = 7, AR_KEY_RGT1 = 8, AR_KEY_TRN4 = 9, AR_KEY_STR = 10, AR_KEY_F20 = 11
} AR_Key;
#define AR_KEY_COUNT 12
/* Test modes that start part-way, in the pose the previous stage ends in (so one key can be tested alone):
 *   TOP1 = start ON junction J facing WEST (where MID ends); turns north, top key, then home.
 *   RGT1 = start at the diagonal start point on the vertical line facing NORTH (where TOP ends); diagonal, up-right
 *          segment, horizontal to the right key, stops there (no trip home).
 *   TRN4 = turn calibration: four 90 deg right pivots in a row (360 deg). Mark the start heading on the floor; any error after
 *          the four turns is four times the error of one.
 *   F20  = encoder accuracy test: drive AR_F20_CM (20 cm) forward and stop. Measure the real distance with a ruler; the run
 *          screen shows the edges counted (left / right) and the distance they mean in mm.
 *   STR  = straight-line test: drive AR_STR_TEST_CM forward and stop. Start along a straight tape; the sideways offset at the end
 *          (and whether the path curved) sets AR_DRIFT_TRIM_PERMILLE. */

typedef enum {
  AR_PH_IDLE = 0,    /* waiting in the start zone */
  AR_PH_COUNTDOWN,   /* START pressed: about to move */
  AR_PH_OUT,         /* driving to the key */
  AR_PH_GRAB,        /* stopped at the key */
  AR_PH_TURNAROUND,  /* spinning round to face home */
  AR_PH_RETURN,      /* driving home with the key */
  AR_PH_HOME_ADVANCE,/* line ended: driving into the start zone */
  AR_PH_DONE,        /* back in the start zone, stopped */
  AR_PH_FAULT        /* stopped because something went wrong */
} AR_Phase;

typedef enum { AR_FAULT_NONE = 0, AR_FAULT_LOST, AR_FAULT_TURN, AR_FAULT_STEP } AR_Fault;

typedef struct {
  int8_t left, right; /* wheel speeds, percent, + = forward. Right turn = left faster than right. */
} AR_Cmd;

typedef enum { AR_SUB_FOLLOW = 0, AR_SUB_ADVANCE, AR_SUB_PIVOT } AR_Sub;

typedef struct {
  AR_Phase phase;
  AR_Key key;
  AR_Fault fault;
  AR_Sub sub;
  uint8_t leg;
  int8_t pivot_dir;
  int8_t last_side; /* -1 left, +1 right, 0 none: where the line was last seen off-centre */
  bool pivot_seen_off;
  bool lost;
  bool leg_seen_line; /* the line has been seen on this leg (so "no line" now can mean its end) */
  uint32_t phase_since, sub_since, lost_since, cross_since, countdown_end;
  uint8_t us_close;
  bool cross_active;
  /* hardcoded route state */
  uint8_t step, step_sub;
  int32_t bal_i; uint32_t bal_t; /* balance integral (edge*ms) and its last update time */
  uint32_t odo_l, odo_r, step_l, step_r; /* per-wheel feedback edges, and their values at the start of the step */
  uint32_t odo;        /* wheel feedback edges (mean of both wheels), set by AR_Nav_SetOdometer */
  uint32_t step_odo;   /* odo when this step began */
  uint32_t zero_odo;   /* odo when the axle was over junction J */
  uint32_t out_edges;  /* edges travelled from J to the middle key (reversed exactly the same amount) */
  uint32_t stall_odo, stall_since; /* wheel-stall detector: pushing against the stone */
  uint32_t settle_l, settle_r, still_since; /* settle: wheel counts last seen and since when they have not changed */
  uint32_t settle_start_l, settle_start_r;
  uint16_t coast_max, settle_max_ms; /* most edges rolled while settling, longest settle (diagnostics for the run screen) */
  uint32_t top_back_edges; /* edges reversed from the top key to the turning point */
  bool top_back_set;
  bool j_us_seen;          /* this leg has had a usable US reading (else the encoder distance decides) */       /* top_back_edges is a measured value (else the return assumes AR_TOP_BACK_CM) */
  uint32_t top_edges;  /* edges travelled from J to the top key (reversed exactly on the way home) */
  uint32_t leg_edges;  /* edges travelled on the last key-approach step (the right key leg; reversed exactly on the way home) */
  uint32_t step_since;
} AR_Nav;

void AR_Nav_Init(AR_Nav *n);
/* Only while idle / done / fault. */
void AR_Nav_SetKey(AR_Nav *n, AR_Key key);
/* Starts a run (from IDLE, DONE or FAULT). */
void AR_Nav_Start(AR_Nav *n, uint32_t now_ms);
/* Stop at once and go back to IDLE. */
void AR_Nav_Abort(AR_Nav *n);
/* Wheel feedback edges (mean of both wheels, wraps at 2^32). Call before each AR_Nav_Step. */
void AR_Nav_SetOdometer(AR_Nav *n, uint32_t edges);
/* Per-wheel counts (left = J3, right = J1); also sets the mean odometer. Used for straight-line balance. */
void AR_Nav_SetWheelOdometer(AR_Nav *n, uint32_t left_edges, uint32_t right_edges);
/* True when this run needs valid IR readings (key A/B, or a route built with AR_ROUTE_USE_IR 1). */
bool AR_Nav_NeedsIR(const AR_Nav *n);
/* "A", "B", "MID", "TOP", "RGT", "TOP2", "ALL", "TOP1", "RGT1" or "TRN4". */
const char *AR_Nav_KeyText(const AR_Nav *n);
/* Call every few ms. us_mm < 0 means no valid distance; us_fresh is true only on the pass where a NEW reading arrived. */
AR_Cmd AR_Nav_Step(AR_Nav *n, uint32_t now_ms, AR_Line s, int32_t us_mm, bool us_fresh);

const char *AR_Nav_PhaseText(const AR_Nav *n);

#endif
