#ifndef TR_AUTO_H
#define TR_AUTO_H

/**
 * Titan Robot autonomous routine for the first 20 seconds (pure logic, no hardware; the glue is tr_auto_app.c).
 *
 * Field (blue side): the TR starts in the TR-SZ against the wall facing EAST along the bottom line. A vertical line
 * leaves that line to the north (a T: only the LEFT IR sees it). Block 1 sits on the vertical line where a short
 * horizontal line leaves to the west toward block 2. The scoring areas (SA) are further north, at the end of the
 * vertical line.
 *
 *   1  EAST   follow the bottom line at full speed for TRA_J1_SEARCH_MM, then
 *      FIND T keep following slowly with the line test's logic until it FINDS the T on the left (not at a set distance)
 *   2  ADV    drive on so the wheels are over the junction, then TURN left 90 (arm now faces north)
 *   3  NORTH  follow the vertical line; stop when the ultrasonic sees block 1
 *   4  GRAB   close the arm on block 1
 *   5  CARRY  keep following north into the SA, SHIFT sideways into one SA, DROP (open the arm), SHIFT BACK onto the line
 *   6  TURN   180 degrees, SOUTH: follow the line back to the short horizontal line (right IR sees it)
 *   7  ADV    drive on, TURN right 90 (facing west), WEST: follow the short line, stop at block 2 and GRAB it
 *
 * Two IR sensors straddle the line. Left IR black = the line is on the left = strafe left; right IR black = strafe
 * right. The heading is held with the wheel-based heading estimate. If the 20 s run out the routine just stops and
 * hands control back. NOT tested on the real field: every distance below is a measurement to take.
 */

#include <stdbool.h>
#include <stdint.h>

/* ---- Timing ---- */
#ifndef TRA_TIME_LIMIT_MS
#define TRA_TIME_LIMIT_MS 20000U
#endif
#ifndef TRA_COUNTDOWN_MS
#define TRA_COUNTDOWN_MS 3000U /* after START: wait this long before moving */
#endif

/* ---- Speed ---- */
#ifndef TRA_TRANSLATE_RPM
#define TRA_TRANSLATE_RPM 130.0f /* wheel rpm at full command during the routine = speed level 5 (was 100 = level 4); 0.6 x 130 rpm = about 0.62 m/s */
#endif
#ifndef TRA_SPEED
#define TRA_SPEED 0.6f /* forward command ratio while following (0.6 = about 0.48 m/s) */
#endif
#ifndef TRA_EAST_SPEED
#define TRA_EAST_SPEED 0.9f /* routine 1, start -> TRA_J1_SEARCH_MM (nothing to find there). Braking to the search speed takes ~80 mm,
                             * well before the original left-IR rule starts. Set back to 0.6f (= TRA_SPEED) if it wobbles. */
#endif
#ifndef TRA_SLOW_SPEED
#define TRA_SLOW_SPEED 0.2f /* ratio when close to a block */
#endif
#ifndef TRA_SLOW_MM
#define TRA_SLOW_MM 300 /* slow down when the ultrasonic is closer than this */
#endif
#ifndef TRA_KX
#define TRA_KX 0.25f /* sideways correction ratio when only one IR sees the line */
#endif
#ifndef TRA_KH
#define TRA_KH 0.02f /* heading-hold gain: command ratio per degree of error */
#endif
#ifndef TRA_KH_MAX
#define TRA_KH_MAX 0.3f
#endif

/* ---- Distances (mm). MEASURE these on the real field, see tr_auto_app.c screen ---- */
/* MEASURED: the first T (the vertical line's near edge) is 1112-1114 mm from the start zone outline's outer edge (centre
 * ~1123; the map said ~1154). With the IRs ~120 mm inside the zone at the start, the IRs reach it after ~1240 mm.
 * Earlier estimate from the rulebook map (TR-SZ = 650 mm as a ruler): the vertical line is ~1800 mm from the start wall. The IR
 * sensors and the ultrasonic are 490 mm from the robot's back, which starts against the wall, so the sensors reach the vertical
 * line after about 1807 - 490 = ~1317 mm (this does not depend on where the turning centre is). VERIFY on the field. */
/* The T is FOUND, not assumed at a distance: from TRA_J1_SEARCH_MM on, the routine follows the line at TRA_SEARCH_SPEED
 * with the line test's logic (tr_linetest.c, search mode: drift corrections, crossings, junction side), which tells a drift
 * from a branch without knowing where the branch is. The distances are only loose limits:
 *   - before TRA_J1_SEARCH_MM it drives at full speed (the IRs are still in / just out of the start box: nothing to find).
 *     Well before both the map estimate (~1.3 m) and a possible ~0.7 m.
 *   - TRA_JUNC1_MAX_MM: never found it -> STOP (FAULT). It does not drive on or turn blindly: the opponent's half is ahead. */
#ifndef TRA_JUNC1_MIN_MM
#define TRA_JUNC1_MIN_MM 1150 /* from here on the ORIGINAL rule also counts (left black + right white = the T): it found the T in two games */
#endif
#ifndef TRA_J1_SEARCH_MM
#define TRA_J1_SEARCH_MM 1000 /* measured T at ~1240 mm of sensor travel: full speed until here (was 500) */
#endif
#ifndef TRA_SEARCH_SPEED
#define TRA_SEARCH_SPEED 0.3f /* forward ratio while searching: slow enough to stop on a 20 mm branch (0.35+ starts to miss them) */
#endif
#ifndef TRA_DIAG_FROM_MM
#define TRA_DIAG_FROM_MM 250 /* screen diagnostics: record where each IR first sees black from here on (past the start zone outline) */
#endif
#ifndef TRA_JUNC1_MAX_MM
#define TRA_JUNC1_MAX_MM 1400 /* sensor travel; measured T at ~1240: not found by here = stop (was 1500) */
#endif
#ifndef TRA_ADVANCE_MM
#define TRA_ADVANCE_MM 200 /* distance from the IR sensors to the robot's turning centre: drive this far after a junction. ESTIMATE: the
                            * centre of the three wheels is ~290 mm from the back of the robot, the sensors are 490 mm from the back. MEASURE */
#endif
#ifndef TRA_PICKUP_MM
#define TRA_PICKUP_MM 70 /* ultrasonic distance (mm) where the TR should STOP so the arm can grab the block (measured) */
#endif
#ifndef TRA_US_LEAD_MM
#define TRA_US_LEAD_MM 20 /* the ultrasonic updates every 50-100 ms and 2 readings are needed, so trigger this much early (20 at speed level 5) */
#endif
/* The block is not flat, so the ultrasonic wobbles by about +-20 mm even on a still block. A median of the last 3
 * readings stops a single bad reading from ending the approach early. */
#ifndef TRA_US_CONFIRM
#define TRA_US_CONFIRM 1U /* medians in a row at or below the pickup distance (the median already needs 2 of 3) */
#endif
/* FIND LINE after turn 1: slide sideways until the vertical line is between the IRs (left first, then right). */
#ifndef TRA_ALIGN_SPEED
#define TRA_ALIGN_SPEED 0.15f /* sideways ratio: slow enough to stop on a 20 mm tape */
#endif
#ifndef TRA_ALIGN_MAX_MM
#define TRA_ALIGN_MAX_MM 200 /* look this far to each side */
#endif
#ifndef TRA_NORTH_MAX_MM
#define TRA_NORTH_MAX_MM 1600 /* never saw block 1: give up after this */
#endif
/* Carrying block 1 north, the left IR crosses the short horizontal line at J2 (where the block sat). That is the checkpoint:
 * the TR then drives TRA_J2_DROP_MM further and drops the block. The black line ENDS where the scoring area starts, but
 * two IRs straddling the line cannot see an end, so the drop distance from J2 is measured instead. */
#ifndef TRA_J2_MIN_MM
#define TRA_J2_MIN_MM 0 /* J2 can't be seen before this distance after the pickup */
#endif
#ifndef TRA_J2_MAX_MM
#define TRA_J2_MAX_MM 250 /* if the left IR never saw J2, assume it here */
#endif
#ifndef TRA_J2_DROP_MM
#define TRA_J2_DROP_MM 430 /* from the J2 checkpoint (IR sensors over the branch) to the drop point. Estimated from the rulebook map:
                              * SA edge ~480 mm past J2, block ~140 mm wide and ~140 mm ahead of the sensors. VERIFY on the field */
#endif
/* Coming back south from the drop point the TR must bring its CENTRE over J2: that is TRA_J2_DROP_MM - TRA_ADVANCE_MM away. If
 * the IR sensors have not passed J2 yet (J2_DROP - 2 x ADVANCE >= 100 mm) the right IR finds the short horizontal line first and
 * the TR drives on ADVANCE; otherwise (the sensors are already past J2 after the turn-around) it just drives that distance. */
#ifndef TRA_SOUTH_EXPECT_MM
#define TRA_SOUTH_EXPECT_MM ((TRA_J2_DROP_MM) - 2 * (TRA_ADVANCE_MM))
#endif
#ifndef TRA_SOUTH_USE_SENSOR
#define TRA_SOUTH_USE_SENSOR (((TRA_J2_DROP_MM) - 2 * (TRA_ADVANCE_MM)) >= 100)
#endif
#ifndef TRA_SOUTH_DIRECT_MM
#define TRA_SOUTH_DIRECT_MM ((TRA_J2_DROP_MM) - (TRA_ADVANCE_MM)) /* distance that puts the centre over J2 */
#endif
#ifndef TRA_SOUTH_MIN_MM
#define TRA_SOUTH_MIN_MM (((TRA_SOUTH_EXPECT_MM) - 150) > 0 ? ((TRA_SOUTH_EXPECT_MM) - 150) : 0)
#endif
#ifndef TRA_SOUTH_MAX_MM
#define TRA_SOUTH_MAX_MM ((TRA_SOUTH_EXPECT_MM) + 250) /* if the right IR never saw it, assume the junction here */
#endif
#ifndef TRA_WEST_MAX_MM
#define TRA_WEST_MAX_MM 1300 /* never saw block 2: give up after this */
#endif
#ifndef TRA_JUNCTION_MS
#define TRA_JUNCTION_MS 15U /* the junction IR must see the line this long */
#endif

/* Before every turn the TR stops completely (the drive ramps gently during turns, so a moving TR coasted ~12 cm while turning) */
#ifndef TRA_STILL_MM
#define TRA_STILL_MM 0.5f /* moved less than this ... */
#endif
#ifndef TRA_STILL_MS
#define TRA_STILL_MS 60U  /* ... for this long = standing still */
#endif
#ifndef TRA_SETTLE_MAX_MS
#define TRA_SETTLE_MAX_MS 800U /* turn anyway after this long */
#endif

/* ---- MODE 2: the two blocks on the FAR-RIGHT vertical line -> both into the bottom-right scoring area ----
 * Distances from the rulebook map with the 650 mm TR start zone as the ruler, x 0.98: on the field the first T is 1112-1114 mm
 * (near edge) from the start zone outline's outer edge, the map said 1154 (centre). MEASURE THE REST ON THE FIELD.
 *   EAST    along the bottom line at TRB_FAST_SPEED past the first T (ignored), then FIND T: the second T (left)
 *   ADV/TURN advance so the centre is over it, stop, turn left (north)
 *   MOVE1   straight diagonal (omni, no turning) TRB_MOVE1_FWD_MM north and TRB_MOVE1_RIGHT_MM east: the IRs end on the
 *           far-right vertical line between the horizontal line and the top block
 *   FIND LINE, NORTH to the top block (ultrasonic), GRAB it
 *   MOVE2   back south and west, clear of the right wall and the barrier, so it can turn round; TURN 180
 *   MOVE3   east back onto the line; FIND LINE; SOUTH: follow the line - the carried block pushes the lower block ahead of
 *           it - and on (heading held) past the end of the line into the scoring area; DROP. */
#ifndef TRB_FAST_SPEED
#define TRB_FAST_SPEED 0.9f /* the long straight along the bottom line (0.6 = the normal routine speed) */
#endif
#ifndef TRB_T2_SEARCH_MM
#define TRB_T2_SEARCH_MM 2300 /* slow down and look for the second T from here (sensor travel; map x 0.98: T2 at ~2590) */
#endif
#ifndef TRB_T2_MIN_MM
#define TRB_T2_MIN_MM 2450 /* the original left-IR rule counts from here */
#endif
#ifndef TRB_T2_MAX_MM
#define TRB_T2_MAX_MM 2950 /* not found by here: stop */
#endif
#ifndef TRB_MOVE1_FWD_MM
#define TRB_MOVE1_FWD_MM 1779 /* T2 -> level with the horizontal line on the right (north) */
#endif
#ifndef TRB_MOVE1_RIGHT_MM
#define TRB_MOVE1_RIGHT_MM 1492 /* T2 -> the far-right vertical line (east) */
#endif
#ifndef TRB_MOVE_SPEED
#define TRB_MOVE_SPEED 0.9f
#endif
#ifndef TRB_NORTH_MAX_MM
#define TRB_NORTH_MAX_MM 900 /* never saw the top block: stop */
#endif
#ifndef TRB_CLEAR_BACK_MM
#define TRB_CLEAR_BACK_MM 260 /* after the grab: back this far (away from the barrier) ... */
#endif
#ifndef TRB_CLEAR_SIDE_MM
#define TRB_CLEAR_SIDE_MM 240 /* ... and this far west (away from the right wall) to turn round */
#endif
#ifndef TRB_SOUTH_MM
#define TRB_SOUTH_MM 1600 /* after turning round: drive this far south, pushing the lower block, so both blocks end in the SA */
#endif
#ifndef TRB_PUSH_SPEED
#define TRB_PUSH_SPEED 0.6f
#endif
#ifndef TRB_TIME_LIMIT_MS
#define TRB_TIME_LIMIT_MS 30000U /* mode 2 is longer than 20 s: after 20 s the drivers may take over, so it keeps going (a stick
                                  * movement still stops it) */
#endif

/* ---- Arm ---- */
#ifndef TRA_GRAB_MS
#define TRA_GRAB_MS 1500U
#endif
#ifndef TRA_DROP_MS
#define TRA_DROP_MS 1500U
#endif
/* Drop into ONE scoring area: the line runs along the boundary between two SAs, so before opening the arm the TR slides sideways
 * (omni wheels, heading held) by TRA_DROP_SHIFT_MM, drops the block, then slides back onto the line for the turn-around.
 * TRA_DROP_SHIFT_DIR: +1 = to the right, -1 = to the left (facing north). 0 mm = drop on the line (no shift). */
#ifndef TRA_DROP_SHIFT_MM
#define TRA_DROP_SHIFT_MM 80
#endif
#ifndef TRA_DROP_SHIFT_DIR
#define TRA_DROP_SHIFT_DIR 1
#endif
#ifndef TRA_SHIFT_SPEED
#define TRA_SHIFT_SPEED 0.3f /* sideways command ratio while sliding */
#endif
#ifndef TRA_SHIFT_MAX_MS
#define TRA_SHIFT_MAX_MS 2000U /* give up sliding after this long (then drop / turn anyway) */
#endif

typedef enum {
  TRA_IDLE = 0, TRA_COUNTDOWN, TRA_EAST, TRA_SEARCH1, TRA_ADV1, TRA_TURN1, TRA_ALIGN1, TRA_NORTH, TRA_GRAB1, TRA_CARRY, TRA_SHIFT, TRA_DROP,
  TRA_UNSHIFT, TRA_TURN2, TRA_SOUTH, TRA_ADV2, TRA_TURN3, TRA_WEST, TRA_GRAB2,
  /* mode 2: the two blocks on the far-right vertical line, both into the bottom-right scoring area */
  TRB_EAST, TRB_SEARCH, TRB_ADV, TRB_TURN1, TRB_MOVE1, TRB_ALIGN1, TRB_NORTH, TRB_GRAB, TRB_MOVE2, TRB_TURN2, TRB_MOVE3,
  TRB_ALIGN2, TRB_SOUTH, TRB_DROP,
  TRA_DONE, TRA_TIMEUP, TRA_FAULT
} TRAuto_State;

typedef struct {
  bool left, right;      /* IR sensors: true = black line seen */
  int32_t us_mm;         /* last valid ultrasonic distance, -1 = none */
  bool us_fresh;         /* a NEW ultrasonic reading arrived on this call */
  float dist_mm;         /* encoder distance along the robot's forward direction (continuous) */
  float strafe_mm;       /* encoder distance sideways, + = to the right (continuous) */
  float heading_deg;     /* heading estimate, clockwise positive */
  float heading_sign;    /* +-1: the direction a positive turn command rotates the heading (TR_HEADING_SIGN) */
  uint8_t turn_state;    /* 0 idle, 1 running, 2 done, 3 abort, 4 no motion (TR_TURN_*) */
  uint16_t arm_us;       /* current servo pulse */
  uint16_t arm_closed_us, arm_open_us;
  uint32_t now;
} TRAuto_In;

typedef struct {
  bool active;            /* the routine is driving (otherwise the joystick has control) */
  float x, y, w;          /* command ratios, same meaning as the joystick: x strafe right, y forward, w rotation */
  int8_t arm;             /* +1 close, -1 open, 0 hold */
  bool turn_req;          /* start a heading turn now ... */
  float turn_deg;         /* ... by this many degrees, clockwise positive */
  bool zero_heading;      /* set the heading to 0 now */
} TRAuto_Out;

typedef struct {
  TRAuto_State st;
  uint32_t t0, since, countdown_end, junc_since;
  float base_mm, hdg_target;
  float junc_mm;                /* FIND T backup rule: where the left IR went black */
  float shift_base, shift_done; /* sideways slide for the drop: start position and how far it went */
  uint8_t us_close, us_n;
  int32_t us_hist[3];
  bool junc_active, turn_requested, turn_seen_run, anchored;
  float settle_dist, settle_strafe; /* before a turn: encoder positions when it last moved */
  uint32_t settle_since;        /* before a turn: time it last moved more than TRA_STILL_MM */
  uint8_t wrong_junctions;
  int16_t t_found_mm;           /* screen: leg distance where the T was taken */
  uint8_t align_phase;          /* FIND LINE: 0 slide to the first side, 1 over the line, 2 slide back, 3 over the line */
  int8_t align_first;           /* FIND LINE: -1 try left first, +1 right first */
  float found_at;               /* FIND T: forward position of the branch centre */
  float mv_f0, mv_s0, mv_fwd, mv_side; /* MOVE: start position and target (forward / right, mm) */
  uint8_t mode;                 /* 1 = blocks 1 + 2 (left side), 2 = the two far-right blocks into the bottom-right SA */
  float align_seen;             /* FIND LINE: sideways position where the IR last saw the line */
  bool align_failed;            /* FIND LINE: not found within TRA_ALIGN_MAX_MM either side */
  int16_t align_mm;             /* FIND LINE: how far it slid (- = left) */
  uint8_t t_by;                 /* 0 not yet, 1 the original left-IR rule (from TRA_JUNC1_MIN_MM), 2 the line-test logic */
  /* diagnostics for the screen after a stop */
  const char *why;              /* fault reason ("" = none) */
  int16_t fault_mm;             /* leg distance where it stopped */
  int16_t l_on[3], r_on[3];     /* EAST / FIND T: leg distances of the LAST 3 white->black changes of each IR (after TRA_DIAG_FROM_MM) */
  uint8_t l_n, r_n;             /* how many there were in total */
  bool diag_prev_l, diag_prev_r;
  uint16_t search_drifts;       /* FIND T: drift corrections */      /* FIND T: junctions found on the RIGHT (there is none at the T: ignored, search continues) */
} TRAuto;

void TRAuto_Init(TRAuto *a);
/* Only while not running: 1 = blocks 1 + 2 (left), 2 = the two far-right blocks into the bottom-right SA. */
void TRAuto_SetMode(TRAuto *a, uint8_t mode);
void TRAuto_Start(TRAuto *a, uint32_t now);
void TRAuto_Abort(TRAuto *a);
bool TRAuto_IsRunning(const TRAuto *a);
bool TRAuto_NeedsUs(const TRAuto *a); /* the routine uses the ultrasonic in this state (else the app stops pinging: a ping blocks) */
void TRAuto_Step(TRAuto *a, const TRAuto_In *in, TRAuto_Out *out);
const char *TRAuto_StateText(TRAuto_State s);

#endif
