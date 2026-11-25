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
 *   6  BACK OFF from the dropped block (the centre ends near the short line), TURN LEFT 90 on the spot (facing west),
 *      FIND LINE: the short line between the IRs (was: turn 180, back north, turn right 90)
 *   7  WEST   follow the short line, stop at block 2 and GRAB it
 *   8  BACK east along the short line until the centre is TRA_B2_SA_X_MM west of the vertical line, TURN RIGHT 90 (north;
 *      open floor, block 1 is on the other side of the line), NORTH TRA_B2_NORTH_MM (block 2 at block 1's depth) into the
 *      LEFT blue scoring area, DROP
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
/* Leaving the start zone (both routines): straight, IRs ignored, until both IRs are white for TRA_LEAVE_CLEAR_MS after
 * crossing the box / its outline (the IRs start ~16 cm inside the 650 mm zone), or TRA_LEAVE_MAX_MM at the latest */
#ifndef TRA_LEAVE_CLEAR_MS
#define TRA_LEAVE_CLEAR_MS 30U
#endif
#ifndef TRA_LEAVE_MIN_MM
#define TRA_LEAVE_MIN_MM 250 /* never saw anything dark: out after this far */
#endif
#ifndef TRA_LEAVE_MAX_MM
#define TRA_LEAVE_MAX_MM 400
#endif
#ifndef TRA_EAST_MAX_SIDE_MM
#define TRA_EAST_MAX_SIDE_MM 80 /* along the bottom line: more than this sideways = an IR reads black where there is none: stop */
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
#ifndef TRA_J2_USE_SQUARE
#define TRA_J2_USE_SQUARE 0 /* 1 = take J2 from block 1's green square when the IRs read it as black (exact). OFF by default:
                             * TRA_J2_DROP_MM (345) was tuned on the field with whatever J2 the TR took then - if it was the
                             * fallback (the square hid the branch), the square moves the drop ~11 cm SOUTH (sim) and
                             * TRA_J2_DROP_MM must be re-tuned. The AUTO page shows "J2 ... GREEN" when the square read black. */
#endif
#ifndef TRA_J2_SQUARE_MM
#define TRA_J2_SQUARE_MM 60 /* both IRs black this long while carrying block 1 = its green square (139 mm) read as black */
#endif
#ifndef TRA_J2_DROP_MM
#define TRA_J2_DROP_MM 345 /* was 430 (370 landed on the boundary): block 1 landed partly in the other team's SA (field). From the J2 checkpoint (IR sensors over the branch) to the drop point.
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
 *   FIND LINE centre on the vertical line leaving the second T (as routine 1 does after its turn): a turn that left the TR
 *           off the junction would otherwise carry that error across the whole diagonal
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
#define TRB_T2_SEARCH_MM 2400 /* (was 2300: field over 20 s) slow down and look for the second T from here (sensor travel; map x 0.98: T2 at ~2590) */
#endif
#ifndef TRB_T2_MIN_MM
#define TRB_T2_MIN_MM 2450 /* the original left-IR rule counts from here */
#endif
#ifndef TRB_T2_MAX_MM
#define TRB_T2_MAX_MM 2950 /* not found by here: stop */
#endif
#ifndef TRB_MOVE1_FWD_MM
#define TRB_MOVE1_FWD_MM 1779 /* T2 -> level with the horizontal line on the right (north). MEASURED 178 cm */
#endif
#ifndef TRB_MOVE1_RIGHT_MM
#define TRB_MOVE1_RIGHT_MM 1460 /* T2 -> the far-right vertical line (east), centre to centre. MEASURED 148 cm from the T2 line's inner (east)
                                 * edge to the far-right line's OUTER (east) edge = 146 cm centre to centre (was 1480, 1492 from the map) */
#endif
#ifndef TRB_LINE_SHORT_MM
#define TRB_LINE_SHORT_MM 50 /* the diagonal (and the move back after turning round) stops the TR's centre this far short of
                              * the far-right line's centre (at 0.3 for its last 15 cm); FIND LINE then centres the line between the
                              * IRs, checking every black is the tape and not the gray strip */
#endif
#ifndef TRB_TAPE_CHECK_MM
#define TRB_TAPE_CHECK_MM 35 /* EDGE FIND LINE: a black that is still black this far on is not the 20 mm line but the boundary's
                              * gray strip (the IRs read it as black, field) */
#endif
#ifndef TRB_IR_SIDE_MM
#define TRB_IR_SIDE_MM 25 /* each IR's distance from the TR's centre line (sideways) */
#endif
#ifndef TRB_ALIGN1_MAX_MM
#define TRB_ALIGN1_MAX_MM 90 /* FIND LINE after the diagonal: look this far toward the field edge, then back the other way.
                               * With the diagonal TRB_LINE_SHORT_MM short this finds the line if the diagonal ended up to ~8 cm
                               * shorter, and the TR's side never reaches the black (if the line is already behind, the left IR
                               * meets it first and FIND LINE stops there) */
#endif
#ifndef TRB_PRESLIDE_MM
#define TRB_PRESLIDE_MM 100 /* before turning north at the second T: slide this far north along its vertical line so the back
                             * wheel clears the AR area's wall while it turns (field: it hit it, and the turn came out 80-85) */
#endif
#ifndef TRB_CLEAR_SPEED
#define TRB_CLEAR_SPEED 0.8f /* the short moves round the 180 turn (clear the wall, back to the line) - was 0.5 */
#endif
/* SQUARE UP on the SECOND T's vertical line, right after the turn north: the TR has no gyro - its heading comes from the
 * wheels - and the turn came out at 80-85 deg on the field (the back wheel hit the AR wall / the omni wheels slipped), which
 * swung the whole diagonal: past the far-right line into the black, and close to the lower block. The line is the reference:
 * drive TRB_SQ_MM north with the LEFT IR riding the line's west edge; how far it had to slide sideways per mm forward IS the
 * angle between the TR and the line. Turn by that angle on the spot, centre on the line again, then the diagonal. */
#ifndef TRB_SQ_MM
#define TRB_SQ_MM 150 /* measure over this much forward travel */
#endif
#ifndef TRB_SQ_SPEED
#define TRB_SQ_SPEED 0.4f /* forward while measuring (was 0.3: field over 20 s) */
#endif
#ifndef TRB_SQ_KX
#define TRB_SQ_KX 0.1f /* sideways while measuring (tracks up to ~25 deg at TRB_SQ_SPEED) */
#endif
#ifndef TRB_SQ_SEEK_MM
#define TRB_SQ_SEEK_MM 40 /* never more than this past where the IR last saw the line (lost: no turn) */
#endif
#ifndef TRB_SQ_MAX_DEG
#define TRB_SQ_MAX_DEG 25.0f /* a bigger "angle" is not believed (no change) */
#endif
#ifndef TRB_SQ_MIN_DEG
#define TRB_SQ_MIN_DEG 0.7f /* smaller: already square, no turn */
#endif
#ifndef TRB_PUSH_KX
#define TRB_PUSH_KX 0.15f /* PUSH SOUTH: sideways ratio while the right IR rides the line's west edge (tracks ~10 deg at 0.85) */
#endif
#ifndef TRB_TOUCH_CHECK_MM
#define TRB_TOUCH_CHECK_MM 20 /* diagonal, slow part: the right IR black and the left one still white this much further = the
                               * far-right line (the horizontal line turns both IRs black within a few mm) */
#endif
#ifndef TRB_TOUCH_RIGHT_MM
#define TRB_TOUCH_RIGHT_MM 30 /* after the diagonal stopped on the far-right line: FIND LINE goes left first, right at most this */
#endif
#ifndef TRB_TURN2_FRAC
#define TRB_TURN2_FRAC 0.8f /* the 180 with the top block turns this fast (the normal turns 0.5): its accuracy hardly matters -
                             * FIND LINE re-centres after it and the push south measures and corrects the angle */
#endif
#ifndef TRB_LINE_END_MM
#define TRB_LINE_END_MM 60 /* PUSH SOUTH: no IR has seen the line for this far = the far-right line has ended */
#endif
#ifndef TRB_SQ_RECHECK_DEG
#define TRB_SQ_RECHECK_DEG 12.0f /* a square-up reading bigger than this is measured again (field: the worst turn was 5-10 deg) */
#endif
#ifndef TRB_SQ_AGREE_DEG
#define TRB_SQ_AGREE_DEG 3.0f /* two readings this close = real (their average is used); further apart = the smaller one */
#endif
#ifndef TRB_DIAG_SLOW_MM
#define TRB_DIAG_SLOW_MM 150 /* (300 for the old diagonal) the diagonal's last this-much at 0.3 (was 150): a line-stop there coasts ~1 cm, not ~9 */
#endif
#ifndef TRB_SQ_TURN_K
#define TRB_SQ_TURN_K 0.05f   /* the square-up's correction turn: command ratio per degree (the heading hold uses 0.02) */
#endif
#ifndef TRB_SQ_TURN_MIN
#define TRB_SQ_TURN_MIN 0.05f /* (was 0.03: field over 20 s) ...but at least this, so the last degree does not crawl */
#endif
#ifndef TRB_SQ_TURN_MS
#define TRB_SQ_TURN_MS 1500U /* the small turn on the spot: at most this long */
#endif
#ifndef TRB_SQUARE_UP
#define TRB_SQUARE_UP 0 /* 1 = the stop-and-measure square-up at the second T (slow); 0 = the angle is fitted while following the
                         * second T's line north and corrected at the horizontal line */
#endif
#ifndef TRB_TURN1_FRAC
#define TRB_TURN1_FRAC 0.8f /* the left turn at the second T (normal turns 0.5); its error is fitted and corrected on the way north */
#endif
#ifndef TRB_HZ_BACKUP
#define TRB_HZ_BACKUP 0 /* 1 = also take 'right IR alone black 15 ms' as the horizontal line (fires on the follow's wobble) */
#endif
#ifndef TRB_UP_SEARCH_MM
#define TRB_UP_SEARCH_MM 1450 /* NO DIAGONAL (field: the diagonal locked onto the horizontal line / ran past the far-right line):
                               * after the square-up it FOLLOWS the second T's vertical line north at TRB_FAST_SPEED and from
                               * here (centre, north of the second T) searches for the horizontal line branching off RIGHT */
#endif
#ifndef TRB_UP_MAX_MM
#define TRB_UP_MAX_MM 1850    /* no right branch by here: stop ('NO HZ LINE') */
#endif
#ifndef TRB_HZ_PAST_MM
#define TRB_HZ_PAST_MM 170    /* then on until the IRs are this far past the horizontal line's centre (keeps the TR's back
                               * ~17 cm clear of the lower block on the slide), then SLIDE straight east to the far-right line */
#endif
#ifndef TRB_MOVE_SPEED
#define TRB_MOVE_SPEED 0.9f /* the diagonal (as the base routine; the slow finish is TRA_MOVE_SLOW_MM at 0.3) */
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
#define TRB_SOUTH_MM 1612 /* (1600 -> 1635 -> 1612: the top block's south face is MEASURED 50 cm north of the horizontal line, 3 cm
                           * nearer than the map, so the grab is 3 cm further south; 1612 = middle of the window where both blocks are inside) after turning round: drive this far south, pushing the lower block, so both blocks end in the SA */
#endif
#ifndef TRB_PUSH_SPEED
#define TRB_PUSH_SPEED 0.85f /* was 0.6: on the field the lower block stays ahead of (or is pushed by) the TR anyway */
#endif
#ifndef TRB_TIME_LIMIT_MS
#define TRB_TIME_LIMIT_MS 20000U /* stops at 20 s like routine 1 (was 30 s) */
#endif

/* After dropping block 1 the TR reverses this far (still facing north) before sliding back and turning round: with the
 * gripper only opened, the block was still between its jaws and the turn dragged it (field). This distance is then taken
 * off the drive south after the turn, so turn 3 still happens over the short line (if it backed past the short line, the
 * TR drives that bit back north after turning round). */
#ifndef TRA_J2_FIND_MM
#define TRA_J2_FIND_MM 120 /* after the left turn at J2: FIND LINE looks this far north first, then up to TRA_ALIGN_MAX_MM south */
#endif
/* Block 2 into the LEFT blue scoring area (west of the vertical line; block 1 is in the right one). */
#ifndef TRA_B2_SA_X_MM
#define TRA_B2_SA_X_MM 400 /* where it turns north: the TR's centre (and block 2) this far west of the vertical line - the middle
                            * of the left blue SA (~840 mm wide, map); block 1 sits ~80 mm EAST of the line */
#endif
#ifndef TRA_B2_NORTH_MM
#define TRA_B2_NORTH_MM 185 /* then north this far from the short line: block 2 lands at the depth block 1 does (sim: the TR's
                             * centre is ~185 mm north of the short line when block 1 drops) - block 1's depth is field-tuned
                             * (TRA_J2_DROP_MM: inside blue, short of red) */
#endif
#ifndef TRA_B2_SPEED
#define TRA_B2_SPEED 0.9f /* the moves with block 2 (open floor) */
#endif
#ifndef TRA_DROP_BACK_MM
#define TRA_DROP_BACK_MM 200
#endif
#ifndef TRA_BACK_SPEED
#define TRA_BACK_SPEED 0.4f
#endif
#if TRA_SOUTH_USE_SENSOR
#error "the back-off after the drop assumes the direct-distance return (TRA_SOUTH_USE_SENSOR 0)"
#endif

/* Straight MOVEs (routine 2's diagonal, the moves round the turn, routine 1's back-off): the last TRA_MOVE_SLOW_MM at
 * TRA_MOVE_SLOW_SPEED at most, so they end gently on target */
#ifndef TRA_MOVE_SLOW_MM
#define TRA_MOVE_SLOW_MM 150
#endif
#ifndef TRA_MOVE_SLOW_SPEED
#define TRA_MOVE_SLOW_SPEED 0.3f
#endif

/* ---- Arm ---- */
#ifndef TRA_ARM_SETTLE_MS
#define TRA_ARM_SETTLE_MS 150U /* after the gripper's pulse reaches closed / open: wait this long (the servo itself trails) */
#endif
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
  TRA_IDLE = 0, TRA_COUNTDOWN, TRA_EAST, TRA_SEARCH1, TRA_ADV1, TRA_TURN1, TRA_ALIGN1, TRA_NORTH, TRA_GRAB1, TRA_CARRY, TRA_SHIFT, TRA_DROP, TRA_BACKOFF,
  TRA_UNSHIFT, TRA_TURN2, TRA_SOUTH, TRA_ADV2, TRA_TURN3, TRA_WEST, TRA_GRAB2, TRA_B2_BACK, TRA_B2_TURN, TRA_B2_NORTH, TRA_B2_DROP,
  /* mode 2: the two blocks on the far-right vertical line, both into the bottom-right scoring area */
  TRB_EAST, TRB_SEARCH, TRB_ADV, TRB_PRESLIDE, TRB_TURN1, TRB_ALIGN0, TRB_SQ_EDGE, TRB_SQ_MEAS, TRB_SQ_TURN, TRB_SQ_ALIGN,
  TRB_UP, TRB_UPSEARCH, TRB_UPPAST, TRB_MOVE1, TRB_ALIGN1, TRB_NORTH, TRB_GRAB, TRB_MOVE2, TRB_TURN2, TRB_MOVE3, TRB_ALIGN2, TRB_SOUTH,
  TRB_DROP,
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
  float turn_frac;        /* ... at this top speed (fraction of the turn rpm), 0 = the normal TR_TURN_MAX_FRAC */
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
  float edge_seen;              /* routine 2 EDGE mode: sideways position where the edge IR last saw the line */
  float mv_slow_mm;             /* MOVE: the last this-much at TRA_MOVE_SLOW_SPEED */
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
  /* routine 2 SQUARE UP: least-squares fit of (forward, sideways) where the right IR crossed the line's inner edge */
  float sq_f0, sq_s0, sq_n, sq_sf, sq_ss, sq_sff, sq_sfs, sq_fmin, sq_fmax, sq_last_on;
  float align_max2;             /* FIND LINE: how far to look the OTHER way (past the start) */
  float align_max;              /* EDGE FIND LINE: how far to look toward the edge */
  bool align_gray;              /* FIND LINE (centre): check every black for the gray strip (routine 2, the far-right line) */
  float align_first_on;         /* FIND LINE: sideways position where the near IR first went black */
  float align_dir3;             /* EDGE FIND LINE phase 3: which way to the inner edge (-1 back from a checked tape, +1 from outside) */
  float trb_f0;                 /* routine 2: forward odometer when it faced north at the second T */
  uint32_t arm_at;              /* when the gripper's pulse reached closed / open in this state (0 = not yet) */
  uint32_t t_end;               /* when the run ended (DONE / TIME UP / FAULT), 0 = not yet: the screen keeps the finish time */
  float so_n, so_sf, so_ss, so_sff, so_sfs, so_s0, so_f_first, so_last_on, so_deg; /* PUSH SOUTH: fit of sideways vs forward
                                * on the line; the heading correction applied where the line ends */
  float so_min_n, so_seen_s, so_on_f;   /* PUSH SOUTH: fewest points for a fit; sideways position where the right IR last saw the line */
  bool so_fixed;
  bool bb_active;               /* R1 CARRY: both IRs black (block 1's green square?) since bb_mm */
  float bb_mm;
  bool j2_green;                /* screen: both IRs were black >= TRA_J2_SQUARE_MM while carrying block 1 (green reads black) */
  float b2_west_mm;             /* R1: how far it followed the short line west to block 2 (the centre was over J2 before) */
  uint8_t j2_by;                /* screen: how J2 was taken - 1 left IR on the branch, 2 green square, 3 not seen (assumed) */
  bool touch_on, touch_hz;      /* R2 diagonal: right IR on a line (checking) / both IRs saw the horizontal line */
  float touch_path;
  bool diag_touch;              /* screen: the diagonal stopped on the far-right line (it was long) */
  bool box_out, box_dark;       /* leaving the start zone: done / crossed something dark */
  float east_s0;                /* sideways position when it started following the bottom line */
  uint32_t box_clear;           /* leaving the start zone: both IRs white since (0 = not) */
  bool gray_seen;               /* EDGE FIND LINE took a black for the boundary (gray strip) and went back to the real line */
  bool sq_second;               /* SQUARE UP: this is the re-measurement after a big first reading */
  float sq_first;               /* SQUARE UP: the first reading (deg) */
  bool sq_on;                   /* SQUARE UP: the right IR saw the line on the previous call */
  float sq_deg;                 /* measured angle (clockwise +), 0 = not measured / not believed */
  uint8_t sq_result;            /* screen: 0 not yet, 1 corrected, 2 already square, 3 line lost, 4 too few points, 5 too big */
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
