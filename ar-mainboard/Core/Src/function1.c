/* Function 1: launch, count crossings without a branch turn, turn back on a US1
 * target, and finish. Inputs are timestamps and qualified sensor events;
 * outputs are wheel requests, not writes to STM32 peripherals. */
#include "function1.h"
#include <stddef.h>

#define LAUNCH_MS 750u
#define LAUNCH_PERCENT 15.0f
#define REVERSE_MS 200u                /* armed three-black finish only */
#define FINISH_INSIDE_PERCENT 12.5f
#define US_APPROACH_MS 350u            /* follow from the first valid near reading */
#define US_STOP_HOLD_MS 1000u
#define US_REVERSE_MS 500u
#define US_REVERSE_PERCENT 20.0f
#define US_PIVOT_PERCENT 12.5f
#define US_SPIN_MIN_MS 1000u
#define US_SPIN_LIMIT_MS 10000u        /* includes the minimum spin */
#define US_SETTLE_MS 450u              /* same rotation after confirmed pickup */
#define ALL_BLACK (LINE_LEFT | LINE_CENTRE | LINE_RIGHT)

typedef struct {
  F1_Phase phase;
  uint32_t phase_at, near_at, return_scan_from;
  unsigned filtered, branches;
  bool f2_base; /* F2 owns its tally and has no left preference on 101 */
  bool split_left; /* F1 forward steering only: filtered 101 until exactly 010 */
  bool approaching; /* overlay on the forward leg; keep its finish marker live */
  bool return_marker; /* fresh 111 on this return, independent of the next-run reverse finish */
  BranchLatch crossing;
  bool next_armed, run_armed;
  bool saw_clear, pickup_pending;
#if !F1_LEGACY
  int8_t last_side;      /* -1 / +1: the side the line last slipped out to (100/110 left, 001/011 right), 0 = centred */
  bool lost;             /* the previous mask was 000 */
  uint32_t lost_at;      /* when it became 000 */
  uint32_t split_at;     /* when the 101 latch was set */
  bool turn_home;        /* F1: home by turning 180 and following the line (F2: retrace) */
  float odo, arc;        /* latest encoder totals from the board (mm) */
  float odo_mark, arc_mark; /* at the current phase's entry */
  float odo_start, odo_out; /* at the start / when the target hold began */
  float back_done;       /* how far it reversed before the pivot */
  float stall_odo; uint32_t stall_at; /* home: odometry at the last progress check (the wall backup) */
  bool side_l, side_r;   /* the outer digital IRs, live */
  bool in_zone;          /* home: past the start-zone outline - straight on, no line following */
  bool edge_hold;        /* home: 2+ IRs black near home - straight while checking it is the outline */
  float zone_lat, zone_odo; /* home, in the zone: sideways shift so far (mm, + = left of the line) and its odometer mark */
  float edge_odo;        /* ...since where */
  float seen_odo[5];     /* where each IR (L, C, R, outer L, outer R) was last black */
  float hdg, line_hdg, hdg_odo; /* encoder heading; its average while following home (= the line's direction); last odo */
  float home_mark, home_exp; /* home: odo where the leg home starts, and how long it should be */
  /* F2 home: the outbound branch turn (odo at its start / end), then on the way back: 0 = to the T, 1 = advancing past it,
   * 3 = pivoting left, 2 = done (home along the bottom line) */
  float odo_b0, odo_b1; bool b_on, b_done;
  unsigned t_state; float t_odo, t_arc0, t_vhdg; bool t_cmd, t_white;
  /* time backups: run start, outbound duration (to the target hold), return start, T advance / pivot start */
  uint32_t start_at, out_ms, return_at, t_at;
  /* F3 (the top-right key): the leg (F3_O_* out, F3_H_* home), and at its start: odo, arc, time; the heading the current
   * turn is measured from; out: odo where the U had turned 20 deg (the leg home is set from it) */
  bool f3_base, f3_cmd, f3_white, f3_armed, f3_cand, f3_near; /* f3_near: the line last slipped out on the kept side */
  unsigned f3_leg;
  float f3_odo, f3_arc0, f3_ref, f3_u0, f3_cand_odo, f3_adv; /* ..., f3_adv: this advance's length */ /* ..., where a corner's IRs first went white */
  uint32_t f3_at, f3_near_at;
  float f3_arm_sum, f3_arm_len, f3_arm_odo; bool f3_hdg_set;
  float f3_side_odo; bool f3_side_seen;
  /* the bend: the outer-right IR high since f3_sr_odo (debounce); the bend's trigger (distance up the leg, 'S' outer-right IR /
   * 'W' centre + left white, 0 = none yet), its pivot so far; one turn back + retry after a pivot with no tape */
  float f3_up_sum, f3_up_len, f3_up_odo; /* the up-right leg's heading, distance-weighted (40-140 mm up it) */
  bool f3_sr_on, f3_retried; float f3_sr_odo, f3_trig_d, f3_bend_deg; char f3_trig_kind;
  /* F4 (the bottom-left key): the leg (F4_O_* out, F4_H_* home), and at its start: odo, time, heading; the heading it
   * holds / pivots to (aim, from H0 = the bottom line's averaged heading west), the leg's length; the crossing latch */
  bool f4_base, f4_cmd, f4_white, f4_blk, f4_fixed, f4_pause, f4_was_straight;
  unsigned f4_leg, f4_xo, f4_xh, f4_prev; /* f4_prev: L/C/R at the last scan; f4_rise: where each last went black */
  float f4_rise[3]; bool f4_wall, f4_blocked; /* f4_wall: the west leg ended on US1 at the barrier */
  float f4_odo, f4_from, f4_aim, f4_len, f4_xodo, f4_h0_sum, f4_h0_len, f4_h0_odo;
  float f4_us, f4_det_odo, f4_s2, f4_key_odo, f4_stall_odo, f4_coast; /* f4_coast: the last stop's roll-on after a straight */
  float f4_done, f4_s2_act, f4_s3_act, f4_s4_act; unsigned f4_done_leg; /* out: each straight as driven (+ its roll-on) */
  uint32_t f4_at, f4_stall_at;
  /* the turn calibration on the bottom line (F4_O_CAL): the centre IR's edges (raw heading at each), how many so far, the
   * scale (true degrees per encoder degree, 1 until measured), done / failed, where it ended */
  float f4_cal_a[4], f4_k, f4_k_seen, f4_cal_odo, f4_drift, f4_outline; unsigned f4_cal_n; bool f4_cal_done, f4_cal_bad;
 /* out: where the outer-left IR first touched the diagonal (the V is a fixed way on) */ /* home: the U's top arm's heading (distance-weighted) */
#endif
} F1_Run;
#if !F1_LEGACY
enum { F3_O_BOTTOM, F3_O_U, F3_O_ARM, F3_O_VADV, F3_O_VPIV, F3_O_UP, F3_O_BADV, F3_O_BPIV, F3_O_EAST,
       F3_H_DOWN, F3_H_VADV, F3_H_VPIV, F3_H_ARMU, F3_H_BOTTOM, F3_O_BBACK };
/* F4, in order (each leg's next is the one after it) */
enum { F4_O_LINE, F4_O_SETTLE, F4_O_P1, F4_O_S1, F4_O_P2, F4_O_S2, F4_O_P3, F4_O_S3, F4_O_P4, F4_O_S4, F4_O_P5, F4_O_KEY,
       F4_H_S0, F4_H_P1, F4_H_S1, F4_H_P2, F4_H_S2, F4_H_P3, F4_H_S3, F4_H_P4, F4_H_S4, F4_H_P5, F4_H_LINE, F4_O_CAL };
#endif
static F1_Run run;
static float last_pivot_deg; /* the home pivot's measured angle: live while pivoting, kept after it ends (screen) */
float F1_LastPivotDeg(void) { return last_pivot_deg; }
#if !F1_LEGACY
bool F1_F3Bend(int *up_mm, char *kind, int *deg, bool *retried) { /* the screen: F3's bend so far (false = not F3 / none yet) */
  if (!run.f3_base || run.f3_trig_kind == 0) return false;
  *up_mm = (int)run.f3_trig_d; *kind = run.f3_trig_kind; *deg = (int)run.f3_bend_deg; *retried = run.f3_retried;
  return true;
}
bool F1_F4Info(int *us_mm, unsigned *x_out, unsigned *x_home, unsigned *leg, unsigned *k_pct, int *drift) {
  if (!run.f4_base) return false;
  *us_mm = (int)run.f4_us; *x_out = run.f4_xo; *x_home = run.f4_xh; *leg = run.f4_leg;
  *k_pct = run.f4_cal_done || run.f4_cal_bad ? (unsigned)(100.0f * run.f4_k_seen + 0.5f) : 0u;
  *drift = (int)(run.f4_drift < 0.0f ? run.f4_drift - 0.5f : run.f4_drift + 0.5f);
  return true;
}
#endif

void F1_Init(void) {
#if !F1_LEGACY
  const float odo = run.odo, arc = run.arc, hdg = run.hdg; /* the board's totals are since boot: keep them */
  run = (F1_Run){.phase = F1_IDLE};
  run.odo = odo; run.arc = arc; run.hdg = hdg;
#else
  run = (F1_Run){.phase = F1_IDLE};
#endif
}
static void start_run(uint32_t now, unsigned filtered, bool f2_base, bool either_side_high) {
  if (run.phase != F1_IDLE) return; /* a repeated start cannot interrupt a run */
  const bool armed = run.next_armed;
  run = (F1_Run){.phase = F1_LAUNCH, .phase_at = now,
                 .filtered = filtered, .f2_base = f2_base, .run_armed = armed};
  last_pivot_deg = 0.0f;
  BranchLatch_Start(&run.crossing, either_side_high);
}
void F1_Start(uint32_t now, unsigned filtered, bool either_side_high) {
#if !F1_LEGACY
  const float odo = run.odo, arc = run.arc;
#endif
  start_run(now, filtered, false, either_side_high);
#if !F1_LEGACY
  run.odo = odo; run.arc = arc;
  run.odo_start = odo; run.start_at = now;
  run.turn_home = F1_RETURN_TURN != 0;
#endif
}
void F1_StartPerScan(uint32_t now, unsigned filtered) {
#if !F1_LEGACY
  const float odo = run.odo, arc = run.arc, hdg = run.hdg;
#endif
  start_run(now, filtered, true, false);
#if !F1_LEGACY
  run.odo = odo; run.arc = arc; run.hdg = hdg;
  run.odo_start = odo; run.start_at = now;
  run.turn_home = F2_RETURN_TURN != 0;
#endif
}
#if !F1_LEGACY
void F1_StartF3(uint32_t now, unsigned filtered, bool either_side_high) {
  const float odo = run.odo, arc = run.arc, hdg = run.hdg;
  start_run(now, filtered, false, either_side_high);
  run.odo = odo; run.arc = arc; run.hdg = hdg;
  run.odo_start = odo; run.start_at = now;
  run.turn_home = true;
  run.f3_base = true; run.f3_leg = F3_O_BOTTOM; run.f3_odo = odo; run.f3_at = now; run.f3_u0 = -1.0f;
}
void F1_StartF4(uint32_t now, unsigned filtered, bool either_side_high) {
  const float odo = run.odo, arc = run.arc, hdg = run.hdg;
  start_run(now, filtered, false, either_side_high);
  run.odo = odo; run.arc = arc; run.hdg = hdg;
  run.odo_start = odo; run.start_at = now;
  run.turn_home = true;
  run.f4_base = true; run.f4_leg = F4_O_LINE; run.f4_odo = odo; run.f4_at = now; run.f4_aim = hdg; run.f4_k = 1.0f;
}
#else
void F1_StartF3(uint32_t now, unsigned filtered, bool either_side_high) { F1_Start(now, filtered, either_side_high); }
void F1_StartF4(uint32_t now, unsigned filtered, bool either_side_high) { F1_Start(now, filtered, either_side_high); }
#endif
void F1_SideSample(BranchSample side) {
  if (run.phase != F1_IDLE && !run.f2_base && BranchLatch_Entered(&run.crossing, side)) ++run.branches;
}
void F1_Stop(void) { run.phase = F1_IDLE; run.approaching = false; run.split_left = false; }
static bool on_outbound(void) { return run.phase == F1_LAUNCH || run.phase == F1_OUTBOUND; }
static F1_Phase base_phase_at(uint32_t now) {
  if (run.phase == F1_LAUNCH && (uint32_t)(now - run.phase_at) >= LAUNCH_MS) return F1_OUTBOUND;
  return run.phase;
}
F1_Phase F1_PhaseAt(uint32_t now) { return run.approaching ? F1_TARGET_APPROACH : base_phase_at(now); }
unsigned F1_BranchCount(void) { return run.branches; }
bool F1_FinishArmed(void) { return run.phase == F1_IDLE ? run.next_armed : run.run_armed; }
static void enter(F1_Phase phase, uint32_t now) {
#if !F1_LEGACY
  run.odo_mark = run.odo; run.arc_mark = run.arc;
#endif
  run.phase = phase; run.phase_at = now; run.approaching = false;
  run.split_left = false; /* do not carry outbound steering through a manoeuvre into the return */
}
static const char *finish(const char *reason) { F1_Stop(); return reason; }
#if !F1_LEGACY
void F1_Odometry(float odo_mm, float arc_mm) { run.odo = odo_mm; run.arc = arc_mm; }
void F1_SideLevels(bool left_high, bool right_high) { run.side_l = left_high; run.side_r = right_high; }
void F1_Heading(float deg) {
  /* following home (not on the outline): average the heading over F1_HEADING_AVG_MM = the line's direction */
  if (run.turn_home && run.phase == F1_RETURN && !run.in_zone && !run.edge_hold && !run.f3_hdg_set) {
    float a = (run.odo - run.hdg_odo) / F1_HEADING_AVG_MM;
    a = a < 0.0f ? 0.0f : (a > 1.0f ? 1.0f : a);
    run.line_hdg += a * (deg - run.line_hdg);
  }
  if (run.turn_home && run.phase == F1_RETURN && run.in_zone) { /* sideways travel off the line's direction (small angles) */
    run.zone_lat += (run.odo - run.zone_odo) * (deg - run.line_hdg) * 0.0174533f;
    run.zone_odo = run.odo;
  }
  run.hdg_odo = run.odo;
  run.hdg = deg;
}
float F1_PivotDeg(void) { return (run.arc - run.arc_mark) / F1_TRACK_MM * 57.29578f; }
/* home by turning: the return distance it should take, and how far it has gone */
static float home_expect(void) { return run.home_exp; }
static float home_done(void) { return run.odo - run.home_mark; }
static float t_deg(void) { return (run.arc - run.t_arc0) / F1_TRACK_MM * 57.29578f; }
static float t_expect(void) { return (run.odo_out - run.odo_b1) - run.back_done; } /* F2: key -> T, as driven out */
void F1_BranchTurning(bool turning) {
  if (!run.f2_base || !on_outbound()) return;
  if (turning && !run.b_on && !run.b_done) { run.b_on = true; run.odo_b0 = run.odo; }
  else if (!turning && run.b_on) { run.b_on = false; run.b_done = true; run.odo_b1 = run.odo; }
}
bool F1_TurnsHome(void) { return run.turn_home; }

/* ---- F3: the top-right key (function1.h F3_*) ---- */
static float f3_deg(void) { return (run.arc - run.f3_arc0) / F1_TRACK_MM * 57.29578f; } /* the corner pivot so far */
static void f3_to(unsigned leg, uint32_t now) {
  run.f3_leg = leg; run.f3_odo = run.odo; run.f3_arc0 = run.arc; run.f3_at = now;
  run.f3_cmd = run.f3_white = run.f3_armed = run.f3_cand = false;
  run.f3_adv = F3_CORNER_ADVANCE_MM;
}
/* a corner: the given IRs white for F3_CORNER_CONFIRM_MM; then the advance counts from where they first went white */
static void f3_corner(bool white, unsigned next, uint32_t now) {
  if (!white) { run.f3_cand = false; return; }
  if (!run.f3_cand) { run.f3_cand = true; run.f3_cand_odo = run.odo; }
  if (run.odo - run.f3_cand_odo >= F3_CORNER_CONFIRM_MM) { const float from = run.f3_cand_odo; f3_to(next, now); run.f3_odo = from; }
}
/* the up-right leg's heading (its average, else where its pivot ended) */
static float f3_up_hdg(void) { return run.f3_up_len > 30.0f ? run.f3_up_sum / run.f3_up_len : run.f3_ref; }
/* already turned F3_ON_EAST_DEG right of the up-right leg = the follower took the bend itself: on the east leg */
static bool f3_on_east(void) { return run.hdg <= f3_up_hdg() - F3_ON_EAST_DEG; }
static bool f3_advancing(void) { return run.f3_leg == F3_O_VADV || run.f3_leg == F3_O_BADV || run.f3_leg == F3_H_VADV; }
static bool f3_pivoting(void) {
  return run.f3_leg == F3_O_VPIV || run.f3_leg == F3_O_BPIV || run.f3_leg == F3_H_VPIV || run.f3_leg == F3_O_BBACK;
}
static bool f3_leg_live(void) { /* a leg F3 steers / times itself (not the plain line follow, the target or home like F1) */
  return run.f3_base && run.f3_leg != F3_O_BOTTOM && run.f3_leg != F3_O_EAST && run.f3_leg != F3_H_BOTTOM;
}
/* every fresh scan on F3's own legs: the corners (by the tape) and the corner pivots' end (centre white, then black) */
static void f3_scan(uint32_t now, unsigned raw, unsigned filtered) {
  const float d = run.odo - run.f3_odo;
  switch (run.f3_leg) {
    case F3_O_ARM: /* to the V along the U's top arm: the diagonal joins on the left; past the V centre + right go white */
      if (!run.f3_side_seen && run.f3_armed) f3_corner((filtered & (LINE_CENTRE | LINE_RIGHT)) == 0u, F3_O_VADV, now);
      break;                                       /* (backup: the outer-left IR missed the diagonal) */
    case F3_O_UP: /* up-right leg: at its top bend the east leg goes off to the right - the outer-right IR crosses it just past
                   * the bend (backup: centre + left white) */
      if (!run.side_r) run.f3_sr_on = false; /* one high sample is not the east leg: high for F3_SIDE_CONFIRM_MM */
      else if (!run.f3_sr_on) { run.f3_sr_on = true; run.f3_sr_odo = run.odo; }
      if (d >= F3_UP_MIN_MM && run.f3_sr_on && run.odo - run.f3_sr_odo >= F3_SIDE_CONFIRM_MM) {
        const float from = run.f3_sr_odo;
        run.f3_trig_d = from - run.f3_odo; run.f3_trig_kind = 'S';
        if (f3_on_east()) { run.f3_trig_kind = 'E'; f3_to(F3_O_EAST, now); break; }
        f3_to(F3_O_BADV, now); run.f3_odo = from; run.f3_adv = F3_CORNER_ADVANCE_MM - F3_BEND_SIDE_MM;
      } else if (d >= F3_UP_MIN_MM) {
        const float up0 = run.f3_odo;
        f3_corner((filtered & (LINE_LEFT | LINE_CENTRE)) == 0u, F3_O_BADV, now);
        if (run.f3_leg == F3_O_BADV) {
          run.f3_trig_d = run.f3_odo - up0; run.f3_trig_kind = 'W';
          if (f3_on_east()) { run.f3_trig_kind = 'E'; f3_to(F3_O_EAST, now); }
        }
      }
      break;
    case F3_O_BBACK: /* the bend's pivot found no tape: back left onto the up-right leg (the centre white, then on tape) */
      if (!run.f3_cmd) break;
      if ((raw & filtered & LINE_CENTRE) == 0u) run.f3_white = true;
      else if (run.f3_white && f3_deg() >= F3_BBACK_MIN_DEG) {
        if (f3_on_east()) { f3_to(F3_O_EAST, now); break; } /* it was past the bend, on the east leg: that's this tape */
        f3_to(F3_O_UP, now);
        run.f3_odo = run.odo - F3_UP_MIN_MM; /* (where on the leg is unknown: its bend counts at once) */
        run.f3_sr_on = false;
      }
      break;
    case F3_H_DOWN: /* home down the up-right leg: at the V the diagonal crosses the outer-right IR's path right at the V, the
                     * top arm just after (backup: centre + left white) */
      if (d >= F3_DOWN_MIN_MM && run.side_r) { f3_to(F3_H_VADV, now); run.f3_adv = F3_CORNER_ADVANCE_MM + F3_HV_SIDE_MM; }
      else if (d >= F3_DOWN_MIN_MM) f3_corner((filtered & (LINE_LEFT | LINE_CENTRE)) == 0u, F3_H_VADV, now);
      break;
    case F3_O_VPIV: case F3_O_BPIV: case F3_H_VPIV: {
      if (!run.f3_cmd) break; /* only scans after the first pivot command */
      const float lo = run.f3_leg == F3_O_VPIV ? F3_V_MIN_DEG : run.f3_leg == F3_O_BPIV ? F3_B_MIN_DEG : F3_HV_MIN_DEG;
      if ((raw & filtered & LINE_CENTRE) == 0u) run.f3_white = true;
      else if (run.f3_white && f3_deg() >= lo) {
        const unsigned next = run.f3_leg == F3_O_VPIV ? F3_O_UP : run.f3_leg == F3_O_BPIV ? F3_O_EAST : F3_H_ARMU;
        f3_to(next, now);
        run.f3_ref = run.hdg; /* home: the U's turn is measured from here */
      }
      break;
    }
    default: break;
  }
}
/* every tick: the legs that end on distance / heading, and every limit. Non-NULL = stop with that reason */
static const char *f3_tick(uint32_t now) {
  const float d = run.odo - run.f3_odo, turned = run.hdg - run.f3_ref; /* + = turned left (CCW) */
  switch (run.f3_leg) {
    case F3_O_BOTTOM:
      if (run.odo - run.odo_start >= F3_KEEP_FROM_MM) { f3_to(F3_O_U, now); run.f3_ref = run.hdg; }
      break;
    case F3_O_U:
      if (run.f3_u0 < 0.0f && turned <= -20.0f) run.f3_u0 = run.odo; /* the U has started: the leg home ends ~here */
      if (turned <= -120.0f && run.side_l && !run.f3_side_seen) { run.f3_side_seen = true; run.f3_side_odo = run.odo; }
      if (turned <= -F3_U_DONE_DEG) f3_to(F3_O_ARM, now);
      else if (d > F3_U_MAX_MM) return "F3 NO U";
      break;
    case F3_O_ARM:
      /* the outer-left IR touches the diagonal (joining from the left) a fixed way before the V: follow on until the IR bar
       * is at the V, then straight on the rest of F3_V_SIDE_ADVANCE_MM, then the pivot */
      if (run.side_l && !run.f3_side_seen) { run.f3_side_seen = true; run.f3_side_odo = run.odo; }
      if (run.f3_side_seen && run.odo - run.f3_side_odo >= F3_V_SIDE_ADVANCE_MM - F3_CORNER_ADVANCE_MM) {
        const float left = F3_V_SIDE_ADVANCE_MM - (run.odo - run.f3_side_odo);
        f3_to(F3_O_VADV, now);
        run.f3_adv = left;
        break;
      }
      if (d >= F3_ARM_MM) run.f3_armed = true; /* backup: the outer-left IR missed the diagonal */
      if (d > F3_LEG_MAX_MM) return "F3 NO V";
      break;
    case F3_O_UP:
      if (!run.f3_retried && d >= 40.0f && d <= 140.0f) { /* settled on the leg, before its bend */
        if (run.f3_up_odo > 0.0f) { run.f3_up_sum += run.hdg * (run.odo - run.f3_up_odo); run.f3_up_len += run.odo - run.f3_up_odo; }
        run.f3_up_odo = run.odo;
      }
      /* the follower took the bend itself (a rounded corner, or both bend triggers missed): turned this far right = on the
       * east leg already. Held 15 mm, so one swing of the follower is not it */
      if (d >= F3_UP_MIN_MM && f3_on_east()) {
        if (!run.f3_cand) { run.f3_cand = true; run.f3_cand_odo = run.odo; }
        else if (run.odo - run.f3_cand_odo >= 15.0f) { run.f3_trig_d = d; run.f3_trig_kind = 'E'; f3_to(F3_O_EAST, now); break; }
      }
      if (d > F3_LEG_MAX_MM) return "F3 NO BEND";
      break;
    case F3_O_BBACK:
      if (f3_deg() > F3_BBACK_MAX_DEG) return "F3 BEND NO TAPE";
      if (now - run.f3_at >= F2_T_TURN_MAX_MS) return "F3 TURN TIMEOUT";
      break;
    case F3_H_DOWN: if (d > F3_LEG_MAX_MM) return "HOME NO V"; break;
    case F3_O_VADV: case F3_O_BADV: case F3_H_VADV:
      if (d >= run.f3_adv) f3_to(run.f3_leg + 1u, now); /* the pivot follows each advance */
      else if (now - run.f3_at >= F2_T_ADVANCE_MAX_MS) return "F3 ADV TIMEOUT";
      break;
    case F3_O_VPIV: case F3_O_BPIV: case F3_H_VPIV: {
      const float hi = run.f3_leg == F3_O_VPIV ? F3_V_MAX_DEG : run.f3_leg == F3_O_BPIV ? F3_B_MAX_DEG : F3_HV_MAX_DEG;
      if (run.f3_leg == F3_O_BPIV) run.f3_bend_deg = f3_deg();
      if (run.f3_leg == F3_O_BPIV && f3_deg() > hi && !run.f3_retried) { run.f3_retried = true; f3_to(F3_O_BBACK, now); break; }
      if (f3_deg() > hi) return run.f3_leg == F3_O_VPIV ? "F3 V NO TAPE" : run.f3_leg == F3_O_BPIV ? "F3 BEND NO TAPE" : "HOME V NO TAPE";
      if (now - run.f3_at >= F2_T_TURN_MAX_MS) return "F3 TURN TIMEOUT";
      break;
    }
    case F3_H_ARMU:
      if (d >= F3_ARM_AVG_FROM_MM && d <= F3_ARM_AVG_TO_MM) { /* settled on the top arm, before the U */
        if (run.f3_arm_odo > 0.0f) { run.f3_arm_sum += run.hdg * (run.odo - run.f3_arm_odo); run.f3_arm_len += run.odo - run.f3_arm_odo; }
        run.f3_arm_odo = run.odo;
      }
      if (turned >= F3_HU_DONE_DEG) { /* round the U, on the bottom line: home like F1 from here */
        run.f3_leg = F3_H_BOTTOM;
        run.home_mark = run.odo;
        run.home_exp = (run.f3_u0 > 0.0f ? run.f3_u0 : run.odo_start + 500.0f) - run.odo_start;
        /* the bottom line's direction: the top arm's + 180 (the U turns left 180 between them). Held, not averaged: the AR
         * leaves the U up to ~20 deg off the line and an average would lag it into the zone at an angle */
        if (run.f3_arm_len > 50.0f) { run.line_hdg = run.f3_arm_sum / run.f3_arm_len + 180.0f; run.f3_hdg_set = true; }
        else run.line_hdg = run.hdg; /* (arm too short to average: from here, averaged as F1) */
        run.hdg_odo = run.odo;
        run.stall_odo = run.odo; run.stall_at = now;
      } else if (d > F3_U_MAX_MM) return "HOME NO U"; /* top arm ~330 + the U ~630 */
      break;
    default: break;
  }
  return NULL;
}
/* ---- F4: the bottom-left key (function1.h F4_*) ---- */
static bool f4_pivot_leg(unsigned leg) {
  return leg == F4_O_P1 || leg == F4_O_P2 || leg == F4_O_P3 || leg == F4_O_P4 || leg == F4_O_P5 ||
         leg == F4_H_P1 || leg == F4_H_P2 || leg == F4_H_P3 || leg == F4_H_P4 || leg == F4_H_P5 || leg == F4_O_CAL;
}
static bool f4_leg_live(void) { /* a leg F4 steers itself (not the line out to the stone, not F1's line home) */
  return run.f4_base && run.f4_leg != F4_O_LINE && run.f4_leg != F4_H_LINE;
}
bool F1_F4OffLine(void) { return f4_leg_live(); }
static void f4_to(unsigned leg, uint32_t now) {
  /* every leg starts from a standstill (F4_PAUSE_MS stopped first): the encoders count a wheel still rolling the old way as
   * turning the new way, ~1 deg per pivot otherwise; the roll-on after a straight is measured and taken off the next */
  run.f4_was_straight = !f4_pivot_leg(run.f4_leg) && run.f4_leg != F4_O_SETTLE;
  run.f4_done = run.odo - run.f4_odo; run.f4_done_leg = run.f4_leg;
  run.f4_pause = leg != F4_O_SETTLE && leg != F4_H_LINE;
  run.f4_leg = leg; run.f4_odo = run.odo; run.f4_at = now; run.f4_from = run.hdg;
  run.f4_cmd = run.f4_white = run.f4_fixed = false;
  run.f4_blk = true; /* tape already under the IRs as the leg starts is not a crossing */
  run.f4_xodo = -1e9f; run.f4_stall_odo = run.odo; run.f4_stall_at = now;
  run.f4_prev = ALL_BLACK; run.f4_rise[0] = run.f4_rise[1] = run.f4_rise[2] = -1e9f;
  switch (leg) { /* + = left (counter-clockwise); H0 = west, H0 - 90 = north, H0 + 90 = south */
    /* turns in encoder degrees: the true angle / the scale measured on the bottom line (F4_O_CAL) */
    case F4_O_P1: case F4_H_P4: run.f4_aim -= 90.0f / run.f4_k; break;
    case F4_O_P2: case F4_O_P5: case F4_H_P1: case F4_H_P5: run.f4_aim += 90.0f / run.f4_k; break; /* (H_P5 ends on the line) */
    case F4_O_P3: case F4_H_P2: run.f4_aim -= F4_GAP_DEG / run.f4_k; break;
    case F4_O_P4: case F4_H_P3: run.f4_aim += F4_GAP_DEG / run.f4_k; break;
    case F4_O_CAL: run.f4_cal_n = 0u; break;
    case F4_O_S1: run.f4_len = F4_NORTH_MM; break;
    case F4_O_S2: run.f4_len = run.f4_s2 + F4_W_EXTRA_MM; break; /* US1 at the barrier's face ends it (f4_ping) */
    case F4_O_S3: run.f4_len = F4_GAP_MM; break;
    /* home: each straight retraces its leg out as driven (the same encoders: a wheel-size error cancels) */
    case F4_H_S1: run.f4_len = run.f4_s4_act; break;
    case F4_H_S2: run.f4_len = run.f4_s3_act; break;
    case F4_H_S3: run.f4_len = run.f4_s2_act; break;
    case F4_O_S4: run.f4_len = F4_S4_MAX_MM; break;
    case F4_O_KEY: run.f4_len = F4_KEY_MAX_MM; run.f4_key_odo = run.odo; break;
    case F4_H_S0: run.f4_len = (run.odo_out - run.f4_key_odo) - run.back_done; break; /* reversing: back to where it turned */
    case F4_H_S4: run.f4_len = F4_H4_MAX_MM; break;
    default: break;
  }
}
static float f4_h0(void) { return run.f4_h0_len > 100.0f ? run.f4_h0_sum / run.f4_h0_len : run.f4_aim; }
/* every valid US1 reading on F4's own straights (both ways): something this near ahead = stop, do not drive into it */
static void f4_guard(uint32_t mm) {
  const unsigned leg = run.f4_leg;
  const bool straight = leg == F4_O_S1 || leg == F4_O_S2 || leg == F4_O_S3 || leg == F4_O_S4 || leg == F4_H_S1 ||
                        leg == F4_H_S2 || leg == F4_H_S3 || leg == F4_H_S4;
  if (straight && !run.f4_pause && (float)mm < F4_US_BLOCK_MM) run.f4_blocked = true;
}
/* US1 on the line out: the F1 stone */
static void f4_ping(uint32_t mm) {
  if (run.f4_leg == F4_O_S2 && !run.f4_pause) { /* W: US1 square on the barrier's face, the gap's south-west side */
    const float d = run.odo - run.f4_odo;
    if (d >= F4_W_FROM_MM && (float)mm <= F4_W_WALL_MM + F4_W_WINDOW_MM) {
      run.f4_len = d + ((float)mm - F4_W_WALL_MM); run.f4_wall = true;
    }
    return;
  }
  if (run.f4_leg != F4_O_LINE || run.approaching || run.odo - run.odo_start < F4_STONE_MIN_MM ||
      (float)mm > F4_STONE_STOP_MM) return;
  run.f4_us = (float)mm; run.f4_det_odo = run.odo;
  run.f4_aim = f4_h0();
  run.f4_drift = (run.hdg - run.f4_aim) * run.f4_k; /* its heading off the line's at the stone (screen: should be small) */
  f4_to(F4_O_SETTLE, run.f4_at);
  run.f4_at = 0u; /* timed from the first stop command */
  run.f4_cmd = false;
}
static void f4_scan(uint32_t now, unsigned raw, unsigned filtered) {
  const unsigned leg = run.f4_leg;
  if (run.f4_pause) return; /* stopped between legs */
  if (leg == F4_O_CAL) {
    /* a full turn right on the spot, the centre IR's edges: leaves the line (0), crosses it behind (in 1, out 2), finds it
     * in front (3). 0 -> 3 is 360 less the tape's width, 1 -> 2 is the tape's width: together exactly 360, whatever the
     * axle's offset from the line (it moves the crossing behind, not the sum) and the IRs' filter delay (it cancels) */
    if (!run.f4_cmd || run.f4_cal_n >= 4u) return;
    const bool c = (filtered & LINE_CENTRE) != 0u, want_black = run.f4_cal_n == 1u || run.f4_cal_n == 3u;
    const float turned = run.f4_cal_n ? run.f4_cal_a[0] - run.hdg : 0.0f; /* right = the heading goes down */
    if (c != want_black) return;
    if (run.f4_cal_n == 1u && turned < F4_CAL_BEHIND_MIN_DEG) return; /* not the line behind yet */
    if (run.f4_cal_n == 3u && turned < F4_CAL_FRONT_MIN_DEG) return;
    run.f4_cal_a[run.f4_cal_n++] = run.hdg;
    if (run.f4_cal_n < 4u) return;
    const float raw360 = (run.f4_cal_a[0] - run.f4_cal_a[3]) + (run.f4_cal_a[1] - run.f4_cal_a[2]);
    const float k = raw360 > 1.0f ? 360.0f / raw360 : 0.0f;
    run.f4_k_seen = run.f4_k = k;
    if (k < F4_CAL_K_MIN || k > F4_CAL_K_MAX || run.f4_cal_a[1] - run.f4_cal_a[2] > F4_CAL_TAPE_MAX_DEG) {
      run.f4_cal_bad = true; return;
    }
    /* within the measurement's own scatter (~1 %; the turns tolerate ~1.5 %): keep the encoders' own scale */
    if (k > 1.0f - F4_CAL_DEADBAND && k < 1.0f + F4_CAL_DEADBAND) run.f4_k = 1.0f;
    /* on the line again, facing out: follow it on; the line's heading is averaged from here (F4_H0_AFTER_CAL_MM on) */
    run.f4_cal_done = true;
    run.odo_start += run.odo - run.f4_odo; /* the turn's wheel travel is not distance out */
    run.f4_cal_odo = run.odo; run.f4_h0_odo = 0.0f;
    run.f4_leg = F4_O_LINE; run.f4_pause = false;
    run.stall_odo = run.odo; run.stall_at = now;
    return;
  }
  if (leg == F4_O_S4 || leg == F4_H_S1 || leg == F4_H_S4) { /* a tape square across */
    const unsigned n = ((filtered & LINE_LEFT) != 0u) + ((filtered & LINE_CENTRE) != 0u) + ((filtered & LINE_RIGHT) != 0u);
    /* across the 1 / the 8: two IRs went black together (one may ride the 8's top edge or middle bar, black all along);
     * the bottom line: all three black at once (the C crosses it at a slant just before: never all three) */
    unsigned rose = 0u;
    for (unsigned i = 0; i < 3u; i++) {
      const unsigned bit = 1u << i;
      if ((filtered & bit) && !(run.f4_prev & bit)) run.f4_rise[i] = run.odo;
      if ((filtered & bit) && run.odo - run.f4_rise[i] <= F4_RISE_SPAN_MM) rose++;
    }
    run.f4_prev = filtered;
    const bool p = leg == F4_H_S4 ? filtered == ALL_BLACK || (n >= 2u && run.side_l && run.side_r) : rose >= 2u;
    if (p && !run.f4_blk && run.odo - run.f4_xodo >= F4_CROSS_GAP_MM) {
      const float d = run.odo - run.f4_odo;
      run.f4_xodo = run.odo;
      if (leg == F4_O_S4) {
        run.f4_xo++;
        if (!run.f4_fixed && d >= F4_S4_EDGE_FROM_MM) { run.f4_fixed = true; run.f4_len = d + F4_EDGE_ADV_MM; }
      } else if (leg == F4_H_S1) {
        run.f4_xh++; /* (the screen only: home retraces the leg out) */
      } else if (!run.f4_fixed && d >= F4_H4_FROM_MM) { run.f4_fixed = true; run.f4_len = d + F4_EDGE_ADV_MM; }
    }
    run.f4_blk = p;
  } else if (leg == F4_H_P5 && run.f4_cmd) { /* onto the bottom line: the centre white, then black inside the window */
    if ((raw & filtered & LINE_CENTRE) == 0u) run.f4_white = true;
    else if (run.f4_white && run.hdg - run.f4_from >= F4_P5_MIN_DEG) { /* home like F1 from here */
      run.f4_leg = F4_H_LINE;
      run.line_hdg = run.hdg; run.hdg_odo = run.odo;
      run.home_mark = run.odo; run.home_exp = run.f4_det_odo - run.odo_start; /* the stop at the stone was this far out */
      run.stall_odo = run.odo; run.stall_at = now;
    }
  }
}
static const char *f4_tick(uint32_t now) {
  const unsigned leg = run.f4_leg;
  if (run.f4_pause && leg != F4_O_LINE && leg != F4_H_LINE) {
    if (now - run.f4_at < F4_PAUSE_MS) return NULL;
    if (run.f4_was_straight) {
      run.f4_coast = run.odo - run.f4_odo;
      const float act = run.f4_done + run.f4_coast;
      if (run.f4_done_leg == F4_O_S2) run.f4_s2_act = act;
      else if (run.f4_done_leg == F4_O_S3) run.f4_s3_act = act;
      else if (run.f4_done_leg == F4_O_S4) run.f4_s4_act = act;
    }
    run.f4_pause = false; /* the leg starts now, from here */
    run.f4_odo = run.odo; run.f4_at = now; run.f4_from = run.hdg; run.f4_stall_odo = run.odo; run.f4_stall_at = now;
    return NULL;
  }
  const float d = run.odo - run.f4_odo;
  if (leg == F4_O_LINE) {
    /* the bottom line's heading, distance-weighted, only while the centre IR alone is on it (not where a line joins, not
     * while steering back to it) */
    const float out = run.odo - run.odo_start;
    if (run.filtered == ALL_BLACK && out < F4_OUTLINE_MAX_MM) run.f4_outline = out; /* the start zone's outline, crossed */
    const float cal_at = run.f4_outline + F4_CAL_PAST_OUTLINE_MM > F4_CAL_AT_MM ? run.f4_outline + F4_CAL_PAST_OUTLINE_MM
                                                                                : F4_CAL_AT_MM;
    if (!run.f4_cal_done && out >= cal_at && run.filtered == LINE_CENTRE) {
      f4_to(F4_O_CAL, now); /* the turn calibration, on a clear stretch of the bottom line */
      return NULL;
    }
    if (run.f4_cal_done && run.odo - run.f4_cal_odo >= F4_H0_AFTER_CAL_MM && run.filtered == LINE_CENTRE &&
        run.f4_h0_odo > 0.0f) {
      run.f4_h0_sum += run.hdg * (run.odo - run.f4_h0_odo); run.f4_h0_len += run.odo - run.f4_h0_odo;
    }
    run.f4_h0_odo = run.odo;
    /* off the bottom line's heading: it went onto a curve - stop, do not do the legs from the wrong place */
    if (run.f4_h0_len > 100.0f && (run.hdg - f4_h0() > F4_LINE_OFF_DEG || f4_h0() - run.hdg > F4_LINE_OFF_DEG))
      return "F4 OFF LINE";
    return run.odo - run.odo_start > F4_STONE_MAX_MM ? "F4 NO STONE" : NULL;
  }
  if (leg == F4_H_LINE) return NULL;
  if (leg == F4_O_SETTLE) {
    if (run.f4_cmd && now - run.f4_at >= F4_SETTLE_MS) {
      /* the west leg: from where US1 saw the stone to W, less the coast since */
      run.f4_s2 = F4_W_PAST_STONE_MM + run.f4_us + F4_US_AHEAD_MM - (run.odo - run.f4_det_odo);
      run.f4_coast = run.odo - run.f4_odo; /* the first estimate: the roll-on from full speed */
      f4_to(F4_O_P1, now);
    }
    return NULL;
  }
  if (run.f4_blocked) return "F4 US1 BLOCKED";
  if (leg == F4_O_KEY && run.approaching) return NULL; /* F1's approach and hold from here */
  /* a dead encoder / against something: the wheels driven, no distance */
  if (run.odo - run.f4_stall_odo >= 5.0f) { run.f4_stall_odo = run.odo; run.f4_stall_at = now; }
  else if (now - run.f4_stall_at >= F4_STALL_MS) {
    if (leg == F4_O_KEY && d >= F4_KEY_STALL_MIN_MM) { /* against the key without US1 seeing it: at the key, hold */
      run.approaching = true; run.near_at = now - US_APPROACH_MS;
      return NULL;
    }
    return "F4 STALLED";
  }
  if (leg == F4_O_CAL) {
    if (run.f4_cal_bad) return "F4 CAL BAD";
    if (run.f4_from - run.hdg > F4_CAL_MAX_DEG) return "F4 CAL NO LINE";
    if (now - run.f4_at >= F4_CAL_MAX_MS) return "F4 TURN TIMEOUT";
    return NULL;
  }
  if (f4_pivot_leg(leg)) {
    if (leg == F4_H_P5) {
      if (run.hdg - run.f4_from > F4_P5_MAX_DEG) return "F4 HOME NO LINE";
    } else {
      const float left = run.f4_aim - run.hdg; /* still to turn (+ = left) */
      if (run.f4_aim >= run.f4_from ? left <= F4_PIVOT_LEAD_DEG : left >= -F4_PIVOT_LEAD_DEG) { f4_to(leg + 1u, now); return NULL; }
    }
    if (now - run.f4_at >= F2_T_TURN_MAX_MS) return "F4 TURN TIMEOUT";
    return NULL;
  }
  if (d + run.f4_coast >= run.f4_len) { /* it rolls on ~f4_coast while stopping */
    if (leg == F4_O_S4 && !run.f4_fixed) return "F4 NO 8";
    if (leg == F4_H_S4 && !run.f4_fixed) return "F4 HOME NO LINE";
    if (leg == F4_O_KEY) return "F4 NO KEY";
    f4_to(leg + 1u, now);
    return NULL;
  }
  if (now - run.f4_at >= 2000u + (uint32_t)(10.0f * run.f4_len)) return "F4 LEG TIMEOUT";
  return NULL;
}
static F1_Command move(float base_left, float base_right, const char *label);
static F1_Command f4_command(uint32_t now) {
  static const char *const name[] = {"F4: LINE", "F4: AT STONE", "F4: TURN NORTH", "F4: NORTH", "F4: TURN WEST",
    "F4: WEST", "F4: TURN TO GAP", "F4: THRU GAP", "F4: TURN WEST 2", "F4: TO THE 8", "F4: TURN SOUTH", "F4: TO KEY",
    "HOME: BACK NORTH", "HOME: TURN EAST", "HOME: EAST", "HOME: TO GAP", "HOME: THRU GAP", "HOME: TURN EAST2",
    "HOME: EAST 2", "HOME: TURN SOUTH", "HOME: TO LINE", "HOME: FIND LINE", "HOME", "F4: CALIBRATE"};
  const unsigned leg = run.f4_leg;
  const char *label = leg < sizeof name / sizeof name[0] ? name[leg] : "F4";
  if (leg == F4_O_SETTLE) {
    if (!run.f4_cmd) { run.f4_cmd = true; run.f4_at = now; run.f4_odo = run.odo; }
    return move(0.0f, 0.0f, label);
  }
  if (run.f4_pause) return move(0.0f, 0.0f, label);
  if (leg == F4_O_CAL) { run.f4_cmd = true; return move(US_PIVOT_PERCENT, -US_PIVOT_PERCENT, label); } /* right */
  if (f4_pivot_leg(leg)) {
    run.f4_cmd = true; /* only scans after the first pivot command count */
    return run.f4_aim >= run.f4_from ? move(-US_PIVOT_PERCENT, US_PIVOT_PERCENT, label)
                                     : move(US_PIVOT_PERCENT, -US_PIVOT_PERCENT, label);
  }
  /* a straight leg: hold the aim (as F1 into the zone) */
  float e = run.f4_aim - run.hdg; /* + = must turn counter-clockwise (left) */
  float dd = F1_STRAIGHT_K * e;
  dd = dd > 8.0f ? 8.0f : (dd < -8.0f ? -8.0f : dd);
  if (leg == F4_H_S0) return move(-LINE_NOMINAL_PERCENT - dd, -LINE_NOMINAL_PERCENT + dd, label); /* reversing */
  return move(LINE_NOMINAL_PERCENT - dd, LINE_NOMINAL_PERCENT + dd, run.approaching ? "TARGET APPROACH" : label);
}
/* steering on F3's own legs: hold to one side of the tape where another line joins or the route forks (the near IR on the
 * tape steers toward it, the far one is ignored while the centre still has the line); sharp = round the U */
static F1_Command move(float base_left, float base_right, const char *label);
static F1_Command f3_keep(bool right, bool sharp, uint32_t now, const char *label);
#else
void F1_Heading(float deg) { (void)deg; }
void F1_SideLevels(bool left_high, bool right_high) { (void)left_high; (void)right_high; }
void F1_Odometry(float odo_mm, float arc_mm) { (void)odo_mm; (void)arc_mm; }
float F1_PivotDeg(void) { return 0.0f; }
void F1_BranchTurning(bool turning) { (void)turning; }
bool F1_TurnsHome(void) { return false; }
#endif

/* A level observation is idempotent. Keep the actual filtered mask intact for
 * the TFT, return marker and finish; this latch changes the heading only. */
static void observe_split(unsigned filtered, uint32_t now) {
  if (run.f2_base) return;
#if F1_LEGACY
  (void)now;
  if (filtered == (LINE_LEFT | LINE_RIGHT)) run.split_left = true;
  else if (filtered == LINE_CENTRE) run.split_left = false;
#else
  if (run.f3_base) return; /* F3 keeps right / left itself where the route forks */
  if (filtered == (LINE_LEFT | LINE_RIGHT)) {
    if (!run.split_left) run.split_at = now;
    run.split_left = true;
  } else if ((filtered & LINE_CENTRE) != 0u) {
    run.split_left = false; /* the centre is back on the line (010 / 110 / 011 / 111): release */
  }
  if (run.split_left && (uint32_t)(now - run.split_at) >= F1_SPLIT_MAX_MS) run.split_left = false;
#endif
}

#if !F1_LEGACY
/* where the line went, on EVERY follow scan and command: remembered so a lost line (000) is searched for on the side
 * it slipped out to */
static void observe_side(unsigned tape, uint32_t now) {
  if (tape == LINE_LEFT || tape == (LINE_LEFT | LINE_CENTRE)) run.last_side = -1;
  else if (tape == LINE_RIGHT || tape == (LINE_RIGHT | LINE_CENTRE)) run.last_side = +1;
  else if (tape == LINE_CENTRE || tape == (LINE_LEFT | LINE_CENTRE | LINE_RIGHT)) run.last_side = 0;
  if (tape == 0u && !run.lost) run.lost_at = now;
  run.lost = tape == 0u;
}
#endif

void F1_Scan(uint32_t started, uint32_t completed, unsigned raw, unsigned filtered) {
  if (run.phase == F1_IDLE) return;
  if (run.phase == F1_RETURN) {
    /* First exclude pre-return scans, then compare against the preceding
     * accepted scan, not an entry timestamp on a now unbounded leg. */
    if ((uint32_t)(started - run.return_scan_from) > UINT32_MAX / 2u) return;
    run.return_scan_from = started;
#if !F1_LEGACY
    if (!run.turn_home) {
      if (filtered == ALL_BLACK) run.return_marker = true;
    } else if (run.f2_base && run.t_state == 0u) {
      /* F2 home, down the vertical line: the bottom line (the T) = 2+ analog IRs AND an outer IR, or all three analog -
       * only near where it is expected (nothing else crosses this line on the way down) */
      const unsigned n = ((filtered & LINE_LEFT) != 0u) + ((filtered & LINE_CENTRE) != 0u) + ((filtered & LINE_RIGHT) != 0u);
      if (home_done() >= t_expect() - F2_T_EARLY_MM && (filtered == ALL_BLACK || (n >= 2u && (run.side_l || run.side_r)))) {
        run.t_state = 1u; run.t_odo = run.odo; run.t_at = started; /* straight on past it first */
        run.t_vhdg = run.line_hdg; /* the vertical line's direction (averaged while following it down) */
      }
    } else if (run.f2_base && run.t_state == 1u) {
      /* advancing past the T: nothing to read */
    } else if (run.f2_base && run.t_state == 3u) {
      /* pivoting left onto the bottom line: the centre white, then black inside the angle window */
      if (run.t_cmd) {
        if ((raw & filtered & LINE_CENTRE) == 0u) run.t_white = true;
        else if (run.t_white && t_deg() >= F2_T_TURN_MIN_DEG) {
          /* on the bottom line, heading home. The leg home = the start -> T leg out, corrected by how far the T came
           * before / after where expected (the same IR-to-axle offset, opposite sign, in both) */
          const float t_actual = run.t_odo - run.home_mark;
          run.home_exp = (run.odo_b0 - run.odo_start) + (t_expect() - t_actual);
          run.home_mark = run.odo;
          run.line_hdg = run.t_vhdg + 90.0f; /* the bottom line: 90 deg left of the vertical line (the map), not the heading
                                               * at pickup (which is up to ~30 deg short of it) */
          run.hdg_odo = run.odo;
          run.t_state = 2u;
        }
      }
    } else if (run.f3_base && run.f3_leg != F3_H_BOTTOM) {
      f3_scan(started, raw, filtered); /* F3 home: down to the V, its pivot, round the U */
    } else if (run.f4_base && run.f4_leg != F4_H_LINE) {
      f4_scan(started, raw, filtered); /* F4 home: the legs back to the bottom line, the pivot onto it */
    } else if (!run.in_zone && home_done() >= home_expect() - F1_HOME_EARLY_MM) {
      /* the start-zone outline, near home only. The IRs need not trip TOGETHER (an angle, unequal knobs): each must have
       * been black within F1_OUTLINE_SPAN_MM. While checking, it goes straight (no steering on the outline). */
      const bool b[5] = {(filtered & LINE_LEFT) != 0u, (filtered & LINE_CENTRE) != 0u, (filtered & LINE_RIGHT) != 0u,
                         run.side_l, run.side_r};
      unsigned nb = 0u;
      for (unsigned i = 0; i < 5u; i++) { if (b[i]) { run.seen_odo[i] = run.odo; nb++; } }
      /* an OUTER IR black: the 20 mm line cannot reach them (60 mm out) while following - something wide in front */
      if (!run.edge_hold && (run.side_l || run.side_r)) { run.edge_hold = true; run.edge_odo = run.odo; }
      if (run.edge_hold) {
        /* the three analog IRs AND at least one outer IR, each black within the span (one outer IR may sit higher and
         * miss it) */
        bool all = true;
        for (unsigned i = 0; i < 3u; i++) if (run.odo - run.seen_odo[i] > F1_OUTLINE_SPAN_MM) all = false;
        if (F1_OUTLINE_ALL5 && run.odo - run.seen_odo[3] > F1_OUTLINE_SPAN_MM && run.odo - run.seen_odo[4] > F1_OUTLINE_SPAN_MM)
          all = false;
        if (all) { /* straight into the zone (first shifted F1_ZONE_SHIFT_MM left) */
          run.return_marker = true; run.in_zone = true; run.edge_hold = false;
          run.zone_lat = 0.0f; run.zone_odo = run.odo;
        }
        else if (run.odo - run.edge_odo > F1_OUTLINE_SPAN_MM && !run.side_l && !run.side_r) run.edge_hold = false; /* not it */
      }
    }
#else
    if (filtered == ALL_BLACK) run.return_marker = true;
#endif
  }
  run.filtered = filtered;
  if (on_outbound() && run.run_armed && filtered == ALL_BLACK) {
    enter(F1_FINISH_REVERSE, completed);
  } else if (run.phase == F1_PIVOT) {
    /* A scan that started before the pivot (or at/after its deadline) cannot
     * clear or acquire the tape. Use the raw AND filtered bit of the SAME sensor. */
    uint32_t age = started - run.phase_at;
    if (age >= US_SPIN_LIMIT_MS) return;
#if !F1_LEGACY
    if (run.turn_home) { /* by ANGLE: the centre IR on tape inside the window ends the turn at the line */
      if ((raw & filtered & LINE_CENTRE) == 0u) run.saw_clear = true;
      else if (run.saw_clear && F1_PivotDeg() >= (run.f3_base ? F3_PIVOT_MIN_DEG : F1_PIVOT_MIN_DEG)) run.pickup_pending = true;
      return;
    }
#endif
    if ((raw | filtered) == 0u) run.saw_clear = true;
    else if (run.saw_clear && age >= US_SPIN_MIN_MS && (raw & filtered) != 0u)
      run.pickup_pending = true;
  } else if (run.approaching || run.phase == F1_RETURN || base_phase_at(completed) == F1_OUTBOUND) {
    observe_split(filtered, completed); /* consume every follow scan, even between two wheel writes */
#if !F1_LEGACY
    observe_side(filtered, completed);
    if (run.f3_base && on_outbound() && !run.approaching) f3_scan(completed, raw, filtered);
    if (run.f4_base && on_outbound() && !run.approaching) f4_scan(completed, raw, filtered);
#endif
  }
}
/* Only the marker is latched. White and distance must agree NOW; an old white
 * gap or a previous close ping is not evidence for the current decision. */
#if !F1_LEGACY
static float home_expect(void);
static float home_done(void);
#endif
static bool return_end(uint32_t fresh_us1_mm) {
#if !F1_LEGACY
  /* turning home: the zone's wall near home finishes it even if the outline was not confirmed (e.g. an outer IR misses it) */
  if (run.turn_home && run.phase == F1_RETURN && fresh_us1_mm < DRIVE_US1_FINISH_MM &&
      (run.in_zone || home_done() >= home_expect() - F1_HOME_EARLY_MM)) return true;
#endif
  return run.phase == F1_RETURN && run.return_marker && run.filtered == 0u &&
         fresh_us1_mm < DRIVE_US1_FINISH_MM;
}
#if !F1_LEGACY
static bool turn_home_returning(void) { return run.turn_home && run.phase == F1_RETURN; }
#endif
bool F1_TargetPing(uint32_t observed_at, uint32_t valid_us1_mm) {
#if !F1_LEGACY
  if (run.f4_base && f4_leg_live()) f4_guard(valid_us1_mm); /* F4: something right ahead on a straight */
  if (turn_home_returning()) return false; /* going home: the US only finishes it (the zone's wall), no second target */
  if (run.f3_base && run.f3_leg != F3_O_EAST) return false; /* F3: the key is at the end of the east leg (a box stands by the bend) */
  if (run.f4_base && on_outbound()) f4_ping(valid_us1_mm); /* F4: the F1 stone on the line out */
  if (run.f4_base && run.f4_leg != F4_O_KEY) return false; /* F4: the key only on the leg south to it */
#endif
  if ((!on_outbound() && run.phase != F1_RETURN) || run.approaching ||
      valid_us1_mm >= DRIVE_US1_STOP_MM || return_end(valid_us1_mm)) return false;
  run.approaching = true;
  run.near_at = observed_at; /* later pings neither cancel nor extend this approach */
  return true;
}
const char *F1_Advance(uint32_t now) {
  /* Complete launch once; a long-running outbound leg must not relaunch at tick wrap. */
  run.phase = base_phase_at(now);
  const uint32_t elapsed = now - run.phase_at;
#if !F1_LEGACY
  if (run.f3_base && !run.approaching && (run.phase == F1_OUTBOUND || run.phase == F1_RETURN)) {
    const char *why = f3_tick(now);
    if (why) return finish(why);
  }
  if (run.f4_base && (run.phase == F1_OUTBOUND || run.phase == F1_RETURN)) {
    const char *why = f4_tick(now);
    if (why) return finish(why);
  }
#endif
  switch (run.phase) {
    case F1_TARGET_HOLD:
#if F1_LEGACY
      if (elapsed >= US_STOP_HOLD_MS) enter(F1_TARGET_REVERSE, now);
#else
      if (elapsed >= US_STOP_HOLD_MS) enter(run.turn_home ? F1_TARGET_REVERSE : F1_RETRACE, now); /* turn home / retrace */
#endif
      break;
    case F1_TARGET_REVERSE:
#if !F1_LEGACY
      if (run.turn_home) {
        if (run.odo - run.odo_mark >= (run.f3_base ? F3_BACK_MM : run.f4_base ? F4_BACK_MM : F1_BACK_MM) || elapsed >= 2000u) {
          run.back_done = run.odo - run.odo_mark;
          if (run.f4_base) { /* F4: home on its own legs (the line is far off), F1's home from the bottom line */
            enter(F1_RETURN, now);
            run.return_marker = false;
            run.in_zone = run.edge_hold = false;
            for (unsigned i = 0; i < 5u; i++) run.seen_odo[i] = -1e9f;
            run.line_hdg = run.hdg; run.home_mark = run.odo; run.home_exp = 1e9f;
            run.stall_odo = run.odo; run.stall_at = now;
            run.return_scan_from = now; run.return_at = now;
            f4_to(F4_H_S0, now);
            break;
          }
          enter(F1_PIVOT, now);
          run.saw_clear = run.pickup_pending = false;
        }
        break;
      }
#endif
      if (elapsed >= US_REVERSE_MS) {
        enter(F1_PIVOT, now);
        run.saw_clear = run.pickup_pending = false;
      }
      break;
    case F1_PIVOT:
#if !F1_LEGACY
      if (run.turn_home) last_pivot_deg = F1_PivotDeg();
#endif
      if (elapsed >= US_SPIN_LIMIT_MS) return finish("LINE NOT FOUND");
#if !F1_LEGACY
      if (run.turn_home) {
        if (run.pickup_pending) { /* on the line: straight into the return (no extra turning) */
          enter(F1_RETURN, now);
          run.return_marker = false;
          run.in_zone = run.edge_hold = false;
          for (unsigned i = 0; i < 5u; i++) run.seen_odo[i] = -1e9f;
          run.line_hdg = run.hdg; /* the line's direction: starts at the heading it picked the line up with */
          run.home_mark = run.odo;
          /* F1: the line home is the way it came. F2: first the T (left turn), then the leg home is set there */
          run.home_exp = run.f2_base ? 1e9f : (run.odo_out - run.odo_start) - run.back_done;
          run.t_state = 0u;
          if (run.f3_base) { f3_to(F3_H_DOWN, now); run.home_exp = 1e9f; } /* F3: the leg home is set past the U */
          run.stall_odo = run.odo; run.stall_at = now;
          run.return_scan_from = now; run.return_at = now;
        } else if (F1_PivotDeg() > (run.f3_base ? F3_PIVOT_MAX_DEG : F1_PIVOT_MAX_DEG)) {
          return finish("TURN NO LINE");
        }
        break;
      }
#endif
      /* Start alignment after the poll, not at the start of its IR scan. */
      if (run.pickup_pending) enter(F1_ALIGN, now);
      break;
#if !F1_LEGACY
    case F1_RETURN:
      if (run.turn_home && home_done() >= home_expect() + F1_HOME_LATE_MM) return finish("HOME BY DIST");
      if (run.turn_home && run.f2_base && run.t_state == 0u && home_done() > t_expect() + F2_T_LATE_MM)
        return finish("HOME NO JUNCTION");
      if (run.turn_home && run.f2_base && run.t_state == 1u && run.odo - run.t_odo >= F2_T_ADVANCE_MM) {
        run.t_state = 3u; run.t_arc0 = run.arc; run.t_cmd = run.t_white = false; run.t_at = now; /* now pivot left */
      }
      if (run.turn_home && run.f2_base && run.t_state == 3u && t_deg() > F2_T_TURN_MAX_DEG) return finish("HOME T NO TAPE");
      /* time backups: these moves end on encoder distance / angle, which a dead encoder never reaches */
      if (run.turn_home && run.f2_base && ((run.t_state == 1u && now - run.t_at >= F2_T_ADVANCE_MAX_MS) ||
                                           (run.t_state == 3u && now - run.t_at >= F2_T_TURN_MAX_MS)))
        return finish("HOME T TIMEOUT");
      if (run.turn_home && now - run.return_at >= 2u * run.out_ms + F1_RETURN_EXTRA_MS) return finish("HOME BY TIME");
      if (run.turn_home) { /* backup when US1 misses the wall: near home, the wheels stopped turning = against the wall */
        if (run.odo - run.stall_odo >= F1_STALL_MM) { run.stall_odo = run.odo; run.stall_at = now; }
        else if ((run.in_zone || home_done() >= home_expect() - F1_HOME_EARLY_MM) && now - run.stall_at >= F1_STALL_MS)
          return finish("HOME AT WALL");
      }
      break;
#endif
    case F1_ALIGN:
      if (elapsed >= US_SETTLE_MS) {
        enter(F1_RETURN, now);
        run.return_marker = false;
        run.return_scan_from = now;
      }
      break;
    case F1_FINISH_REVERSE:
      if (elapsed >= REVERSE_MS) return finish("FINISH 3 BLACK");
      break;
    default: break;
  }
  return NULL;
}
static F1_Command move(float base_left, float base_right, const char *label) {
  return (F1_Command){F1_MOVE, {F12_DUTY(base_left), F12_DUTY(base_right)}, label};
}
#if !F1_LEGACY
static F1_Command f3_keep(bool right, bool sharp, uint32_t now, const char *label) {
  const unsigned tape = run.filtered, near = right ? LINE_RIGHT : LINE_LEFT, far = right ? LINE_LEFT : LINE_RIGHT;
  observe_side(tape, now);
  if (tape & (near | LINE_CENTRE)) { run.f3_near = (tape & near) != 0u && !(tape & LINE_CENTRE); run.f3_near_at = now; }
  LineWheels w;
  if ((tape & near) && !(tape & LINE_CENTRE))      /* only the near IR: the line went that way - hard (sharper on the U) */
    w = sharp ? Line_ForwardSplit(near, F3_SHARP_INSIDE, F3_SHARP_OUTSIDE) : Line_Forward(near);
  else if (tape & near)                            /* near + centre */
    w = sharp ? Line_Forward(near) : Line_ForwardSplit(near, LINE_STEER_MEAN_PERCENT - 0.5f * LINE_STEER_BIAS_PERCENT,
                                                       LINE_STEER_MEAN_PERCENT + 0.5f * LINE_STEER_BIAS_PERCENT);
  else if (tape & LINE_CENTRE) w = Line_Forward(LINE_CENTRE); /* centre (+ far: another line joining) - straight */
  else if (run.f3_near && (uint32_t)(now - run.f3_near_at) < F1_LOST_TURN_MS) /* slipped out on the near side (or between */
    w = sharp ? Line_ForwardSplit(near, F3_SHARP_INSIDE, F3_SHARP_OUTSIDE)    /* the centre and near IRs): back to it */
              : Line_ForwardSplit(near, 6.0f, 18.0f);
  else if ((tape & far) && sharp) w = Line_Forward(far); /* round the U: only the far IR = the line is over there */
  else if (!(tape & far) && (uint32_t)(now - run.f3_near_at) < F1_LOST_TURN_MS) /* lost from the centre: most likely into */
    w = Line_ForwardSplit(near, LINE_STEER_MEAN_PERCENT - 0.5f * LINE_STEER_BIAS_PERCENT,  /* the gap on the kept side */
                          LINE_STEER_MEAN_PERCENT + 0.5f * LINE_STEER_BIAS_PERCENT);
  else w = Line_Forward(LINE_CENTRE); /* on a corner leg the far IR alone is the other line (or past the corner): straight */
  return move(w.left, w.right, label);
}
#endif
#if !F1_LEGACY
/* Base steering duties (before F12_DUTY), see function1.h */
#define F1_GENTLE_INSIDE (LINE_STEER_MEAN_PERCENT - 0.5f * LINE_STEER_BIAS_PERCENT)
#define F1_GENTLE_OUTSIDE (LINE_STEER_MEAN_PERCENT + 0.5f * LINE_STEER_BIAS_PERCENT)
#define F1_LOST_INSIDE 6.0f
#define F1_LOST_OUTSIDE 18.0f
#endif
#if !F1_LEGACY
bool F1_EncoderStraight(uint32_t now) {
  const F1_Phase p = F1_PhaseAt(now);
  return run.turn_home && (p == F1_TARGET_REVERSE || (p == F1_RETURN && run.f2_base && run.t_state == 1u) ||
                           ((p == F1_OUTBOUND || p == F1_RETURN) && run.f3_base && f3_advancing()) ||
                           /* F4's pivots: both wheels to equal pulses = the pivot stays on the spot (one wheel weaker in
                            * reverse walks the axle ~13 mm per 90 deg otherwise) */
                           ((p == F1_OUTBOUND || p == F1_RETURN) && f4_leg_live() && !run.f4_pause && f4_pivot_leg(run.f4_leg)));
}
#else
bool F1_EncoderStraight(uint32_t now) { (void)now; return false; }
#endif
static F1_Command follow(uint32_t now) {
  const unsigned tape = run.filtered;
  const bool returning = run.phase == F1_RETURN;
#if !F1_LEGACY
  if (returning && run.turn_home && run.f2_base && run.t_state == 1u)
    return move(LINE_NOMINAL_PERCENT, LINE_NOMINAL_PERCENT, "HOME: PAST THE T"); /* straight: the board matches the wheels */
  if (returning && run.turn_home && run.f2_base && run.t_state == 3u) {
    run.t_cmd = true; /* only scans after the first turn command count */
    return move(-US_PIVOT_PERCENT, US_PIVOT_PERCENT, "HOME: T LEFT"); /* in place, like the 180 */
  }
  if (f4_leg_live() && (returning || on_outbound())) return f4_command(now); /* F4's own legs (and its key approach) */
  if (f3_leg_live() && !run.approaching && (returning || on_outbound())) {
    if (f3_advancing()) return move(LINE_NOMINAL_PERCENT, LINE_NOMINAL_PERCENT, "F3: PAST CORNER"); /* the board matches */
    if (f3_pivoting()) {
      run.f3_cmd = true;
      return run.f3_leg == F3_O_VPIV || run.f3_leg == F3_O_BBACK ?
             move(-US_PIVOT_PERCENT, US_PIVOT_PERCENT, run.f3_leg == F3_O_VPIV ? "F3: V LEFT" : "F3: BEND BACK") :
             move(US_PIVOT_PERCENT, -US_PIVOT_PERCENT, run.f3_leg == F3_O_BPIV ? "F3: BEND RIGHT" : "HOME: V RIGHT");
    }
    switch (run.f3_leg) {
      case F3_O_U: return f3_keep(true, true, now, "F3: U RIGHT");
      case F3_O_ARM: return f3_keep(true, false, now, "F3: TO THE V");
      case F3_O_UP: return f3_keep(false, false, now, "F3: UP");
      case F3_H_DOWN: return f3_keep(false, false, now, "HOME: DOWN");
      default: return f3_keep(false, true, now, "HOME: U LEFT"); /* F3_H_ARMU */
    }
  }
  if (returning && run.turn_home && (run.in_zone || run.edge_hold)) {
    /* on / past the outline: straight on along the LINE's direction (its averaged heading), not whatever angle a steering
     * correction had at that moment */
    float aim = run.line_hdg; /* in the zone: first angle left until shifted F1_ZONE_SHIFT_MM, then the line's direction */
    if (run.in_zone && run.zone_lat < F1_ZONE_SHIFT_MM) aim += F1_ZONE_SHIFT_DEG;
    float e = aim - run.hdg; /* + = must turn counter-clockwise (left) */
    float d = F1_STRAIGHT_K * e;
    d = d > 8.0f ? 8.0f : (d < -8.0f ? -8.0f : d);
    return move(LINE_NOMINAL_PERCENT - d, LINE_NOMINAL_PERCENT + d, run.in_zone ? "HOME: INTO ZONE" : "HOME: OUTLINE?");
  }
#endif
  /* On the first forward command (e.g. after launch), the current mask can
   * already be 101 before a new scan arrives. Do not remember older launch scans. */
  observe_split(tape, now);
#if F1_LEGACY
  const unsigned steer = run.split_left ? LINE_LEFT : tape;
  const LineWheels wheels = Line_Forward(steer);
#else
  observe_side(tape, now);
  unsigned steer = run.split_left ? LINE_LEFT : tape;
  LineWheels wheels;
  if (run.split_left || tape == LINE_LEFT || tape == LINE_RIGHT) wheels = Line_Forward(steer); /* hard, as before */
  else if (tape == (LINE_LEFT | LINE_CENTRE)) { wheels = Line_ForwardSplit(LINE_LEFT, F1_GENTLE_INSIDE, F1_GENTLE_OUTSIDE); steer = LINE_LEFT; }
  else if (tape == (LINE_RIGHT | LINE_CENTRE)) { wheels = Line_ForwardSplit(LINE_RIGHT, F1_GENTLE_INSIDE, F1_GENTLE_OUTSIDE); steer = LINE_RIGHT; }
  else if (tape == 0u && run.last_side != 0 && (uint32_t)(now - run.lost_at) < F1_LOST_TURN_MS) {
    /* the line slipped out to that side: turn back toward it, slower */
    steer = run.last_side < 0 ? LINE_LEFT : LINE_RIGHT;
    wheels = Line_ForwardSplit(steer, F1_LOST_INSIDE, F1_LOST_OUTSIDE);
  } else wheels = Line_Forward(tape == LINE_CENTRE ? LINE_CENTRE : 0u); /* straight (incl. a gap after a centred line) */
#endif
  const char *label;
  if (run.approaching) label = "TARGET APPROACH";
#if F1_LEGACY
  else if (steer == LINE_LEFT) label = returning ? "TARGET FOLLOW L" : "RUN LEFT";
  else if (steer == LINE_RIGHT) label = returning ? "TARGET FOLLOW R" : "RUN RIGHT";
#else
  else if (steer == LINE_LEFT) label = returning ? "TARGET FOLLOW L" : (tape == 0u && !run.split_left ? "LOST: SEEK LEFT" : "RUN LEFT");
  else if (steer == LINE_RIGHT) label = returning ? "TARGET FOLLOW R" : (tape == 0u ? "LOST: SEEK RIGHT" : "RUN RIGHT");
#endif
  else if (returning) label = "TARGET FOLLOW";
  else label = tape == LINE_CENTRE ? "RUN STRAIGHT" : tape == 0u ? "NO LINE STRAIGHT" : "BRANCH STRAIGHT";
  return move(wheels.left, wheels.right, label);
}
F1_Command F1_CommandAt(uint32_t now, uint32_t fresh_us1_mm) {
  const unsigned tape = run.filtered;
  /* Resolve the return finish on the latest scan AND ping before committing
   * any due hold, even if the 350 ms deadline falls on this exact tick. */
  if (return_end(fresh_us1_mm))
    return (F1_Command){.request = F1_STOP, .status = finish("TARGET LINE END")};
  if (run.approaching && (uint32_t)(now - run.near_at) >= US_APPROACH_MS) {
    enter(F1_TARGET_HOLD, now); /* time the full hold from the first zero request, not the ping */
#if !F1_LEGACY
    run.odo_out = run.odo; run.out_ms = now - run.start_at;
#endif
#if F1_LEGACY
    run.next_armed = true; /* a committed hold arms the next standalone run */
#endif
  }
  switch (F1_PhaseAt(now)) {
    case F1_IDLE:
      return (F1_Command){.request = F1_KEEP_STOPPED, .status = "IDLE"};
    case F1_TARGET_HOLD:
      return (F1_Command){.request = F1_KEEP_STOPPED, .status = "TARGET " DRIVE_MM_LABEL(DRIVE_US1_STOP_MM) " HOLD"};
    case F1_LAUNCH:
      return move(LAUNCH_PERCENT, LAUNCH_PERCENT, "LAUNCH 13.5%");
    case F1_OUTBOUND: case F1_RETURN: case F1_TARGET_APPROACH: return follow(now);
    case F1_RETRACE: /* the board drives home on the encoders (retrace.h) */
      return (F1_Command){.request = F1_KEEP_STOPPED, .status = "RETRACE HOME"};
    case F1_TARGET_REVERSE:
      return move(-US_REVERSE_PERCENT, -US_REVERSE_PERCENT, run.phase == F1_TARGET_REVERSE && F1_RETURN_TURN && !F1_LEGACY ? "HOME: BACK OFF" : "TARGET REV 18%");
    case F1_PIVOT: case F1_ALIGN: {
      const char *label = run.phase == F1_ALIGN ? "TARGET LINE GO" :
                          (uint32_t)(now - run.phase_at) < US_SPIN_MIN_MS ? "TARGET SPIN CCW" : "TARGET FIND LINE";
      return move(-US_PIVOT_PERCENT, US_PIVOT_PERCENT, label);
    }
    case F1_FINISH_REVERSE:
      return move(tape == 4u ? -FINISH_INSIDE_PERCENT : -LINE_NOMINAL_PERCENT,
                  tape == 1u ? -FINISH_INSIDE_PERCENT : -LINE_NOMINAL_PERCENT, "FINISH REV");
  }
  return (F1_Command){.request = F1_STOP, .status = finish("DRIVE PLAN ERROR")};
}
