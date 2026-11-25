#include "ar_nav.h"

typedef enum { AR_FOLLOW_NORMAL = 0, AR_FOLLOW_IGNORE_L, AR_FOLLOW_IGNORE_R, AR_FOLLOW_PREFER_R } AR_FollowMode;
typedef enum { AR_END_CROSS = 0, AR_END_KEY, AR_END_HOME } AR_LegEnd;

typedef struct {
  AR_FollowMode mode;
  AR_LegEnd end;
  int8_t pivot_dir; /* END_CROSS only: -1 = pivot left, +1 = pivot right */
} AR_Leg;

/* Key A: bottom line (ignore the right-hand curve branch), CROSS, pivot right, vertical line to the key.
 * Home: vertical line down (ignore the diagonal that joins on the left), CROSS, pivot left, bottom line home. */
static const AR_Leg out_a[] = {{AR_FOLLOW_IGNORE_R, AR_END_CROSS, +1}, {AR_FOLLOW_IGNORE_R, AR_END_KEY, 0}};
static const AR_Leg ret_a[] = {{AR_FOLLOW_IGNORE_L, AR_END_CROSS, -1}, {AR_FOLLOW_IGNORE_L, AR_END_HOME, 0}};
/* Key B: bottom line, take the right-hand branch (U-curve), follow the line round its corners to the key. */
static const AR_Leg out_b[] = {{AR_FOLLOW_PREFER_R, AR_END_KEY, 0}};
static const AR_Leg ret_b[] = {{AR_FOLLOW_NORMAL, AR_END_HOME, 0}};

typedef struct {
  const AR_Leg *legs;
  uint8_t count;
} AR_Route;

static AR_Route route_for(const AR_Nav *n, bool returning) {
  AR_Route r;
  if (n->key == AR_KEY_A) {
    r.legs = returning ? ret_a : out_a;
    r.count = 2;
  } else {
    r.legs = returning ? ret_b : out_b;
    r.count = 1;
  }
  return r;
}

static uint32_t since(uint32_t now, uint32_t t) { return (uint32_t)(now - t); }

static AR_Cmd cmd(int l, int r) {
  AR_Cmd c;
  if (l > 100) l = 100;
  if (l < -100) l = -100;
  if (r > 100) r = 100;
  if (r < -100) r = -100;
  c.left = (int8_t)l;
  c.right = (int8_t)r;
  return c;
}

static AR_Cmd stop_cmd(void) { return cmd(0, 0); }

static AR_Cmd pivot_cmd(int dir) { /* +1 = turn right on the spot */
  return dir > 0 ? cmd(AR_PIVOT_PCT, -AR_PIVOT_PCT) : cmd(-AR_PIVOT_PCT, AR_PIVOT_PCT);
}

static void enter_phase(AR_Nav *n, AR_Phase p, uint32_t now) {
  n->phase = p;
  n->phase_since = now;
}

static void begin_leg(AR_Nav *n, uint32_t now) {
  n->sub = AR_SUB_FOLLOW;
  n->lost = false;
  n->cross_active = false;
  n->us_close = 0;
  n->leg_seen_line = false;
  n->lost_since = now;
  n->sub_since = now;
}

void AR_Nav_Init(AR_Nav *n) {
  n->phase = AR_PH_IDLE;
  n->key = AR_KEY_A;
  n->fault = AR_FAULT_NONE;
  n->sub = AR_SUB_FOLLOW;
  n->leg = 0;
  n->pivot_dir = 0;
  n->last_side = 0;
  n->pivot_seen_off = false;
  n->lost = false;
  n->leg_seen_line = false;
  n->phase_since = n->sub_since = n->lost_since = n->cross_since = n->countdown_end = 0;
  n->us_close = 0;
  n->cross_active = false;
  n->step = n->step_sub = 0;
  n->odo = n->step_odo = n->zero_odo = n->out_edges = n->leg_edges = n->top_edges = n->top_back_edges = n->step_since = 0;
}

void AR_Nav_SetKey(AR_Nav *n, AR_Key key) {
  if (n->phase == AR_PH_IDLE || n->phase == AR_PH_DONE || n->phase == AR_PH_FAULT)
    n->key = key;
}

void AR_Nav_Start(AR_Nav *n, uint32_t now_ms) {
  if (n->phase != AR_PH_IDLE && n->phase != AR_PH_DONE && n->phase != AR_PH_FAULT)
    return;
  n->fault = AR_FAULT_NONE;
  n->leg = 0;
  n->last_side = 0;
  n->countdown_end = now_ms + AR_COUNTDOWN_MS;
  n->coast_max = n->settle_max_ms = 0;
  enter_phase(n, AR_PH_COUNTDOWN, now_ms);
}

void AR_Nav_SetOdometer(AR_Nav *n, uint32_t edges) { n->odo = n->odo_l = n->odo_r = edges; }
void AR_Nav_SetWheelOdometer(AR_Nav *n, uint32_t l, uint32_t r) {
  n->odo_l = l;
  n->odo_r = r;
  n->odo = (uint32_t)(((uint64_t)l + r) / 2u);
}
bool AR_Nav_NeedsIR(const AR_Nav *n) { return n->key < AR_KEY_MID || AR_ROUTE_USE_IR; }

const char *AR_Nav_KeyText(const AR_Nav *n) {
  switch (n->key) {
    case AR_KEY_A: return "A";
    case AR_KEY_B: return "B";
    case AR_KEY_MID: return "MID";
    case AR_KEY_TOP: return "TOP";
    case AR_KEY_RGT: return "RGT";
    case AR_KEY_TOP2: return "TOP2";
    case AR_KEY_TOP1: return "TOP1";
    case AR_KEY_RGT1: return "RGT1";
    case AR_KEY_TRN4: return "TRN4";
    case AR_KEY_STR: return "STR";
    case AR_KEY_F20: return "F20";
    default: return "ALL";
  }
}

void AR_Nav_Abort(AR_Nav *n) {
  n->phase = AR_PH_IDLE;
  n->fault = AR_FAULT_NONE;
}

/* Steer from the three sensors. IGNORE_x only ignores that side while the centre still sees the line (that is a branch
 * leaving the line we are on); PREFER_R follows the right-hand branch whenever it is there. */
static AR_Cmd follow(AR_Nav *n, AR_Line s, AR_FollowMode mode) {
  const int b = AR_BASE_PCT;
  if (mode == AR_FOLLOW_IGNORE_L && s.c) s.l = false;
  if (mode == AR_FOLLOW_IGNORE_R && s.c) s.r = false;
  if (mode == AR_FOLLOW_PREFER_R && s.r) s.c = false;

  if (s.c && !s.l && !s.r) return cmd(b, b);
  if (s.l && s.c) { n->last_side = -1; return cmd(b * 55 / 100, b); }
  if (s.r && s.c) { n->last_side = +1; return cmd(b, b * 55 / 100); }
  if (s.l && !s.c && !s.r) { n->last_side = -1; return cmd(0, b); }
  if (s.r && !s.c && !s.l) { n->last_side = +1; return cmd(b, 0); }
  return cmd(b / 2, b / 2); /* left and right but not centre: unclear, go straight slowly */
}

static AR_Cmd search(const AR_Nav *n) {
  if (n->last_side < 0) return cmd(-AR_PIVOT_PCT / 2, AR_PIVOT_PCT); /* line was last on the left: swing left */
  if (n->last_side > 0) return cmd(AR_PIVOT_PCT, -AR_PIVOT_PCT / 2);
  return cmd(AR_BASE_PCT / 2, AR_BASE_PCT / 2); /* never seen it: creep forward to find it */
}

static AR_Cmd fault(AR_Nav *n, AR_Fault f, uint32_t now) {
  n->fault = f;
  enter_phase(n, AR_PH_FAULT, now);
  return stop_cmd();
}

/* A pivot is finished when the centre sensor sees a line again after the sensors have been off it. */
static bool pivot_done_min(AR_Nav *n, AR_Line s, uint32_t now, uint32_t min_ms) {
  if (!s.l && !s.c && !s.r) n->pivot_seen_off = true;
  return n->pivot_seen_off && s.c && since(now, n->sub_since) >= min_ms;
}
static bool pivot_done(AR_Nav *n, AR_Line s, uint32_t now) { return pivot_done_min(n, s, now, AR_PIVOT_MIN_MS); }

/* ===================================================================================================================
 * HARDCODED ROUTE: middle key -> top key -> right key (-> home).  Robot starts in the start zone facing west.
 *
 *   MID   follow the bottom line west, ignore the curve branch on the right. The first CROSS is junction J: drive on
 *         so the axle is over J and note the wheel-feedback count. Stop at the key (ultrasonic). REVERSE exactly the
 *         counted distance (so the axle is back over J whatever the wheel scale is), then pivot right to face north.
 *   TOP   follow the vertical line north to the key (ultrasonic), reverse AR_TOP_BACK_CM, pivot right (about 116 deg)
 *         until the centre sensor finds the diagonal.
 *   RIGHT follow the diagonal AR_DIAG_CM, pivot left (about 89 deg) onto the short up-right segment, follow
 *         AR_UP_CM, pivot right (about 63 deg) onto the horizontal, follow it to the key (ultrasonic).
 *   HOME  (ALL only) the normal grab / turn-around / follow-the-line-home logic takes over.
 * Pivots finish when the centre sensor finds a line after the sensors have left the old one, so the angles below only
 * size the minimum pivot time. Distances use the wheel feedback (AR_EDGES_PER_CM); the keys use the ultrasonic. */
typedef enum { RS_FOLLOW_KEY, RS_FOLLOW_KEY_ZERO, RS_GRAB, RS_BACK_ZERO, RS_BACK_CM, RS_PIVOT, RS_FOLLOW_CM, RS_BACK_MEASURED, RS_BACK_TOP, RS_BACK_US, RS_FWD_US, RS_FWD_J, RS_BACK_J } RouteType;
typedef struct {
  uint8_t type;
  int16_t arg; /* FOLLOW_KEY: follow mode; BACK_CM / FOLLOW_CM: cm; PIVOT: degrees, + = right */
  uint8_t stage; /* 1 = middle key, 2 = top key, 3 = right key */
  const char *name;
} RouteStep;

static const RouteStep route[] = {
    {RS_FOLLOW_KEY_ZERO, AR_FOLLOW_IGNORE_R, 1, "MID TO KEY"},
    {RS_GRAB, 0, 1, "MID GRAB"},
#if AR_J_US && !AR_ROUTE_USE_IR
    {RS_BACK_J, 0, 1, "MID REVERSE"}, /* back to J: until the US reads AR_J_US_MM from the middle stone */
#else
    {RS_BACK_ZERO, 0, 1, "MID REVERSE"},
#endif
    {RS_PIVOT, +90, 2, "TURN NORTH"},
    {RS_FOLLOW_KEY, AR_FOLLOW_IGNORE_R, 2, "TOP TO KEY"},
    {RS_GRAB, 0, 2, "TOP GRAB"},
#if AR_TOP_TURN_US
    {RS_BACK_US, AR_TOP_TURN_MM, 2, "TOP REVERSE"}, /* reverse until the US reads AR_TOP_TURN_MM from the top stone */
#else
    {RS_BACK_CM, AR_TOP_BACK_CM, 2, "TOP REVERSE"},
#endif
    {RS_PIVOT, AR_RIGHT_STRAIGHT ? 90 : AR_HDG_DIAG_DEG, 3, "TURN DIAGONAL"}, /* straight mode: 90 deg, east, then straight to the key */
    {RS_FOLLOW_CM, AR_DIAG_CM, 3, "DIAGONAL"},
    {RS_PIVOT, AR_HDG_UP_DEG - AR_HDG_DIAG_DEG, 3, "TURN UP-RIGHT"},
    {RS_FOLLOW_CM, AR_UP_CM, 3, "UP-RIGHT"},
    {RS_PIVOT, 90 - AR_HDG_UP_DEG, 3, "TURN EAST"},
    {RS_FOLLOW_KEY, AR_FOLLOW_NORMAL, 3, "RIGHT TO KEY"},
    /* ---- return-home sequences (entered by a jump, see route_next); stage 9 = never part of the normal flow ---- */
    {RS_BACK_CM, AR_J_TO_SZ_CM, 9, "MID HOME"},                                /* MID: reverse from J into the zone */
    {RS_BACK_TOP, 1, 9, "TOP BACK TO J"},   /* from the diagonal start (ALL / RGT1 mirror): measured top leg minus AR_TOP_BACK_CM */
    {RS_PIVOT, +90, 9, "TURN EAST"},
    {RS_FOLLOW_CM, AR_J_TO_SZ_CM, 9, "TOP HOME"},
    /* mirror of the route back from the right key (ALL / RGT1); it finishes by jumping to the TOP return above */
    {RS_GRAB, 0, 9, "RIGHT GRAB"},
    {RS_BACK_MEASURED, 0, 9, "RIGHT BACK"},
    {RS_PIVOT, AR_RIGHT_STRAIGHT ? -90 : -(90 - AR_HDG_UP_DEG), 9, "UNTURN EAST"}, /* straight mode: left 90 back to north */
    {RS_BACK_CM, AR_UP_CM, 9, "BACK UP-RIGHT"},
    {RS_PIVOT, AR_HDG_DIAG_DEG - AR_HDG_UP_DEG, 9, "UNTURN"},
    {RS_BACK_CM, AR_DIAG_CM, 9, "BACK DIAGONAL"},
    {RS_PIVOT, -AR_HDG_DIAG_DEG, 9, "UNTURN DIAG"},
    /* turn calibration (TRN4): four 90 degree right pivots */
    {RS_PIVOT, +AR_TRN_DEG, 9, "TURN 1 OF 4"},
    {RS_PIVOT, +AR_TRN_DEG, 9, "TURN 2 OF 4"},
    {RS_PIVOT, +AR_TRN_DEG, 9, "TURN 3 OF 4"},
    {RS_PIVOT, +AR_TRN_DEG, 9, "TURN 4 OF 4"},
    {RS_BACK_TOP, 0, 9, "TOP BACK TO J"}, /* TOP / TOP1 / TOP2: straight back down to J, exactly what the approach travelled */
    /* TOP / RGT from the start zone */
#if AR_J_US && !AR_ROUTE_USE_IR
    {RS_FWD_J, 0, 9, "TO J"},                                 /* west to J: until the US reads AR_J_US_MM from the middle stone */
#else
    {RS_FOLLOW_CM, AR_SZ_TO_J_CM, 9, "TO J"},                 /* west along the bottom line to junction J */
#endif
    {RS_FWD_US, AR_TOP_TURN_MM, 9, "TO TURN POINT"},          /* RGT: north until the US reads AR_TOP_TURN_MM from the top stone */
    {RS_FOLLOW_CM, AR_STR_TEST_CM, 9, "STRAIGHT TEST"},       /* STR: drive straight, then stop */
    {RS_FOLLOW_CM, AR_F20_CM, 9, "FORWARD 20 CM"},            /* F20: encoder accuracy test, then stop */
};
#define ROUTE_COUNT ((uint8_t)(sizeof(route) / sizeof(route[0])))
#define ROUTE_MAIN_COUNT 13u
#define HOME_MID_STEP 13u
#define HOME_TOP_STEP 14u
#define HOME_TOP_END 16u   /* last step of the TOP return */
#define HOME_RIGHT_STEP 17u
#define HOME_RIGHT_END 23u
#define TURN_TEST_STEP 24u
#define HOME_TOPFULL_STEP 28u
#define TO_J_STEP 29u
#define TO_TURN_STEP 30u
#define STR_TEST_STEP 31u /* last step of the mirror; then continue at HOME_TOP_STEP */
#define F20_STEP 32u

static uint8_t route_last_stage(const AR_Nav *n) {
  return n->key == AR_KEY_MID ? 1 : ((n->key == AR_KEY_TOP || n->key == AR_KEY_TOP1 || n->key == AR_KEY_TOP2) ? 2 : 3);
}
static uint8_t route_first_step(const AR_Nav *n) {
  if (n->key == AR_KEY_TOP || n->key == AR_KEY_RGT) return TO_J_STEP;
  if (n->key == AR_KEY_STR) return STR_TEST_STEP;
  if (n->key == AR_KEY_F20) return F20_STEP;
  return n->key == AR_KEY_TOP1 ? 3 : (n->key == AR_KEY_RGT1 ? 7 : (n->key == AR_KEY_TRN4 ? TURN_TEST_STEP : 0));
}
static uint32_t cm_to_edges(int cm) { return (uint32_t)((float)cm * AR_EDGES_PER_CM); }

static void route_begin_step(AR_Nav *n, uint32_t now) {
  n->step_sub = 0; /* 0 = stand still until the wheels have stopped (see AR_STEP_SETTLE_MS / AR_STILL_MS) */
  n->settle_l = n->settle_start_l = n->odo_l;
  n->settle_r = n->settle_start_r = n->odo_r;
  n->still_since = now;
  n->step_since = now;
  n->sub_since = now;
  n->step_odo = n->odo;
  n->step_l = n->odo_l;
  n->step_r = n->odo_r;
  n->bal_i = 0;
  n->bal_t = now;
  n->lost = false;
  n->lost_since = now;
  n->cross_active = false;
  n->us_close = 0;
  n->leg_seen_line = true; /* each step starts on a line */
  n->j_us_seen = false;
  n->pivot_seen_off = false;
}

static AR_Cmd route_finish(AR_Nav *n, uint32_t now) {
#if AR_ROUTE_HOME
  if (n->key == AR_KEY_ALL || n->key == AR_KEY_RGT1 || n->key == AR_KEY_RGT) { /* right key done: mirror the route back (grab, reverse, unturn...) */
    n->step = HOME_RIGHT_STEP;
    route_begin_step(n, now);
    return stop_cmd();
  }
#else
  if (n->key == AR_KEY_ALL) { enter_phase(n, AR_PH_GRAB, now); return stop_cmd(); } /* old: grab, turn round, follow the line home */
#endif
  enter_phase(n, AR_PH_DONE, now);
  return stop_cmd();
}

static AR_Cmd route_next(AR_Nav *n, uint32_t now) {
  const uint8_t cur = n->step;
  if (cur == STR_TEST_STEP || cur == F20_STEP) { /* straight-line / 20 cm test: done */
    enter_phase(n, AR_PH_DONE, now);
    return stop_cmd();
  }
  if (cur == TO_J_STEP) { /* at J: turn north */
    n->step = 3u;
    route_begin_step(n, now);
    return stop_cmd();
  }
  if (cur == TO_TURN_STEP) { /* at the turning point: turn east to the right key */
    n->step = 7u;
    route_begin_step(n, now);
    return stop_cmd();
  }
  if (cur == 3u && n->key == AR_KEY_RGT) { /* RGT: facing north at J, drive up to the turning point */
    n->step = TO_TURN_STEP;
    route_begin_step(n, now);
    return stop_cmd();
  }
  if (cur == HOME_TOPFULL_STEP) { /* straight back to J, then face east and drive into the zone */
    n->step = HOME_TOP_STEP + 1u; /* TURN EAST */
    route_begin_step(n, now);
    return stop_cmd();
  }
  if (cur >= TURN_TEST_STEP) { /* turn calibration: after the fourth pivot, stop */
    if (cur == HOME_TOPFULL_STEP - 1u) {
      enter_phase(n, AR_PH_DONE, now);
      return stop_cmd();
    }
    n->step++;
    route_begin_step(n, now);
    return stop_cmd();
  }
  if (cur >= ROUTE_MAIN_COUNT) { /* inside a return-home sequence */
    if (cur == HOME_MID_STEP || cur == HOME_TOP_END) { /* arrived in the start zone */
      enter_phase(n, AR_PH_DONE, now);
      return stop_cmd();
    }
    n->step = cur == HOME_RIGHT_END ? HOME_TOP_STEP : (uint8_t)(cur + 1u); /* mirror ends at the diagonal start, then the TOP return */
#if AR_RIGHT_STRAIGHT
    if (cur == HOME_RIGHT_STEP + 2u) n->step = HOME_TOP_STEP; /* after the 90 deg unturn: already facing north at the diagonal start */
#endif
    route_begin_step(n, now);
    return stop_cmd();
  }
  n->step++;
#if AR_RIGHT_STRAIGHT
  if (cur == 7u) { /* turned east: straight to the right key, no zig-zag */
    n->step = 12u;
    route_begin_step(n, now);
    return stop_cmd();
  }
#endif
#if AR_ROUTE_HOME
  if (n->step == 6u && (n->key == AR_KEY_TOP || n->key == AR_KEY_TOP1 || n->key == AR_KEY_TOP2)) { /* going home anyway: skip the 13 cm stop, reverse to J in one go */
    n->step = HOME_TOPFULL_STEP;
    route_begin_step(n, now);
    return stop_cmd();
  }
#endif
  if (n->step >= ROUTE_MAIN_COUNT) return route_finish(n, now); /* after the right key: ALL goes home, RGT1 stops */
  if (route[n->step].stage > route_last_stage(n)) {
#if AR_ROUTE_HOME
    const uint8_t last = route_last_stage(n);
    if (last == 1u || last == 2u) { /* MID or TOP / TOP1: drive back into the start zone */
      n->step = last == 1u ? HOME_MID_STEP : HOME_TOP_STEP;
      route_begin_step(n, now);
      return stop_cmd();
    }
#endif
    enter_phase(n, AR_PH_DONE, now);
    return stop_cmd();
  }
  route_begin_step(n, now);
  return stop_cmd();
}

#if AR_ROUTE_USE_IR
static AR_Cmd reverse_cmd(AR_Line s) {
  const int b = -AR_BASE_PCT, slow = b * AR_REV_TRIM_PCT / 100;
  if (s.l && !s.r) return cmd(b, slow);  /* line on the left: front must swing left */
  if (s.r && !s.l) return cmd(slow, b);
  return cmd(b, b);
}
#endif

#if !AR_ROUTE_USE_IR
/* Encoder-only straight drive: if one wheel has turned more than the other since the step began, slow it down. */
/* Balance correction in percent points (+ = left wheel is ahead and must slow down, right must speed up). PI on the difference
 * between the two wheels' edge counts since the step began. Works for driving AND pivoting (a pivot needs equal wheel travel
 * too, or the AR swings round a point off its centre). */
static int balance_corr(AR_Nav *n, uint32_t now) {
  const int32_t dl = (int32_t)(n->odo_l - n->step_l), dr = (int32_t)(n->odo_r - n->step_r);
  const int32_t err = dl + dl * AR_DRIFT_TRIM_PERMILLE / 1000 - dr; /* + = left ahead (scaled for a bigger / smaller left wheel) */
  uint32_t dt = (uint32_t)(now - n->bal_t);
  if (dt > 50U) dt = 50U;
  n->bal_t = now;
  n->bal_i += err * (int32_t)dt;
  const int32_t i_max = (int32_t)AR_BALANCE_MAX_PCT * 1000 / AR_BALANCE_I_PCT_PER_EDGE_S; /* keeps KI*integral in range */
  if (n->bal_i > i_max * 1000) n->bal_i = i_max * 1000;
  if (n->bal_i < -i_max * 1000) n->bal_i = -i_max * 1000;
  int corr = (int)(err * AR_BALANCE_PCT_PER_EDGE) + (int)(n->bal_i / 1000 * AR_BALANCE_I_PCT_PER_EDGE_S);
  if (corr > AR_BALANCE_MAX_PCT) corr = AR_BALANCE_MAX_PCT;
  if (corr < -AR_BALANCE_MAX_PCT) corr = -AR_BALANCE_MAX_PCT;
  return corr;
}
static AR_Cmd straight_cmd(AR_Nav *n, int dir, uint32_t now, int base) {
  const uint32_t since_start = (uint32_t)(now - n->sub_since); /* soft start: no twist as the AR sets off */
  if (since_start < AR_STRAIGHT_RAMP_MS && base > AR_STRAIGHT_START_PCT)
    base = AR_STRAIGHT_START_PCT + (int)((uint32_t)(base - AR_STRAIGHT_START_PCT) * since_start / AR_STRAIGHT_RAMP_MS);
  const int corr = balance_corr(n, now);
  return dir > 0 ? cmd(base - corr, base + corr) : cmd(-(base - corr), -(base + corr));
}
/* Pivot with equal wheel travel: dir +1 = right (left wheel forward, right wheel back). */
static AR_Cmd pivot_bal_cmd(AR_Nav *n, int dir, uint32_t now, int base) {
  const uint32_t since_start = (uint32_t)(now - n->sub_since); /* soft start: both motors spin up together */
  if (since_start < AR_PIVOT_RAMP_MS && base > AR_PIVOT_START_PCT)
    base = AR_PIVOT_START_PCT + (int)((uint32_t)(base - AR_PIVOT_START_PCT) * since_start / AR_PIVOT_RAMP_MS);
  int corr = balance_corr(n, now);
  if (corr > base - 5) corr = base - 5; /* never stall or reverse the slow wheel */
  if (corr < -(base - 5)) corr = -(base - 5);
  return dir > 0 ? cmd(base - corr, -(base + corr)) : cmd(-(base - corr), base + corr);
}
/* Speed for a fixed-distance straight leg: slower for the last AR_SLOW_ZONE_CM, but never more than the last third of the
 * leg, so a short leg (the 13 cm reverse from the top key) still starts at full speed and does not stall at low duty. */
static int leg_speed(uint32_t remaining, uint32_t total) {
  uint32_t zone = (uint32_t)((float)AR_SLOW_ZONE_CM * AR_EDGES_PER_CM);
  if (zone > total / 3u) zone = total / 3u;
  return remaining < zone ? AR_SLOW_SPEED_PCT : AR_BASE_PCT;
}
static uint32_t coast_edges(void) { return (uint32_t)(AR_COAST_CM * AR_EDGES_PER_CM); }
static uint32_t reverse_stop_edges(void) { return (uint32_t)((AR_COAST_CM + AR_REVERSE_TRIM_CM) * AR_EDGES_PER_CM); }
static uint32_t pivot_edges(int deg) {
  if (deg < 0) deg = -deg;
  float eff = (float)deg;
  if (deg > 90) eff -= AR_PIVOT_LONG_TRIM_PCT / 100.0f * (float)(deg - 90); /* long turns coast further */
  return (uint32_t)(3.14159265f * (float)AR_TRACK_MM / 10.0f * eff / 360.0f * AR_EDGES_PER_CM);
}
#endif

static AR_Cmd route_run(AR_Nav *n, uint32_t now, AR_Line s, int32_t us_mm, bool us_fresh) {
  const bool any = s.l || s.c || s.r;
  if (n->step >= ROUTE_COUNT) return route_finish(n, now);
  const RouteStep *st = &route[n->step];
  if (since(now, n->step_since) >= AR_STEP_MAX_MS) return fault(n, AR_FAULT_STEP, now);

  if (n->step_sub == 0) { /* settle: wait until the AR has really stopped rolling */
    if (n->odo_l != n->settle_l || n->odo_r != n->settle_r) {
      n->settle_l = n->odo_l;
      n->settle_r = n->odo_r;
      n->still_since = now;
    }
    const uint32_t waited = since(now, n->step_since);
    const uint32_t min_ms = AR_STEP_SETTLE_MS + (st->type == RS_PIVOT ? AR_PIVOT_SETTLE_EXTRA_MS : 0U);
    if (waited < AR_STEP_SETTLE_MAX_MS && (waited < min_ms || since(now, n->still_since) < AR_STILL_MS)) return stop_cmd();
    {
      const uint32_t rl = n->odo_l - n->settle_start_l, rr = n->odo_r - n->settle_start_r;
      const uint32_t rolled = rl > rr ? rl : rr;
      if (rolled > n->coast_max) n->coast_max = (uint16_t)(rolled > 65535U ? 65535U : rolled);
      if (waited > n->settle_max_ms) n->settle_max_ms = (uint16_t)(waited > 65535U ? 65535U : waited);
    }
    n->step_sub = 1;
    n->sub_since = now;
    n->step_odo = n->odo;
    n->step_l = n->odo_l;
    n->step_r = n->odo_r;
    n->bal_i = 0;
    n->bal_t = now;
    n->stall_odo = n->odo;
    n->stall_since = now;
    n->zero_odo = n->odo;
    return stop_cmd();
  }
  const uint32_t travelled = n->odo - n->step_odo;

  switch (st->type) {
    case RS_FOLLOW_KEY:
    case RS_FOLLOW_KEY_ZERO: {
#if !AR_ROUTE_USE_IR
      if (us_fresh) {
        if (us_mm >= 0 && us_mm <= AR_STOP_MM) { if (n->us_close < 255U) n->us_close++; }
        else n->us_close = 0;
      }
      if (n->us_close >= AR_US_CONFIRM) {
        n->leg_edges = n->odo - n->step_odo;
        if (n->step == 4u) n->top_edges = n->leg_edges;
        return route_next(n, now);
      }
      /* backup: wheels no longer turning while driving = pushing against the stone (US may give no echo this close) */
      if ((int32_t)(n->odo - n->stall_odo) >= AR_STALL_MIN_EDGES) {
        n->stall_odo = n->odo;
        n->stall_since = now;
      } else if (since(now, n->stall_since) >= AR_STALL_MS) {
        n->leg_edges = n->odo - n->step_odo;
        if (n->step == 4u) n->top_edges = n->leg_edges;
        return route_next(n, now);
      }
      return straight_cmd(n, +1, now, AR_BASE_PCT);
#endif
      if (n->step_sub == 2) { /* axle over J: drive on a little, then note the count */
        if (since(now, n->sub_since) >= AR_CROSS_ADVANCE_MS) {
          n->zero_odo = n->odo;
          n->step_sub = 3;
        }
        return cmd(AR_BASE_PCT, AR_BASE_PCT);
      }
      if (st->type == RS_FOLLOW_KEY_ZERO && n->step_sub == 1) {
        if (s.l && s.c && s.r) {
          if (!n->cross_active) { n->cross_active = true; n->cross_since = now; }
          if (since(now, n->cross_since) >= AR_CROSS_MS) {
            n->step_sub = 2;
            n->sub_since = now;
            return cmd(AR_BASE_PCT, AR_BASE_PCT);
          }
        } else {
          n->cross_active = false;
        }
      }
      if (us_fresh) {
        if (us_mm >= 0 && us_mm <= AR_STOP_MM) { if (n->us_close < 255U) n->us_close++; }
        else n->us_close = 0;
      }
      if (n->us_close >= AR_US_CONFIRM) {
        if (st->type == RS_FOLLOW_KEY_ZERO) n->out_edges = n->odo - n->zero_odo;
        n->leg_edges = n->odo - n->step_odo;
        if (n->step == 4u) n->top_edges = n->leg_edges;
        return route_next(n, now);
      }
      if (n->lost && since(now, n->lost_since) >= AR_END_LOST_MS) return fault(n, AR_FAULT_LOST, now);
      if (!any) return search(n);
      return follow(n, s, (AR_FollowMode)st->arg);
    }
    case RS_GRAB:
      if (since(now, n->sub_since) >= AR_GRAB_PAUSE_MS) return route_next(n, now);
      return stop_cmd();
    case RS_BACK_TOP: { /* reverse the measured top-key approach (arg 1: minus the 13 cm already reversed) */
      uint32_t target = n->top_edges ? n->top_edges : cm_to_edges(AR_TOP_KEY_FROM_J_CM);
      if (st->arg) { /* minus what was already reversed from the key to the turning point */
        const uint32_t back = n->top_back_set ? n->top_back_edges : cm_to_edges(AR_TOP_BACK_CM);
        target = target > back ? target - back : 0u;
      }
#if AR_ROUTE_USE_IR
      if (travelled >= target) return route_next(n, now);
      return reverse_cmd(s);
#else
      if (travelled + reverse_stop_edges() >= target) return route_next(n, now);
      return straight_cmd(n, -1, now, leg_speed(target - travelled, target));
#endif
    }
#if !AR_ROUTE_USE_IR
    case RS_FWD_J:
    case RS_BACK_J: { /* to junction J by the US (distance to the middle stone), within a window around the encoder distance */
      const bool fwd = st->type == RS_FWD_J;
      const uint32_t nominal = cm_to_edges(fwd ? AR_SZ_TO_J_CM : AR_MID_BACK_CM);
      const uint32_t win = cm_to_edges(AR_J_WINDOW_CM);
      if (us_fresh) {
        if (us_mm > 0 && us_mm < 1500) n->j_us_seen = true;
        const bool hit = fwd ? (us_mm >= 0 && us_mm <= AR_J_US_MM + AR_TOP_TURN_LEAD_MM) : (us_mm >= AR_J_US_MM - AR_TOP_TURN_LEAD_MM);
        if (hit) { if (n->us_close < 255U) n->us_close++; }
        else n->us_close = 0;
      }
      const uint32_t early = fwd ? coast_edges() : reverse_stop_edges();
      bool at = (travelled + win >= nominal && n->us_close >= AR_US_CONFIRM) /* US says J, inside the window */
                || travelled >= nominal + win                                 /* hard limit */
                || (!n->j_us_seen && travelled + early >= nominal);           /* US never usable: encoder distance */
      if (at) return route_next(n, now);
      const bool near = us_mm >= 0 && (fwd ? us_mm < AR_J_US_MM + 150 : us_mm > AR_J_US_MM - 150);
      return straight_cmd(n, fwd ? +1 : -1, now, near ? AR_SLOW_SPEED_PCT : AR_BASE_PCT);
    }
#endif
    case RS_FWD_US: { /* forward until the front US reads arg mm or less (the top stone), or the encoder limit */
      if (us_fresh) {
        if (us_mm >= 0 && us_mm <= st->arg + AR_TOP_TURN_LEAD_MM) { if (n->us_close < 255U) n->us_close++; }
        else n->us_close = 0;
      }
      if (n->us_close >= AR_US_CONFIRM || travelled >= cm_to_edges(AR_J_TO_TURN_MAX_CM)) {
        n->top_edges = travelled;   /* the way home reverses exactly this (J -> turning point) */
        n->top_back_edges = 0;
        n->top_back_set = true;
        return route_next(n, now);
      }
#if AR_ROUTE_USE_IR
      if (!(s.l || s.c || s.r)) return search(n);
      return follow(n, s, AR_FOLLOW_IGNORE_R);
#else
      return straight_cmd(n, +1, now, (us_mm >= 0 && us_mm < st->arg + 150) ? AR_SLOW_SPEED_PCT : AR_BASE_PCT);
#endif
    }
    case RS_BACK_US: { /* reverse slowly until the front US reads arg mm from the stone (or the encoder limit) */
      if (us_fresh) {
        if (us_mm >= st->arg - AR_TOP_TURN_LEAD_MM) { if (n->us_close < 255U) n->us_close++; }
        else n->us_close = 0;
      }
      if (n->us_close >= AR_US_CONFIRM || travelled >= cm_to_edges(AR_TOP_TURN_MAX_CM)) {
        n->top_back_edges = travelled;
        n->top_back_set = true;
        return route_next(n, now);
      }
#if AR_ROUTE_USE_IR
      return reverse_cmd(s);
#else
      return straight_cmd(n, -1, now, AR_SLOW_SPEED_PCT);
#endif
    }
    case RS_BACK_MEASURED: /* reverse exactly what the right-key approach travelled */
#if AR_ROUTE_USE_IR
      if (travelled >= n->leg_edges) return route_next(n, now);
      return reverse_cmd(s);
#else
      if (travelled + reverse_stop_edges() >= n->leg_edges) return route_next(n, now);
      return straight_cmd(n, -1, now, leg_speed(n->leg_edges - travelled, n->leg_edges));
#endif
    case RS_BACK_ZERO:
    case RS_BACK_CM: {
#if AR_ROUTE_USE_IR
      const uint32_t target = st->type == RS_BACK_ZERO ? n->out_edges : cm_to_edges(st->arg);
#else
      const uint32_t target = cm_to_edges(st->type == RS_BACK_ZERO ? AR_MID_BACK_CM : st->arg);
#endif
#if AR_ROUTE_USE_IR
      if (travelled >= target) return route_next(n, now);
      return reverse_cmd(s);
#else
      if (travelled + reverse_stop_edges() >= target) return route_next(n, now);
      return straight_cmd(n, -1, now, leg_speed(target - travelled, target));
#endif
    }
    case RS_FOLLOW_CM:
#if AR_ROUTE_USE_IR
      if (travelled >= cm_to_edges(st->arg)) return route_next(n, now);
#else
      if (travelled + coast_edges() >= cm_to_edges(st->arg)) return route_next(n, now);
#endif
#if AR_HOME_WALL_MM > 0
      if (n->step == HOME_TOP_END) { /* last leg east: also stop at the east wall (front US) */
        if (us_fresh) {
          if (us_mm >= 0 && us_mm <= AR_HOME_WALL_MM) { if (n->us_close < 255U) n->us_close++; }
          else n->us_close = 0;
        }
        if (n->us_close >= AR_US_CONFIRM) return route_next(n, now);
      }
#endif
#if !AR_ROUTE_USE_IR
      return straight_cmd(n, +1, now, leg_speed(cm_to_edges(st->arg) - travelled, cm_to_edges(st->arg)));
#endif
      if (n->lost && since(now, n->lost_since) >= AR_LOST_FAULT_MS) return fault(n, AR_FAULT_LOST, now);
      if (!any) return search(n);
      return follow(n, s, AR_FOLLOW_NORMAL);
    case RS_PIVOT: {
      const int deg = st->arg < 0 ? -st->arg : st->arg;
      const uint32_t min_ms = (uint32_t)deg * 1000U / (2U * AR_PIVOT_DEG_PER_S);
#if AR_ROUTE_USE_IR
      if (pivot_done_min(n, s, now, min_ms)) return route_next(n, now);
#else
      (void)min_ms;
      const uint32_t target = pivot_edges(deg);
      const uint32_t stop_at = target > AR_PIVOT_EARLY_EDGES ? target - AR_PIVOT_EARLY_EDGES : 0u;
      if (travelled >= stop_at) return route_next(n, now);
      if (since(now, n->sub_since) >= AR_PIVOT_MAX_MS) return fault(n, AR_FAULT_TURN, now);
      if (target - travelled < target * AR_PIVOT_SLOW_ZONE_PCT / 100U) /* last stretch: slower, less coast */
        return pivot_bal_cmd(n, st->arg < 0 ? -1 : +1, now, AR_PIVOT_SLOW_PCT);
      return pivot_bal_cmd(n, st->arg < 0 ? -1 : +1, now, AR_PIVOT_PCT);
#endif
      if (since(now, n->sub_since) >= AR_PIVOT_MAX_MS) return fault(n, AR_FAULT_TURN, now);
      return pivot_cmd(st->arg < 0 ? -1 : +1);
    }
  }
  return stop_cmd();
}

AR_Cmd AR_Nav_Step(AR_Nav *n, uint32_t now, AR_Line s, int32_t us_mm, bool us_fresh) {
  const bool any = s.l || s.c || s.r;
  if (any) {
    n->lost = false;
    n->leg_seen_line = true;
  } else if (!n->lost) {
    n->lost = true;
    n->lost_since = now;
  }

  switch (n->phase) {
    case AR_PH_IDLE:
    case AR_PH_DONE:
    case AR_PH_FAULT:
      return stop_cmd();

    case AR_PH_COUNTDOWN:
      if ((int32_t)(now - n->countdown_end) >= 0) {
        n->leg = 0;
        begin_leg(n, now);
        if (n->key >= AR_KEY_MID) {
          n->step = route_first_step(n);
          n->out_edges = n->leg_edges = n->top_edges = n->top_back_edges = 0;
          n->top_back_set = false;
          route_begin_step(n, now);
        }
        enter_phase(n, AR_PH_OUT, now);
      }
      return stop_cmd();

    case AR_PH_OUT:
    case AR_PH_RETURN: {
      if (n->phase == AR_PH_OUT && n->key >= AR_KEY_MID) return route_run(n, now, s, us_mm, us_fresh);
      const bool returning = (n->phase == AR_PH_RETURN);
      const AR_Route route = route_for(n, returning);
      if (n->leg >= route.count)
        return fault(n, AR_FAULT_LOST, now);
      const AR_Leg *leg = &route.legs[n->leg];

      if (n->sub == AR_SUB_ADVANCE) {
        if (since(now, n->sub_since) >= AR_CROSS_ADVANCE_MS) {
          n->sub = AR_SUB_PIVOT;
          n->sub_since = now;
          n->pivot_seen_off = false;
          n->pivot_dir = leg->pivot_dir;
        }
        return cmd(AR_BASE_PCT, AR_BASE_PCT);
      }
      if (n->sub == AR_SUB_PIVOT) {
        if (pivot_done(n, s, now)) {
          n->leg++;
          begin_leg(n, now);
          n->leg_seen_line = true;
          return follow(n, s, route.legs[n->leg < route.count ? n->leg : 0].mode);
        }
        if (since(now, n->sub_since) >= AR_PIVOT_MAX_MS)
          return fault(n, AR_FAULT_TURN, now);
        return pivot_cmd(n->pivot_dir);
      }

      /* AR_SUB_FOLLOW */
      if (leg->end == AR_END_CROSS) {
        if (s.l && s.c && s.r) {
          if (!n->cross_active) {
            n->cross_active = true;
            n->cross_since = now;
          }
          if (since(now, n->cross_since) >= AR_CROSS_MS) {
            n->sub = AR_SUB_ADVANCE;
            n->sub_since = now;
            return cmd(AR_BASE_PCT, AR_BASE_PCT);
          }
        } else {
          n->cross_active = false;
        }
      }
      if (leg->end == AR_END_KEY && n->leg_seen_line) {
        if (us_fresh) {
          if (us_mm >= 0 && us_mm <= AR_STOP_MM) {
            if (n->us_close < 255U) n->us_close++;
          } else {
            n->us_close = 0;
          }
        }
        if (n->us_close >= AR_US_CONFIRM) {
          enter_phase(n, AR_PH_GRAB, now);
          return stop_cmd();
        }
        /* Losing the line alone cannot establish that a key was reached. */
        if (n->lost && since(now, n->lost_since) >= AR_END_LOST_MS)
          return fault(n, AR_FAULT_LOST, now);
      }
      if (leg->end == AR_END_HOME && n->leg_seen_line && n->lost && since(now, n->lost_since) >= AR_HOME_LOST_MS) {
        enter_phase(n, AR_PH_HOME_ADVANCE, now);
        return cmd(AR_BASE_PCT, AR_BASE_PCT);
      }
      if (n->lost && since(now, n->lost_since) >= AR_LOST_FAULT_MS)
        return fault(n, AR_FAULT_LOST, now);

      if (!any)
        return search(n);
      return follow(n, s, leg->mode);
    }

    case AR_PH_GRAB:
      if (since(now, n->phase_since) >= AR_GRAB_PAUSE_MS) {
        n->sub = AR_SUB_PIVOT;
        n->sub_since = now;
        n->pivot_seen_off = false;
        n->pivot_dir = AR_TURNAROUND_DIR;
        enter_phase(n, AR_PH_TURNAROUND, now);
      }
      return stop_cmd();

    case AR_PH_TURNAROUND:
      if (pivot_done(n, s, now)) {
        n->leg = 0;
        begin_leg(n, now);
        n->leg_seen_line = true;
        enter_phase(n, AR_PH_RETURN, now);
        return follow(n, s, route_for(n, true).legs[0].mode);
      }
      if (since(now, n->sub_since) >= AR_PIVOT_MAX_MS)
        return fault(n, AR_FAULT_TURN, now);
      return pivot_cmd(n->pivot_dir);

    case AR_PH_HOME_ADVANCE:
      if (since(now, n->phase_since) >= AR_HOME_ADVANCE_MS) {
        enter_phase(n, AR_PH_DONE, now);
        return stop_cmd();
      }
      return cmd(AR_BASE_PCT, AR_BASE_PCT);
  }
  return stop_cmd();
}

const char *AR_Nav_PhaseText(const AR_Nav *n) {
  switch (n->phase) {
    case AR_PH_IDLE: return "READY";
    case AR_PH_COUNTDOWN: return "STARTING";
    case AR_PH_OUT:
      if (n->key >= AR_KEY_MID) return n->step < ROUTE_COUNT ? route[n->step].name : "ROUTE END";
      return n->sub == AR_SUB_FOLLOW ? "TO KEY" : (n->sub == AR_SUB_ADVANCE ? "AT CROSS" : "TURNING");
    case AR_PH_GRAB: return "GRAB KEY";
    case AR_PH_TURNAROUND: return "TURN AROUND";
    case AR_PH_RETURN: return n->sub == AR_SUB_FOLLOW ? "HOME" : (n->sub == AR_SUB_ADVANCE ? "AT CROSS" : "TURNING");
    case AR_PH_HOME_ADVANCE: return "ENTERING ZONE";
    case AR_PH_DONE: return (n->key == AR_KEY_ALL || n->key < AR_KEY_MID || (AR_ROUTE_HOME && n->step >= ROUTE_MAIN_COUNT && n->step < TURN_TEST_STEP)) ? "DONE AT HOME" : "ROUTE PART DONE";
    case AR_PH_FAULT: return n->fault == AR_FAULT_TURN ? "FAULT TURN" : (n->fault == AR_FAULT_STEP ? "FAULT STEP" : "FAULT LOST");
  }
  return "?";
}
