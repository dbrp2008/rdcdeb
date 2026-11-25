#include "tr_auto.h"
#include "tr_linetest.h" /* search mode: finds the T with the line test's drift / junction logic */
#include <math.h>

static TRLine s_find; /* one routine runs at a time */

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static uint32_t since(uint32_t now, uint32_t t) { return (uint32_t)(now - t); }

static void enter(TRAuto *a, TRAuto_State s, const TRAuto_In *in) {
  a->st = s;
  a->since = in->now;
  a->base_mm = in->dist_mm;
  a->us_close = 0;
  a->us_n = 0;
  a->junc_active = false;
  a->turn_requested = false;
  a->turn_seen_run = false;
  a->anchored = false;
  a->settle_dist = in->dist_mm;
  a->settle_strafe = in->strafe_mm;
  a->settle_since = in->now;
}

void TRAuto_Init(TRAuto *a) {
  a->st = TRA_IDLE;
  a->t0 = a->since = a->countdown_end = a->junc_since = 0;
  a->base_mm = 0.0f;
  a->hdg_target = 0.0f;
  a->shift_base = a->shift_done = 0.0f;
  a->us_close = 0;
  a->us_n = 0;
  a->junc_active = a->turn_requested = a->turn_seen_run = a->anchored = false;
  a->wrong_junctions = 0;
  a->t_found_mm = 0;
  a->t_by = 0;
  a->align_phase = 0;
  a->align_seen = 0.0f;
  a->align_failed = false;
  a->align_mm = 0;
  a->why = "";
  a->fault_mm = 0;
  a->l_n = a->r_n = 0;
  a->diag_prev_l = a->diag_prev_r = true;
  a->search_drifts = 0;
  a->settle_dist = a->settle_strafe = 0.0f;
  a->settle_since = 0;
  if (a->mode != 2U) a->mode = 1U; /* the mode survives a reset */
  TRLine_Init(&s_find);
}

void TRAuto_SetMode(TRAuto *a, uint8_t mode) {
  if (TRAuto_IsRunning(a)) return;
  a->mode = (mode == 2U) ? 2U : 1U;
}

void TRAuto_Start(TRAuto *a, uint32_t now) {
  if (a->st != TRA_IDLE && a->st != TRA_DONE && a->st != TRA_TIMEUP && a->st != TRA_FAULT)
    return;
  a->st = TRA_COUNTDOWN;
  a->countdown_end = now + TRA_COUNTDOWN_MS;
  a->wrong_junctions = 0;
  a->t_found_mm = 0;
  a->t_by = 0;
  a->align_phase = 0;
  a->align_seen = 0.0f;
  a->align_failed = false;
  a->align_mm = 0;
  a->why = "";
  a->fault_mm = 0;
  a->l_n = a->r_n = 0;
  a->diag_prev_l = a->diag_prev_r = true;
  a->search_drifts = 0;
}

void TRAuto_Abort(TRAuto *a) { a->st = TRA_IDLE; }

bool TRAuto_NeedsUs(const TRAuto *a) { /* the block approaches; elsewhere the routine ignores the ultrasonic */
  return a->st == TRA_COUNTDOWN || a->st == TRA_NORTH || a->st == TRA_GRAB1 || a->st == TRA_CARRY || a->st == TRA_TURN3 ||
         a->st == TRA_WEST || a->st == TRA_GRAB2 || a->st == TRA_SOUTH || a->st == TRA_ADV2 || a->st == TRB_NORTH ||
         a->st == TRB_GRAB;
}

bool TRAuto_IsRunning(const TRAuto *a) {
  return a->st != TRA_IDLE && a->st != TRA_DONE && a->st != TRA_TIMEUP && a->st != TRA_FAULT;
}

/* Strafe to keep the line between the two sensors, and hold the heading. */
static void follow(const TRAuto *a, const TRAuto_In *in, float speed, TRAuto_Out *o) {
  float x = 0.0f;
  if (in->left && !in->right) x = -TRA_KX;      /* line on the left: strafe left */
  else if (in->right && !in->left) x = TRA_KX;  /* line on the right: strafe right */
  o->x = x;
  o->y = speed;
  const float err = a->hdg_target - in->heading_deg; /* + = need to turn clockwise */
  o->w = in->heading_sign * clampf(TRA_KH * err, -TRA_KH_MAX, TRA_KH_MAX);
}

/* Hold the heading only (used while sliding sideways). */
static void hold_heading(const TRAuto *a, const TRAuto_In *in, TRAuto_Out *o) {
  const float err = a->hdg_target - in->heading_deg;
  o->w = in->heading_sign * clampf(TRA_KH * err, -TRA_KH_MAX, TRA_KH_MAX);
}

static float approach_speed(const TRAuto_In *in) {
  return (in->us_mm >= 0 && in->us_mm <= TRA_SLOW_MM) ? TRA_SLOW_SPEED : TRA_SPEED;
}

/* Median of the last 3 fresh ultrasonic readings, compared with the pickup distance (plus a lead for the delay). */
static bool block_seen(TRAuto *a, const TRAuto_In *in) {
  if (in->us_fresh) {
    a->us_hist[0] = a->us_hist[1];
    a->us_hist[1] = a->us_hist[2];
    a->us_hist[2] = (in->us_mm >= 0) ? in->us_mm : 9999; /* no echo counts as far away */
    if (a->us_n < 3U) a->us_n++;
    if (a->us_n >= 3U) {
      int32_t h0 = a->us_hist[0], h1 = a->us_hist[1], h2 = a->us_hist[2], med;
      if ((h0 <= h1 && h1 <= h2) || (h2 <= h1 && h1 <= h0)) med = h1;
      else if ((h1 <= h0 && h0 <= h2) || (h2 <= h0 && h0 <= h1)) med = h0;
      else med = h2;
      if (med <= TRA_PICKUP_MM + TRA_US_LEAD_MM) {
        if (a->us_close < 255U) a->us_close++;
      } else {
        a->us_close = 0;
      }
    }
  }
  return a->us_close >= TRA_US_CONFIRM;
}

/* A junction: the given IR sees the line (and the other does not) for TRA_JUNCTION_MS, inside the distance window. */
static bool junction(TRAuto *a, const TRAuto_In *in, bool sensor, bool other, float d, int min_mm, int max_mm) {
  if (d >= (float)max_mm)
    return true; /* dead reckoning fallback */
  if (d >= (float)min_mm && sensor && !other) {
    if (!a->junc_active) {
      a->junc_active = true;
      a->junc_since = in->now;
    }
    return since(in->now, a->junc_since) >= TRA_JUNCTION_MS;
  }
  a->junc_active = false;
  return false;
}

/* Run a heading turn: request it once, wait for it to finish. Returns 1 done, 0 busy, -1 failed. */
/* mm the TR still rolls after the command drops from ratio s to 0 with the normal ramp (TR_ACCEL 600 rpm/s, 152 mm wheels) */
static float brake_mm(float s) {
  const float rpm = s * TRA_TRANSLATE_RPM;
  return rpm * rpm * (3.14159265f * 152.0f / 60.0f) / (2.0f * 600.0f);
}

/* Stop with a reason the screen can show (why, where, and what the IRs had seen). */
static void fail(TRAuto *a, const char *why, float d) {
  a->st = TRA_FAULT;
  a->why = why;
  a->fault_mm = (int16_t)(d > 32767.0f ? 32767.0f : (d < -32767.0f ? -32767.0f : d));
}
static const char *turn_fail_text(uint8_t turn_state, int which) {
  static const char *const txt[3][3] = {{"TURN1 ABORTED", "TURN1 NO MOTION", "TURN1 NEVER RAN"},
                                        {"TURN2 ABORTED", "TURN2 NO MOTION", "TURN2 NEVER RAN"},
                                        {"TURN3 ABORTED", "TURN3 NO MOTION", "TURN3 NEVER RAN"}};
  return txt[which][turn_state == 3U ? 0 : (turn_state == 4U ? 1 : 2)];
}

/* 1 = done, 0 = still turning, -1 = failed.
 * Before the turn starts the TR must be STANDING STILL: once a turn runs the drive stops commanding forward motion but ramps
 * at the gentle turn rate (200 rpm/s), so a TR still moving at speed level 5 coasted ~12 cm past the junction while it
 * turned (the overshoot seen on the field). So: command zero, hold the heading, wait until the encoders show no movement. */
static int do_turn(TRAuto *a, const TRAuto_In *in, TRAuto_Out *o, float delta_deg) {
  if (!a->turn_requested) {
    o->x = o->y = 0.0f;
    hold_heading(a, in, o);
    const float moved = fabsf(in->dist_mm - a->settle_dist) + fabsf(in->strafe_mm - a->settle_strafe);
    if (moved > TRA_STILL_MM) { /* moved since the reference: it is still rolling, restart the still-time from here */
      a->settle_dist = in->dist_mm;
      a->settle_strafe = in->strafe_mm;
      a->settle_since = in->now;
    }
    if (since(in->now, a->settle_since) < TRA_STILL_MS && since(in->now, a->since) < TRA_SETTLE_MAX_MS)
      return 0; /* still rolling */
    a->turn_requested = true;
    a->since = in->now; /* the "turn never started" timeout counts from the request */
    o->turn_req = true;
    o->turn_deg = delta_deg;
    return 0;
  }
  if (in->turn_state == 1U) {
    a->turn_seen_run = true;
    return 0;
  }
  if (a->turn_seen_run) {
    if (in->turn_state == 2U) {
      a->hdg_target = in->heading_deg;
      return 1;
    }
    return -1;
  }
  if (since(in->now, a->since) > 1000U && in->turn_state != 1U)
    return -1; /* the turn never started */
  return 0;
}

/* FIND T: follow slowly until a T on the LEFT is found. Backup = the ORIGINAL rule (left IR black, right IR white for
 * TRA_JUNCTION_MS) from backup_from on. 1 = found (a->found_at = forward position of the branch centre), 0 = still looking,
 * -1 = not found by max_mm (a->why set). */
static int search_step(TRAuto *a, const TRAuto_In *in, TRAuto_Out *o, float d, int backup_from, int max_mm, float speed) {
  if (d >= (float)backup_from && in->left && !in->right) {
    if (!a->junc_active) { a->junc_active = true; a->junc_since = in->now; a->junc_mm = in->dist_mm; }
    if (since(in->now, a->junc_since) >= TRA_JUNCTION_MS) {
      a->found_at = a->junc_mm + 10.0f; /* the left IR first saw the branch's near edge, ~half a 20 mm tape earlier */
      a->t_found_mm = (int16_t)(a->junc_mm - a->base_mm); /* screen: where the T was taken, and by which rule */
      a->t_by = 1;
      return 1;
    }
  } else {
    a->junc_active = false;
  }
  TRLine_Step(&s_find, in, o);
  o->active = true;
  a->search_drifts = s_find.drifts;
  if (s_find.st == TRL_FOUND) {
    if (s_find.side < 0) {
      a->found_at = s_find.junc_mm;
      a->t_found_mm = (int16_t)(s_find.junc_mm - a->base_mm);
      a->t_by = 2;
      return 1;
    }
    if (a->wrong_junctions < 255U) a->wrong_junctions++; /* a "junction" on the right: there is none at this T */
    TRLine_Search(&s_find, in, a->hdg_target, speed);
    o->y = speed;
    return 0;
  }
  if (!TRLine_IsRunning(&s_find) || d >= (float)max_mm) {
    a->why = a->wrong_junctions ? "NO T:R JUNC SEEN" : "NO T FOUND";
    return -1;
  }
  return 0;
}

/* FIND LINE: slide sideways until the line is between the IRs (the IRs are only ~5 cm apart: a turn a few cm off leaves
 * the line outside them, the follow then never sees it and drives BESIDE the line). Slides to a->align_first (-1 left,
 * +1 right) first, then the other way, up to TRA_ALIGN_MAX_MM each. Call align_begin first. 1 = done. */
static void align_begin(TRAuto *a, const TRAuto_In *in, int8_t first) {
  a->align_phase = 0;
  a->align_first = first;
  a->shift_base = in->strafe_mm;
}
static int align_step(TRAuto *a, const TRAuto_In *in, TRAuto_Out *o) {
  const int8_t f = a->align_first;
  const float moved = (in->strafe_mm - a->shift_base) * (float)f; /* + = toward the first side */
  const bool near_ir = f < 0 ? in->left : in->right;              /* the IR on the side it slides toward */
  const bool far_ir = f < 0 ? in->right : in->left;
  hold_heading(a, in, o);
  o->x = 0.0f;
  int done = 0;
  switch (a->align_phase) {
    case 0: /* slide toward the first side until an IR sees the line */
      if (far_ir && !near_ir) { done = 1; break; }                            /* the line was between the IRs */
      if (near_ir) { a->align_phase = 1; a->align_seen = in->strafe_mm; }     /* on this side: slide over it */
      else if (moved >= (float)TRA_ALIGN_MAX_MM) a->align_phase = 2;          /* not here: try the other side */
      o->x = (float)f * TRA_ALIGN_SPEED;
      break;
    case 1: /* keep sliding until that IR is past the line: it is then between the IRs */
      if (near_ir) a->align_seen = in->strafe_mm;
      if (!near_ir && fabsf(in->strafe_mm - a->align_seen) >= 3.0f) { done = 1; break; }
      o->x = (float)f * TRA_ALIGN_SPEED;
      break;
    case 2: /* slide the other way, up to TRA_ALIGN_MAX_MM past the start */
      if (near_ir && !far_ir) { done = 1; break; }
      if (far_ir) { a->align_phase = 3; a->align_seen = in->strafe_mm; }
      else if (-moved >= (float)TRA_ALIGN_MAX_MM) { a->align_failed = true; done = 1; break; } /* not found: go on */
      o->x = -(float)f * TRA_ALIGN_SPEED;
      break;
    default:
      if (far_ir) a->align_seen = in->strafe_mm;
      if (!far_ir && fabsf(in->strafe_mm - a->align_seen) >= 3.0f) { done = 1; break; }
      o->x = -(float)f * TRA_ALIGN_SPEED;
      break;
  }
  if (done) { o->x = 0.0f; a->align_mm = (int16_t)((in->strafe_mm - a->shift_base)); }
  return done;
}

/* MOVE: drive a straight path relative to where it starts, fwd mm forward and side mm to the RIGHT (omni wheels: no
 * turning), heading held, up to vmax, slowing so it can stop on the spot. Call move_begin first. 1 = arrived. */
static void move_begin(TRAuto *a, const TRAuto_In *in, float fwd, float side) {
  a->mv_f0 = in->dist_mm;
  a->mv_s0 = in->strafe_mm;
  a->mv_fwd = fwd;
  a->mv_side = side;
}
static int move_step(TRAuto *a, const TRAuto_In *in, TRAuto_Out *o, float vmax) {
  const float rf = a->mv_fwd - (in->dist_mm - a->mv_f0), rs = a->mv_side - (in->strafe_mm - a->mv_s0);
  const float mag = sqrtf(rf * rf + rs * rs);
  hold_heading(a, in, o);
  if (mag <= 8.0f) { o->x = o->y = 0.0f; return 1; }
  float s = sqrtf(mag * 2.0f * 600.0f / (3.14159265f * 152.0f / 60.0f)) / TRA_TRANSLATE_RPM * 0.8f; /* can still stop in mag */
  if (s > vmax) s = vmax;
  if (s < 0.06f) s = 0.06f;
  o->y = rf / mag * s;
  o->x = rs / mag * s;
  return 0;
}

void TRAuto_Step(TRAuto *a, const TRAuto_In *in, TRAuto_Out *o) {
  o->active = false;
  o->x = o->y = o->w = 0.0f;
  o->arm = 0;
  o->turn_req = false;
  o->turn_deg = 0.0f;
  o->zero_heading = false;

  if (a->st == TRA_IDLE || a->st == TRA_DONE || a->st == TRA_TIMEUP || a->st == TRA_FAULT)
    return;

  if (a->st == TRA_COUNTDOWN) {
    if ((int32_t)(in->now - a->countdown_end) >= 0) {
      a->t0 = in->now;
      a->hdg_target = 0.0f;
      o->zero_heading = true; /* the heading at the start (facing the line) is 0 */
      enter(a, a->mode == 2U ? TRB_EAST : TRA_EAST, in);
    }
    return;
  }

  if (since(in->now, a->t0) >= (a->mode == 2U ? TRB_TIME_LIMIT_MS : TRA_TIME_LIMIT_MS)) { /* time is up: stop, keep whatever the arm holds */
    a->st = TRA_TIMEUP;
    return;
  }

  o->active = true;
  const float d = in->dist_mm - a->base_mm;
  int t;

  if ((a->st == TRA_EAST || a->st == TRA_SEARCH1 || a->st == TRB_EAST || a->st == TRB_SEARCH) && d >= (float)TRA_DIAG_FROM_MM) {
    /* diagnostics: the LAST 3 places where each IR went from white to black (where the T, drifts and crossings were) */
    if (in->left && !a->diag_prev_l) { a->l_on[a->l_n % 3u] = (int16_t)d; a->l_n++; }
    if (in->right && !a->diag_prev_r) { a->r_on[a->r_n % 3u] = (int16_t)d; a->r_n++; }
  }
  a->diag_prev_l = in->left;
  a->diag_prev_r = in->right;

  switch (a->st) {
    case TRA_EAST: /* fast out of the start area: nothing to find before TRA_J1_SEARCH_MM */
      follow(a, in, TRA_EAST_SPEED, o);
      if (d >= (float)TRA_J1_SEARCH_MM) {
        a->st = TRA_SEARCH1; /* keeps base_mm: d is still measured from the start */
        a->since = in->now;
        TRLine_Search(&s_find, in, a->hdg_target, TRA_SEARCH_SPEED);
      }
      break;

    case TRA_SEARCH1: { /* follow slowly until the T on the LEFT is found */
      const int r = search_step(a, in, o, d, TRA_JUNC1_MIN_MM, TRA_JUNC1_MAX_MM, TRA_SEARCH_SPEED);
      if (r > 0) {
        const float at = a->found_at;
        enter(a, TRA_ADV1, in);
        a->base_mm = at; /* advance from the branch centre */
        o->active = true;
        o->y = TRA_SEARCH_SPEED;
        hold_heading(a, in, o);
      } else if (r < 0) {
        fail(a, a->why, d); /* never found the T: STOP here rather than drive on / turn blindly (the opponent's half is ahead) */
      }
      break;
    }

    case TRA_ADV1: /* slowly (search speed), and stop commanding early by the braking distance: the centre stops over the T */
      o->y = TRA_SEARCH_SPEED;
      hold_heading(a, in, o);
      if (d >= (float)TRA_ADVANCE_MM - brake_mm(TRA_SEARCH_SPEED))
        enter(a, TRA_TURN1, in);
      break;

    case TRA_TURN1:
      t = do_turn(a, in, o, -90.0f); /* left */
      if (t > 0) { enter(a, TRA_ALIGN1, in); align_begin(a, in, -1); } /* past the T = the line on the left */
      else if (t < 0) fail(a, turn_fail_text(in->turn_state, 0), d);
      break;

    case TRA_ALIGN1: /* FIND LINE before driving north */
      if (align_step(a, in, o)) enter(a, TRA_NORTH, in);
      break;

    case TRA_NORTH:
      follow(a, in, approach_speed(in), o);
      if (block_seen(a, in))
        enter(a, TRA_GRAB1, in);
      else if (d >= (float)TRA_NORTH_MAX_MM)
        fail(a, "NO BLOCK 1 (US)", d);
      break;

    case TRA_GRAB1:
      o->arm = 1; /* close */
      if (in->arm_us <= (uint16_t)(in->arm_closed_us + 15U) || since(in->now, a->since) >= TRA_GRAB_MS)
        enter(a, TRA_CARRY, in);
      break;

    case TRA_CARRY:
      follow(a, in, TRA_SPEED, o);
      if (!a->anchored) {
        /* checkpoint J2: the left IR crosses the short horizontal line */
        if (junction(a, in, in->left, in->right, d, TRA_J2_MIN_MM, TRA_J2_MAX_MM)) {
          a->anchored = true;
          a->base_mm = in->dist_mm; /* distances are now measured from the checkpoint */
        }
      } else if (d >= (float)TRA_J2_DROP_MM) {
        a->shift_done = 0.0f;
        if (TRA_DROP_SHIFT_MM > 0) {
          enter(a, TRA_SHIFT, in);
          a->shift_base = in->strafe_mm;
        } else {
          enter(a, TRA_DROP, in);
        }
      }
      break;

    case TRA_SHIFT: { /* slide sideways into one scoring area, heading held */
      const float moved = (in->strafe_mm - a->shift_base) * (float)TRA_DROP_SHIFT_DIR;
      o->x = (float)TRA_DROP_SHIFT_DIR * TRA_SHIFT_SPEED;
      hold_heading(a, in, o);
      if (moved >= (float)TRA_DROP_SHIFT_MM || since(in->now, a->since) >= TRA_SHIFT_MAX_MS) {
        a->shift_done = moved > 0.0f ? moved : 0.0f;
        enter(a, TRA_DROP, in);
      }
      break;
    }

    case TRA_DROP:
      o->arm = -1; /* open: release the block */
      if (in->arm_us + 15U >= in->arm_open_us || since(in->now, a->since) >= TRA_DROP_MS) {
        if (a->shift_done > 0.0f) {
          enter(a, TRA_UNSHIFT, in);
          a->shift_base = in->strafe_mm;
        } else {
          enter(a, TRA_TURN2, in);
        }
      }
      break;

    case TRA_UNSHIFT: { /* slide back onto the line for the turn-around */
      const float back = (a->shift_base - in->strafe_mm) * (float)TRA_DROP_SHIFT_DIR;
      o->x = -(float)TRA_DROP_SHIFT_DIR * TRA_SHIFT_SPEED;
      hold_heading(a, in, o);
      if (back >= a->shift_done - 3.0f || since(in->now, a->since) >= TRA_SHIFT_MAX_MS) {
        a->shift_done = 0.0f;
        enter(a, TRA_TURN2, in);
      }
      break;
    }

    case TRA_TURN2:
      t = do_turn(a, in, o, 180.0f);
      if (t > 0) enter(a, TRA_SOUTH, in);
      else if (t < 0) fail(a, turn_fail_text(in->turn_state, 1), d);
      break;

    case TRA_SOUTH:
      follow(a, in, TRA_SPEED, o);
#if TRA_SOUTH_USE_SENSOR
      /* heading south: the short line leaves to the west, which is on the RIGHT */
      if (junction(a, in, in->right, in->left, d, TRA_SOUTH_MIN_MM, TRA_SOUTH_MAX_MM))
        enter(a, TRA_ADV2, in);
#else
      /* the sensors are already past J2 after the turn-around: bring the centre over J2 by distance, then turn */
      if (d >= (float)TRA_SOUTH_DIRECT_MM - brake_mm(TRA_SPEED)) /* it rolls brake_mm more while stopping */
        enter(a, TRA_TURN3, in);
#endif
      break;

    case TRA_ADV2:
      o->y = TRA_SPEED;
      if (d >= (float)TRA_ADVANCE_MM - brake_mm(TRA_SPEED))
        enter(a, TRA_TURN3, in);
      break;

    case TRA_TURN3:
      t = do_turn(a, in, o, 90.0f); /* right, to face west */
      if (t > 0) enter(a, TRA_WEST, in);
      else if (t < 0) fail(a, turn_fail_text(in->turn_state, 2), d);
      break;

    case TRA_WEST:
      follow(a, in, approach_speed(in), o);
      if (block_seen(a, in))
        enter(a, TRA_GRAB2, in);
      else if (d >= (float)TRA_WEST_MAX_MM)
        fail(a, "NO BLOCK 2 (US)", d);
      break;

    case TRA_GRAB2:
      o->arm = 1;
      if (in->arm_us <= (uint16_t)(in->arm_closed_us + 15U) || since(in->now, a->since) >= TRA_GRAB_MS)
        a->st = TRA_DONE;
      break;

    /* ---------------- MODE 2: the two far-right blocks into the bottom-right scoring area ---------------- */
    case TRB_EAST: /* the long straight along the bottom line, fast, past the first T */
      follow(a, in, TRB_FAST_SPEED, o);
      if (d >= (float)TRB_T2_SEARCH_MM) {
        a->st = TRB_SEARCH; /* keeps base_mm */
        a->since = in->now;
        TRLine_Search(&s_find, in, a->hdg_target, TRA_SEARCH_SPEED);
      }
      break;

    case TRB_SEARCH: { /* the second T (left) */
      const int r = search_step(a, in, o, d, TRB_T2_MIN_MM, TRB_T2_MAX_MM, TRA_SEARCH_SPEED);
      if (r > 0) {
        const float at = a->found_at;
        enter(a, TRB_ADV, in);
        a->base_mm = at;
        o->active = true;
        o->y = TRA_SEARCH_SPEED;
        hold_heading(a, in, o);
      } else if (r < 0) {
        fail(a, a->why, d);
      }
      break;
    }

    case TRB_ADV: /* centre over the second T */
      o->y = TRA_SEARCH_SPEED;
      hold_heading(a, in, o);
      if (d >= (float)TRA_ADVANCE_MM - brake_mm(TRA_SEARCH_SPEED))
        enter(a, TRB_TURN1, in);
      break;

    case TRB_TURN1: /* face north (in the open: the right wall is too close to turn later) */
      t = do_turn(a, in, o, -90.0f);
      if (t > 0) { enter(a, TRB_MOVE1, in); move_begin(a, in, (float)TRB_MOVE1_FWD_MM, (float)TRB_MOVE1_RIGHT_MM); }
      else if (t < 0) fail(a, turn_fail_text(in->turn_state, 0), d);
      break;

    case TRB_MOVE1: /* straight diagonal, still facing north, to the far-right vertical line */
      if (move_step(a, in, o, TRB_MOVE_SPEED)) { enter(a, TRB_ALIGN1, in); align_begin(a, in, -1); }
      break;

    case TRB_ALIGN1:
      if (align_step(a, in, o)) enter(a, TRB_NORTH, in);
      break;

    case TRB_NORTH: /* up the line to the top block */
      follow(a, in, approach_speed(in), o);
      if (block_seen(a, in))
        enter(a, TRB_GRAB, in);
      else if (d >= (float)TRB_NORTH_MAX_MM)
        fail(a, "NO TOP BLOCK (US)", d);
      break;

    case TRB_GRAB:
      o->arm = 1; /* close */
      if (in->arm_us <= (uint16_t)(in->arm_closed_us + 15U) || since(in->now, a->since) >= TRA_GRAB_MS) {
        enter(a, TRB_MOVE2, in);
        move_begin(a, in, -(float)TRB_CLEAR_BACK_MM, -(float)TRB_CLEAR_SIDE_MM); /* back (south) and left (west) */
      }
      break;

    case TRB_MOVE2: /* clear of the barrier and the right wall so it can turn round with the block */
      if (move_step(a, in, o, 0.5f)) enter(a, TRB_TURN2, in);
      break;

    case TRB_TURN2:
      t = do_turn(a, in, o, 180.0f);
      if (t > 0) { enter(a, TRB_MOVE3, in); move_begin(a, in, 0.0f, -(float)TRB_CLEAR_SIDE_MM); } /* facing south: east = left */
      else if (t < 0) fail(a, turn_fail_text(in->turn_state, 1), d);
      break;

    case TRB_MOVE3: /* back east onto the line */
      if (move_step(a, in, o, 0.5f)) { enter(a, TRB_ALIGN2, in); align_begin(a, in, -1); }
      break;

    case TRB_ALIGN2:
      if (align_step(a, in, o)) enter(a, TRB_SOUTH, in);
      break;

    case TRB_SOUTH: /* follow the line south (the carried block pushes the lower block), then on into the scoring area */
      follow(a, in, TRB_PUSH_SPEED, o);
      if (d >= (float)TRB_SOUTH_MM - brake_mm(TRB_PUSH_SPEED)) enter(a, TRB_DROP, in);
      break;

    case TRB_DROP:
      o->arm = -1; /* open: release the block */
      if (in->arm_us + 15U >= in->arm_open_us || since(in->now, a->since) >= TRA_DROP_MS) a->st = TRA_DONE;
      break;

    default:
      break;
  }
  if (a->st == TRA_FAULT || a->st == TRA_DONE) { /* stop at once */
    o->active = (a->st == TRA_DONE) ? true : false;
    o->x = o->y = o->w = 0.0f;
    if (a->st == TRA_DONE) o->arm = a->mode == 2U ? -1 : 1; /* mode 2 ends with both blocks dropped in the SA */
  }
}

const char *TRAuto_StateText(TRAuto_State s) {
  switch (s) {
    case TRA_IDLE: return "READY";
    case TRA_COUNTDOWN: return "COUNTDOWN";
    case TRA_EAST: return "EAST";
    case TRA_SEARCH1: return "FIND T";
    case TRA_ADV1: return "ADVANCE";
    case TRA_TURN1: return "TURN LEFT";
    case TRA_ALIGN1: return "FIND LINE";
    case TRA_NORTH: return "NORTH";
    case TRA_GRAB1: return "GRAB 1";
    case TRA_CARRY: return "CARRY";
    case TRA_SHIFT: return "SHIFT";
    case TRA_DROP: return "DROP";
    case TRA_UNSHIFT: return "SHIFT BACK";
    case TRA_TURN2: return "TURN 180";
    case TRA_SOUTH: return "SOUTH";
    case TRA_ADV2: return "ADVANCE";
    case TRA_TURN3: return "TURN RIGHT";
    case TRA_WEST: return "WEST";
    case TRA_GRAB2: return "GRAB 2";
    case TRB_EAST: return "EAST FAST";
    case TRB_SEARCH: return "FIND T2";
    case TRB_ADV: return "ADVANCE";
    case TRB_TURN1: return "TURN LEFT";
    case TRB_MOVE1: return "DIAGONAL";
    case TRB_ALIGN1: return "FIND LINE";
    case TRB_NORTH: return "TO TOP BLOCK";
    case TRB_GRAB: return "GRAB TOP";
    case TRB_MOVE2: return "CLEAR WALL";
    case TRB_TURN2: return "TURN 180";
    case TRB_MOVE3: return "BACK TO LINE";
    case TRB_ALIGN2: return "FIND LINE";
    case TRB_SOUTH: return "PUSH SOUTH";
    case TRB_DROP: return "DROP";
    case TRA_DONE: return "DONE";
    case TRA_TIMEUP: return "TIME UP";
    case TRA_FAULT: return "FAULT STOP";
  }
  return "?";
}
