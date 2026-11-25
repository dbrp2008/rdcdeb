#include "tr_linetest.h"
#include <math.h>

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static uint32_t since(uint32_t now, uint32_t t) { return (uint32_t)(now - t); }

static void enter(TRLine *a, TRL_State s, const TRAuto_In *in) {
  a->st = s;
  a->phase = 0;
  a->since = in->now;
  a->base_mm = in->dist_mm;
  a->turn_requested = a->turn_seen_run = false;
}

void TRLine_Init(TRLine *a) {
  a->st = TRL_IDLE;
  a->speed = TRL_SPEED;
  a->search = false;
  a->junc_mm = 0.0f;
  a->t0 = a->since = a->countdown_end = 0;
  a->base_mm = a->hdg_target = a->hdg_ref = 0.0f;
  a->side = 0;
  a->clear_since = 0;
  a->leave_dark = false;
  a->single_mm = 0.0f;
  a->seen_mm = 0.0f;
  a->check_mm = 0.0f;
  a->both_mm = -1.0f;
  a->cross_mm = a->lb_left = a->lb_right = 0.0f;
  a->turn_requested = a->turn_seen_run = false;
  a->phase = 0;
  a->slide_base = 0.0f;
  a->gap_mm = 0.0f;
  a->us_close = 0;
  a->drifts = a->junctions = 0;
}

void TRLine_Start(TRLine *a, uint32_t now) {
  if (TRLine_IsRunning(a)) return;
  TRLine_Init(a);
  a->st = TRL_COUNTDOWN;
  a->countdown_end = now + TRA_COUNTDOWN_MS;
}

void TRLine_Search(TRLine *a, const TRAuto_In *in, float heading_deg, float speed) {
  TRLine_Init(a);
  a->search = true;
  a->speed = speed;
  a->t0 = in->now;
  a->hdg_target = a->hdg_ref = heading_deg;
  enter(a, TRL_FOLLOW, in);
}

void TRLine_Abort(TRLine *a) { a->st = TRL_IDLE; }

bool TRLine_IsRunning(const TRLine *a) {
  return a->st != TRL_IDLE && a->st != TRL_END && a->st != TRL_TIMEUP && a->st != TRL_FAULT && a->st != TRL_FOUND;
}

/* mm the TR slides after a sideways command of ratio s stops: v^2 / 2a, v = s x TRA_TRANSLATE_RPM rpm on 152 mm wheels,
 * a = 600 rpm/s on the same wheels */
static float brake_mm(float s) {
  const float rpm = s * TRA_TRANSLATE_RPM;
  return rpm * rpm * (3.14159265f * 152.0f / 60.0f) / (2.0f * 600.0f);
}
/* Slide speed: TRL_PROBE_SPEED, but slower when the measured gap is small (braking under a quarter of the gap). */
static float slide_speed(const TRLine *a) {
  float s = TRL_PROBE_SPEED;
  if (a->gap_mm > 0.0f) {
    const float s_gap = sqrtf(0.25f * a->gap_mm * 2.0f * 600.0f / (3.14159265f * 152.0f / 60.0f)) / TRA_TRANSLATE_RPM;
    if (s_gap < s) s = s_gap;
  }
  return s < TRL_PROBE_MIN ? TRL_PROBE_MIN : s;
}

/* A drift correction finished: count it, and turn the held heading a little toward the line (a drift to one side means
 * the TR is running slightly off the line's direction, e.g. the wheel-based heading estimate has drifted). */
static void drift_fixed(TRLine *a) {
  if (a->drifts < 65535U) a->drifts++;
  float t = a->hdg_target + (float)a->side * TRL_HDG_TRIM_DEG - a->hdg_ref;
  if (t > TRL_HDG_TRIM_MAX_DEG) t = TRL_HDG_TRIM_MAX_DEG;
  if (t < -TRL_HDG_TRIM_MAX_DEG) t = -TRL_HDG_TRIM_MAX_DEG;
  a->hdg_target = a->hdg_ref + t;
}

/* A junction toward a->side; the branch is at forward position base (the IRs were over it there) */
static void junction_found(TRLine *a, const TRAuto_In *in, float base) {
  if (a->junctions < 65535U) a->junctions++;
  if (a->search) { /* search mode: report it to the caller and stop here */
    a->junc_mm = base;
    a->st = TRL_FOUND;
    return;
  }
  enter(a, TRL_ADV, in);
  a->base_mm = base;
}

static void enter_cross(TRLine *a, const TRAuto_In *in) {
  enter(a, TRL_CROSS, in);
  a->cross_mm = a->lb_left = a->lb_right = in->dist_mm;
}

static void hold_heading(const TRLine *a, const TRAuto_In *in, TRAuto_Out *o) {
  const float err = a->hdg_target - in->heading_deg; /* + = need to turn clockwise */
  o->w = in->heading_sign * clampf(TRA_KH * err, -TRA_KH_MAX, TRA_KH_MAX);
}

/* 1 = done, 0 = still turning, -1 = failed (same protocol as the main routine) */
static int do_turn(TRLine *a, const TRAuto_In *in, TRAuto_Out *o, float delta_deg) {
  if (!a->turn_requested) {
    a->turn_requested = true;
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
      a->hdg_target = a->hdg_ref = in->heading_deg; /* the new leg's direction */
      return 1;
    }
    return -1;
  }
  if (since(in->now, a->since) > 1000U && in->turn_state != 1U) return -1;
  return 0;
}

void TRLine_Step(TRLine *a, const TRAuto_In *in, TRAuto_Out *o) {
  o->active = false;
  o->x = o->y = o->w = 0.0f;
  o->arm = 0;
  o->turn_req = false;
  o->turn_deg = 0.0f;
  o->zero_heading = false;

  if (!TRLine_IsRunning(a)) return;

  if (a->st == TRL_COUNTDOWN) {
    if ((int32_t)(in->now - a->countdown_end) >= 0) {
      a->t0 = in->now;
      a->hdg_target = a->hdg_ref = 0.0f;
      o->zero_heading = true; /* the heading at the start is 0 */
      a->clear_since = 0;
      a->leave_dark = false;
      a->side = 0;
      enter(a, TRL_LEAVE, in);
    }
    return;
  }
  if (!a->search && since(in->now, a->t0) >= TRL_TIME_LIMIT_MS) {
    a->st = TRL_TIMEUP;
    return;
  }

  o->active = true;
  if (in->us_fresh && a->st != TRL_TURN && !a->search) { /* wall or object ahead: stop (2 readings in a row) */
    if (in->us_mm > 0 && in->us_mm <= TRL_STOP_MM) { if (a->us_close < 255U) a->us_close++; }
    else a->us_close = 0;
  }
  if (a->us_close >= 2U) {
    a->st = TRL_END;
    o->x = o->y = o->w = 0.0f;
    o->active = false;
    return;
  }
  switch (a->st) {
    case TRL_LEAVE: /* drive straight until the IRs are past the start box: its dark fill and/or its black outline */
      hold_heading(a, in, o);
      o->y = a->speed;
      if (in->left || in->right) a->leave_dark = true;
      if (!in->left && !in->right) {
        if (a->clear_since == 0) a->clear_since = in->now ? in->now : 1u;
        /* clear on white -- but only after crossing something dark (the outline is always in front of the IRs at the start),
         * or after TRL_LEAVE_MAX_MM if it never saw any (started with the IRs already past the outline) */
        if (since(in->now, a->clear_since) >= TRL_CLEAR_MS &&
            (a->leave_dark || in->dist_mm - a->base_mm >= (float)TRL_LEAVE_MAX_MM)) enter(a, TRL_FOLLOW, in);
      } else {
        a->clear_since = 0;
        if (in->left != in->right) { /* one IR black: the outline at an angle, or the line itself under that IR */
          const int8_t s = in->left ? -1 : +1;
          if (a->side != s) { a->side = s; a->single_mm = in->dist_mm; }
          else if (in->dist_mm - a->single_mm >= (float)TRL_LEAVE_LINE_MM) { enter(a, TRL_FOLLOW, in); break; }
        } else {
          a->side = 0; /* both black: inside the dark box / on its outline */
        }
      }
      if (a->st == TRL_LEAVE && since(in->now, a->since) >= TRL_LEAVE_MAX_MS) enter(a, TRL_FOLLOW, in);
      break;

    case TRL_FOLLOW:
      hold_heading(a, in, o);
      if (in->left && in->right) { /* a crossing (or a branch while the other IR touches the main line): see which clears first */
        enter_cross(a, in);
        o->y = a->speed;
        break;
      }
      if (in->left || in->right) { /* one IR on black: a crossing at an angle, a drift or a junction? check first */
        a->side = in->left ? -1 : +1;
        enter(a, TRL_CHECK, in);
        a->check_mm = in->dist_mm; /* where this IR first saw black */
        a->seen_mm = 0.0f;
        a->both_mm = -1.0f;
        o->y = TRL_CHECK_SPEED;
        break;
      }
      o->y = a->speed; /* both on white: the line is between the IRs */
      break;

    case TRL_CHECK: { /* creep forward a little: does the other IR reach a line too (a crossing)? */
      hold_heading(a, in, o);
      const bool seen = (a->side < 0) ? in->left : in->right;
      const bool other = (a->side < 0) ? in->right : in->left;
      const float d = in->dist_mm - a->base_mm;
      if (seen) a->seen_mm = d;
      if (seen && other) {
        if (a->both_mm < 0.0f) a->both_mm = d;
      } else {
        a->both_mm = -1.0f; /* a single tick of both black is usually the far IR flickering at the main line's edge */
      }
      if (a->both_mm >= 0.0f && d - a->both_mm >= (float)TRL_BOTH_MM) {
        /* both IRs on black for a few mm: a crossing met at an angle (or a branch + the main line): see which clears first */
        const float back = d - a->both_mm;
        enter_cross(a, in);
        a->cross_mm -= back;
        o->y = a->speed;
        break;
      }
      if (d >= (float)TRL_CHECK_MM) {
        /* decided only here: still black (or black within the last TRL_FLICKER_MM: the IR flickers at a tape edge) -> stop and
         * probe sideways; white -> it passed the back edge of a crossing, or the line drifted back by itself: carry on */
        if (d - a->seen_mm < (float)TRL_FLICKER_MM && a->both_mm < 0.0f) {
          enter(a, TRL_PROBE, in);
          a->slide_base = in->strafe_mm; /* progress of this probe's slide is measured from here */
          o->y = 0.0f;
        } else {
          enter(a, TRL_FOLLOW, in);
          o->y = a->speed;
        }
        break;
      }
      o->y = TRL_CHECK_SPEED;
      break;
    }

    case TRL_CROSS: { /* both IRs went black: drive on and see which clears first */
      hold_heading(a, in, o);
      o->y = a->speed;
      const float d = in->dist_mm;
      if (in->left) a->lb_left = d;   /* where each IR last read black. An IR counts as clear only after TRL_CROSS_CLEAR_MM */
      if (in->right) a->lb_right = d; /* of solid white: tape edges flicker */
      const bool l_clear = d - a->lb_left >= (float)TRL_CROSS_CLEAR_MM, r_clear = d - a->lb_right >= (float)TRL_CROSS_CLEAR_MM;
      if (l_clear && r_clear) { enter(a, TRL_FOLLOW, in); break; } /* both clear: a crossing, passed */
      if (l_clear != r_clear) {
        const float cleared_at = l_clear ? a->lb_left : a->lb_right;
        if (d - cleared_at >= (float)TRL_CROSS_MM) {
          /* one IR still black long after the other cleared: it is on the main line, and the IR that cleared crossed a
           * BRANCH on its side. The branch was where both went black: advance from there and turn toward it */
          a->side = l_clear ? -1 : +1;
          junction_found(a, in, a->cross_mm);
          break;
        }
      }
      if (d - a->cross_mm >= (float)TRL_CROSS_MAX_MM) enter(a, TRL_FOLLOW, in);
      break;
    }

    case TRL_PROBE: {
      hold_heading(a, in, o);
      const bool seen = (a->side < 0) ? in->left : in->right;   /* the IR that saw black */
      const bool other = (a->side < 0) ? in->right : in->left;  /* the IR on the other side */
      /* signed sideways progress since slide_base: + = toward the side that saw black (strafe_mm is + to the right) */
      const float prog = (in->strafe_mm - a->slide_base) * (float)a->side;
      const float sp = slide_speed(a);
      const float half = a->gap_mm * 0.5f - brake_mm(sp); /* stop this early: the TR rolls brake_mm more */
      if (seen && other) {
        /* both IRs on black while probing: a branch is under one of them (sliding along a branch brings the other IR onto
         * the main line; or the other IR has just found a branch on ITS side). A drift cannot do this: the free gap
         * separates the two IRs. Drive on and see which IR clears first: that one is on the branch */
        enter_cross(a, in);
        o->y = a->speed;
        break;
      }
      if (a->phase == 0) {
        if (!seen) { /* the line slid back between the IRs: a drift. Now re-centre it */
          a->phase = 1;
          a->slide_base = in->strafe_mm;
        } else if (since(in->now, a->since) >= TRL_PROBE_MS) { /* still black while sliding along it: a branch */
          junction_found(a, in, in->dist_mm);
          break;
        }
        o->x = (float)a->side * sp; /* slide toward the side that saw black */
        break;
      }
      if (a->phase == 1) { /* keep sliding the same way: half the known gap, or until the other IR touches the line */
        if (seen && prog < (float)TRL_FLICKER_MM) { /* black again right away: it flickered at the tape edge. Keep probing (the
                                                       junction timer keeps running from the start of the probe) */
          a->phase = 0;
          o->x = (float)a->side * sp;
          break;
        }
        if (other || prog >= (float)TRL_GAP_MAX_MM) { /* touched the other side: the gap is this slide; go back half of it */
          if (prog < (float)TRL_GAP_MIN_MM) {
            /* far too small to be the real gap: another line (a branch or crossing) touched the other IR while the TR
             * was still rolling. Keep the gap we had and just end this correction. */
            if (a->drifts < 65535U) a->drifts++;
            enter(a, TRL_FOLLOW, in);
            break;
          }
          a->gap_mm = (a->gap_mm > 0.0f) ? 0.5f * (a->gap_mm + prog) : prog; /* average: one odd reading cannot spoil it */
          a->phase = 2;
          a->slide_base = in->strafe_mm;
          o->x = -(float)a->side * sp;
          break;
        }
        if (a->gap_mm > 0.0f && prog >= half) { /* centred (after braking) */
          drift_fixed(a);
          enter(a, TRL_FOLLOW, in);
          break;
        }
        o->x = (float)a->side * sp;
        break;
      }
      /* phase 2: slide back half the gap (progress counts only movement back, not the momentum still going the other way) */
      if (-prog >= half || seen) {
        drift_fixed(a);
        enter(a, TRL_FOLLOW, in);
        break;
      }
      o->x = -(float)a->side * sp;
      break;
    }

    case TRL_ADV: /* bring the turning centre over the junction (a robot dimension, not a field distance) */
      hold_heading(a, in, o);
      o->y = a->speed;
      if (in->dist_mm - a->base_mm >= (float)TRA_ADVANCE_MM) enter(a, TRL_TURN, in);
      break;

    case TRL_TURN: {
      const int t = do_turn(a, in, o, (float)a->side * 90.0f); /* clockwise positive: right +90, left -90 */
      if (t > 0) enter(a, TRL_FOLLOW, in);
      else if (t < 0) a->st = TRL_FAULT;
      break;
    }

    default:
      break;
  }
  if (!TRLine_IsRunning(a)) { o->x = o->y = o->w = 0.0f; o->active = false; }
}

const char *TRLine_StateText(TRL_State s) {
  switch (s) {
    case TRL_IDLE: return "READY";
    case TRL_COUNTDOWN: return "COUNTDOWN";
    case TRL_LEAVE: return "LEAVE START";
    case TRL_FOLLOW: return "FOLLOW";
    case TRL_CHECK: return "CHECK";
    case TRL_CROSS: return "CROSS";
    case TRL_PROBE: return "PROBE";
    case TRL_ADV: return "ADVANCE";
    case TRL_TURN: return "TURN 90";
    case TRL_END: return "STOP: US";
    case TRL_TIMEUP: return "TIME UP";
    case TRL_FAULT: return "FAULT STOP";
    case TRL_FOUND: return "FOUND";
  }
  return "?";
}
