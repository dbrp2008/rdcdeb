#include "retrace.h"
#include <string.h>

static uint32_t absdiff(int32_t a, int32_t b) { const int32_t d = a - b; return (uint32_t)(d < 0 ? -d : d); }
static int16_t x10(float pct) {
  const float v = pct * 10.0f;
  return (int16_t)(v < -1000.0f ? -1000 : (v > 1000.0f ? 1000 : (v < 0.0f ? v - 0.5f : v + 0.5f)));
}

void Retrace_Start(Retrace *r, uint32_t now, int32_t j1, int32_t j3) {
  r->state = RETRACE_RECORDING;
  r->n = 0;
  r->overflow = false;
  r->sum_left = r->sum_right = 0.0f;
  r->acc_ms = r->acc_pulses = 0;
  r->acc_pl = r->acc_pr = 0;
  r->last_ms = now;
  r->last_j1 = j1;
  r->last_j3 = j3;
  r->have_last = true;
  r->cur_left = r->cur_right = 0.0f;
  r->k = -1;
  r->recorded_pulses = 0;
  r->edge_marked = r->edge_in = r->anchored = false;
  r->edge_mark = 0;
  r->anchor_fix = 0;
}

static void close_crumb(Retrace *r) {
  if (r->acc_ms == 0u && r->acc_pulses == 0u) return;
  if (r->n >= RETRACE_MAX_CRUMBS) { r->overflow = true; return; }
  RetraceCrumb *c = &r->crumbs[r->n++];
  const float ms = r->acc_ms ? (float)r->acc_ms : 1.0f;
  c->left_x10 = x10(r->sum_left / ms);
  c->right_x10 = x10(r->sum_right / ms);
  c->pulses = (uint16_t)(r->acc_pulses > 65535u ? 65535u : r->acc_pulses);
  r->recorded_pulses += c->pulses;
  c->ms = (uint16_t)(r->acc_ms > 65535u ? 65535u : r->acc_ms);
  c->pl = (uint16_t)(r->acc_pl > 65535u ? 65535u : r->acc_pl);
  c->pr = (uint16_t)(r->acc_pr > 65535u ? 65535u : r->acc_pr);
  r->sum_left = r->sum_right = 0.0f;
  r->acc_ms = r->acc_pulses = 0;
  r->acc_pl = r->acc_pr = 0;
}

void Retrace_Record(Retrace *r, uint32_t now, LineWheels sent, int32_t j1, int32_t j3) {
  if (r->state != RETRACE_RECORDING) return;
  /* the time and pulses since the previous call belong to the command that was running since then */
  const uint32_t dt = now - r->last_ms;
  const uint32_t dpr = absdiff(j1, r->last_j1), dpl = absdiff(j3, r->last_j3); /* J1 = right wheel, J3 = left */
  const uint32_t dp = dpl + dpr;
  r->last_ms = now;
  r->last_j1 = j1;
  r->last_j3 = j3;
  const bool moving = r->cur_left != 0.0f || r->cur_right != 0.0f;
  if (moving) {
    r->sum_left += r->cur_left * (float)dt;
    r->sum_right += r->cur_right * (float)dt;
    r->acc_ms += dt;
    r->acc_pulses += dp;
    r->acc_pl += dpl;
    r->acc_pr += dpr;
  }
  /* a new kind of motion (a wheel changes direction, e.g. the start of a pivot) always starts a new breadcrumb */
  const bool flip = (sent.left > 0.0f) != (r->cur_left > 0.0f) || (sent.right > 0.0f) != (r->cur_right > 0.0f) ||
                    (sent.left < 0.0f) != (r->cur_left < 0.0f) || (sent.right < 0.0f) != (r->cur_right < 0.0f);
  if (r->acc_pulses >= RETRACE_CRUMB_PULSES || r->acc_ms >= RETRACE_CRUMB_MAX_MS || (flip && r->acc_ms > 0u)) close_crumb(r);
  r->cur_left = sent.left;
  r->cur_right = sent.right;
}

static float reverse_duty(int16_t x) {
  if (x == 0) return 0.0f;
  float v = -(float)x / 10.0f;
  if (v > 0.0f && v < RETRACE_MIN_PERCENT) v = RETRACE_MIN_PERCENT;
  if (v < 0.0f && v > -RETRACE_MIN_PERCENT) v = -RETRACE_MIN_PERCENT;
  return v;
}

static int32_t remaining(const Retrace *r);
/* still allowed to look for the start-zone outline (trail used up, outline not yet crossed back, within the window) */
static bool seeking(const Retrace *r) {
  return r->edge_marked && !r->anchored && r->n > 0u && (int32_t)r->edge_mark - remaining(r) <= (int32_t)RETRACE_EDGE_WINDOW_PULSES;
}

bool Retrace_Step(Retrace *r, uint32_t now, int32_t j1, int32_t j3, LineWheels *cmd) {
  cmd->left = cmd->right = 0.0f;
  if (r->state == RETRACE_RECORDING) { /* first call: close the last breadcrumb and start from the end */
    const LineWheels stop = {0.0f, 0.0f};
    Retrace_Record(r, now, stop, j1, j3);
    close_crumb(r);
    r->state = RETRACE_RETURNING;
    r->k = (int32_t)r->n - 1;
    r->done_pulses = 0;
    r->done_pl = r->done_pr = 0;
    r->edge_in = false;
    r->crumb_start_ms = r->last_pulse_ms = now;
    r->last_j1 = j1;
    r->last_j3 = j3;
  }
  if (r->state != RETRACE_RETURNING) return false;
  const uint32_t dpr = absdiff(j1, r->last_j1), dpl = absdiff(j3, r->last_j3);
  const uint32_t dp = dpl + dpr;
  r->last_j1 = j1;
  r->last_j3 = j3;
  if (dp) r->last_pulse_ms = now;
  r->done_pulses += (int32_t)dp;
  r->done_pl += (int32_t)dpl;
  r->done_pr += (int32_t)dpr;
  for (;;) {
    if (r->k < 0) {
      if (seeking(r)) break; /* the trail is used up but the outline was not crossed back yet: keep reversing a little */
      r->state = r->overflow ? RETRACE_PARTIAL : RETRACE_DONE;
      return false;
    }
    const RetraceCrumb *c = &r->crumbs[r->k];
    const bool finished = c->pulses ? r->done_pulses >= (int32_t)c->pulses : (uint32_t)(now - r->crumb_start_ms) >= c->ms;
    if (!finished) break;
    r->done_pulses = c->pulses ? r->done_pulses - (int32_t)c->pulses : 0; /* carry the overshoot into the next breadcrumb */
    r->done_pl = c->pulses ? r->done_pl - (int32_t)c->pl : 0;               /* ...per wheel too: each wheel's total stays exact */
    r->done_pr = c->pulses ? r->done_pr - (int32_t)c->pr : 0;
    r->k--;
    r->crumb_start_ms = now;
  }
  if ((uint32_t)(now - r->last_pulse_ms) >= RETRACE_STALL_MS) { r->state = RETRACE_STALLED; return false; }
  const RetraceCrumb *c = &r->crumbs[r->k < 0 ? 0 : r->k]; /* seeking: keep the first breadcrumb's (reversed) motion */
  cmd->left = reverse_duty(c->left_x10);
  cmd->right = reverse_duty(c->right_x10);
  /* each wheel to ITS recorded pulses: the wheel that is ahead (as a fraction of its share of this breadcrumb) is slowed */
  if (r->k >= 0 && c->pl && c->pr && cmd->left != 0.0f && cmd->right != 0.0f) {
    const float fl = (float)r->done_pl / (float)c->pl, fr = (float)r->done_pr / (float)c->pr;
    float kl = 1.0f - RETRACE_WHEEL_K * (fl - fr), kr = 1.0f - RETRACE_WHEEL_K * (fr - fl);
    kl = kl < 0.0f ? 0.0f : (kl > 1.0f ? 1.0f : kl);
    kr = kr < 0.0f ? 0.0f : (kr > 1.0f ? 1.0f : kr);
    cmd->left *= kl;   /* may drop below RETRACE_MIN_PERCENT (even to 0) for a moment: the other wheel catches up */
    cmd->right *= kr;
  } else if (r->k >= 0 && c->pulses && cmd->left != 0.0f && c->pl == 0) {
    cmd->left = 0.0f;  /* this wheel did not turn in this breadcrumb */
  } else if (r->k >= 0 && c->pulses && cmd->right != 0.0f && c->pr == 0) {
    cmd->right = 0.0f;
  }
  return true;
}

/* pulses the trail still has to drive (the current breadcrumb's rest and all earlier ones) */
static int32_t remaining(const Retrace *r) {
  int32_t sum = 0;
  for (int32_t i = 0; i <= r->k; i++) sum += (int32_t)r->crumbs[i].pulses;
  return sum - r->done_pulses;
}
/* jump to the point of the trail that is `left` pulses from the start (the rest is then replayed as recorded) */
static void set_remaining(Retrace *r, int32_t left) {
  int32_t sum = 0, i = 0;
  while (i < (int32_t)r->n - 1 && sum + (int32_t)r->crumbs[i].pulses < left) sum += (int32_t)r->crumbs[i++].pulses;
  r->k = i;
  r->done_pulses = sum + (int32_t)r->crumbs[i].pulses - left; /* the part of breadcrumb i beyond `left` counts as done */
  { const RetraceCrumb *c = &r->crumbs[i];
    const float f = c->pulses ? (float)r->done_pulses / (float)c->pulses : 0.0f;
    r->done_pl = (int32_t)(f * (float)c->pl);
    r->done_pr = (int32_t)(f * (float)c->pr); }
}

void Retrace_Line(Retrace *r, unsigned filtered) {
  const bool all_black = (filtered & 7u) == 7u;
  if (r->state == RETRACE_RECORDING) {
    /* the first all-black within the first part of the run: the IRs reached the start zone's outline (inner edge) */
    const uint32_t so_far = r->recorded_pulses + r->acc_pulses;
    if (!r->edge_marked && all_black && so_far <= RETRACE_EDGE_MAX_PULSES) {
      r->edge_marked = true;
      r->edge_mark = so_far;
    }
    return;
  }
  if (r->state != RETRACE_RETURNING || !r->edge_marked || r->anchored) return;
  /* reversing home: the IRs reach the outline from outside; when they LEAVE it they are at its inner edge again, i.e.
   * exactly edge_mark from the start */
  if (all_black) { r->edge_in = true; return; }
  if (!r->edge_in) return;
  r->edge_in = false;
  const int32_t left = remaining(r), want = (int32_t)r->edge_mark;
  const int32_t diff = left - want;
  if (diff > (int32_t)RETRACE_EDGE_WINDOW_PULSES || diff < -(int32_t)RETRACE_EDGE_WINDOW_PULSES) return; /* not the outline */
  set_remaining(r, want); /* skip (or redo) what the encoders got wrong over the trip */
  r->anchor_fix = -diff;
  r->anchored = true;
}
