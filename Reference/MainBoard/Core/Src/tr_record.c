#include "tr_record.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DEG2RAD 0.017453293f

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ---------------- pose ---------------- */
void TRR_PoseStart(TRR_Pose *p, float fwd_mm, float strafe_mm, float heading_deg) {
  p->x = p->y = 0.0f;
  p->f_prev = fwd_mm;
  p->s_prev = strafe_mm;
  p->h_prev = heading_deg;
  p->h0 = heading_deg;
  p->started = true;
}

/* The drive's forward / right distances are measured along the robot's own axes with the rotation removed. Each small step is
 * turned into the field frame with the heading at the middle of the step: forward = (cos h, sin h), right = (-sin h, cos h)
 * in (X forward-at-start, Y right-at-start), h clockwise from the start heading. */
void TRR_PoseUpdate(TRR_Pose *p, float fwd_mm, float strafe_mm, float heading_deg) {
  if (!p->started) {
    TRR_PoseStart(p, fwd_mm, strafe_mm, heading_deg);
    return;
  }
  const float df = fwd_mm - p->f_prev, ds = strafe_mm - p->s_prev;
  const float h = (0.5f * (heading_deg + p->h_prev) - p->h0) * DEG2RAD;
  const float c = cosf(h), s = sinf(h);
  p->x += df * c - ds * s;
  p->y += df * s + ds * c;
  p->f_prev = fwd_mm;
  p->s_prev = strafe_mm;
  p->h_prev = heading_deg;
}

float TRR_PoseHeading(const TRR_Pose *p, float heading_deg) { return heading_deg - p->h0; }

static int16_t to_i16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32767.0f) return -32767;
  return (int16_t)(v < 0.0f ? v - 0.5f : v + 0.5f);
}

/* ---------------- recorder ---------------- */
void TRR_RecStart(TRR_Recorder *r, uint32_t now, float fwd_mm, float strafe_mm, float heading_deg) {
  r->n = 0;
  r->on = true;
  r->t0 = now;
  r->next = now;
  r->vmax_mm_s = 0.0f;
  TRR_PoseStart(&r->pose, fwd_mm, strafe_mm, heading_deg);
}

bool TRR_RecStep(TRR_Recorder *r, uint32_t now, float fwd_mm, float strafe_mm, float heading_deg, uint16_t arm_us) {
  if (!r->on) return false;
  TRR_PoseUpdate(&r->pose, fwd_mm, strafe_mm, heading_deg);
  /* one sample per TRR_PERIOD_MS slot; if the loop was late, every missed slot gets the current pose (the replay assumes
   * evenly spaced samples) */
  while ((int32_t)(now - r->next) >= 0) {
    if (r->n >= TRR_MAX_SAMPLES) {
      r->on = false;
      return false;
    }
    TRR_Sample *s = &r->buf[r->n];
    s->x_mm = to_i16(r->pose.x);
    s->y_mm = to_i16(r->pose.y);
    s->hdg_ddeg = to_i16(TRR_PoseHeading(&r->pose, heading_deg) * 10.0f);
    s->arm_us = arm_us;
    if (r->n > 0) {
      const TRR_Sample *q = &r->buf[r->n - 1];
      const float dx = (float)(s->x_mm - q->x_mm), dy = (float)(s->y_mm - q->y_mm);
      const float v = sqrtf(dx * dx + dy * dy) * 1000.0f / (float)TRR_PERIOD_MS;
      if (v > r->vmax_mm_s) r->vmax_mm_s = v;
    }
    r->n++;
    r->next += TRR_PERIOD_MS;
  }
  return true;
}

static bool same_place(const TRR_Sample *a, const TRR_Sample *b) {
  return abs(a->x_mm - b->x_mm) <= TRR_STILL_MM && abs(a->y_mm - b->y_mm) <= TRR_STILL_MM &&
         abs(a->hdg_ddeg - b->hdg_ddeg) <= TRR_STILL_DDEG && abs((int)a->arm_us - (int)b->arm_us) <= TRR_ARM_DEADBAND_US;
}

/* Stop, and cut the waiting at both ends (walking to the controller after pressing B2, walking back to stop): the replay
 * then starts moving straight after its countdown and does not stand still at the end. One sample is kept before the first
 * movement and after the last. */
void TRR_RecStop(TRR_Recorder *r) {
  r->on = false;
  if (r->n < 2U) return;
  uint16_t first = 0;
  while (first + 1U < r->n && same_place(&r->buf[first + 1U], &r->buf[0])) first++;
  uint16_t last = (uint16_t)(r->n - 1U);
  while (last > first && same_place(&r->buf[last - 1U], &r->buf[r->n - 1U])) last--;
  const uint16_t keep = (uint16_t)(last - first + 1U);
  if (keep < 2U) { /* it never moved */
    r->n = 0;
    return;
  }
  if (first > 0U) memmove(&r->buf[0], &r->buf[first], (size_t)keep * sizeof(TRR_Sample));
  r->n = keep;
}

/* ---------------- player ---------------- */
bool TRR_PlayStart(TRR_Player *p, const TRR_Sample *s, uint16_t n, bool mirror, uint32_t now, float fwd_mm, float strafe_mm,
                   float heading_deg) {
  memset(p, 0, sizeof *p);
  if (s == 0 || n < 2U) {
    p->st = TRR_IDLE;
    return false;
  }
  p->s = s;
  p->n = n;
  p->mirror = mirror;
  p->t_ms = 0.0f;
  p->last_ms = now;
  p->st = TRR_RUN;
  TRR_PoseStart(&p->pose, fwd_mm, strafe_mm, heading_deg);
  return true;
}

bool TRR_PlayRunning(const TRR_Player *p) { return p->st == TRR_RUN || p->st == TRR_END; }

/* sample k, mirrored if needed (Y and heading change sign) */
static void sample_at(const TRR_Player *p, uint16_t k, float *x, float *y, float *h, float *arm) {
  const TRR_Sample *s = &p->s[k];
  *x = (float)s->x_mm;
  *y = p->mirror ? -(float)s->y_mm : (float)s->y_mm;
  *h = (p->mirror ? -(float)s->hdg_ddeg : (float)s->hdg_ddeg) * 0.1f;
  *arm = (float)s->arm_us;
}

void TRR_PlayStep(TRR_Player *p, const TRAuto_In *in, TRAuto_Out *o) {
  o->active = false;
  o->x = o->y = o->w = 0.0f;
  o->arm = 0;
  o->turn_req = false;
  o->turn_deg = 0.0f;
  o->zero_heading = false;
  if (!TRR_PlayRunning(p)) return;

  TRR_PoseUpdate(&p->pose, in->dist_mm, in->strafe_mm, in->heading_deg);
  uint32_t dt_ms = in->now - p->last_ms;
  if (dt_ms > 100U) dt_ms = 100U; /* after a stall, do not jump far ahead */
  p->last_ms = in->now;

  /* where the recording says the TR should be now (interpolated), and how fast it was going there */
  const float t_end = (float)(p->n - 1U) * (float)TRR_PERIOD_MS;
  float t = p->t_ms < t_end ? p->t_ms : t_end;
  uint16_t k = (uint16_t)(t / (float)TRR_PERIOD_MS);
  if (k >= p->n - 1U) k = (uint16_t)(p->n - 2U);
  const float frac = clampf((t - (float)k * (float)TRR_PERIOD_MS) / (float)TRR_PERIOD_MS, 0.0f, 1.0f);
  float x0, y0, h0, a0, x1, y1, h1, a1;
  sample_at(p, k, &x0, &y0, &h0, &a0);
  sample_at(p, (uint16_t)(k + 1U), &x1, &y1, &h1, &a1);
  const float tx = x0 + (x1 - x0) * frac, ty = y0 + (y1 - y0) * frac, th = h0 + (h1 - h0) * frac, ta = a0 + (a1 - a0) * frac;
  const bool ended = p->t_ms >= t_end;
  const float ffx = ended ? 0.0f : (x1 - x0) * 1000.0f / (float)TRR_PERIOD_MS; /* mm/s */
  const float ffy = ended ? 0.0f : (y1 - y0) * 1000.0f / (float)TRR_PERIOD_MS;
  const float ffh = ended ? 0.0f : (h1 - h0) * 1000.0f / (float)TRR_PERIOD_MS; /* deg/s */

  const float ex = tx - p->pose.x, ey = ty - p->pose.y;
  const float h_now = TRR_PoseHeading(&p->pose, in->heading_deg);
  const float eh = th - h_now;
  p->err_mm = sqrtf(ex * ex + ey * ey);
  if (p->err_mm > p->max_err_mm) p->max_err_mm = p->err_mm;

  /* the replay clock: real time, but slower while the TR is behind the path (so it does not cut corners) */
  const float slow = clampf(1.0f - (p->err_mm - TRR_SLOW_FROM_MM) / TRR_SLOW_SPAN_MM, TRR_SLOW_MIN, 1.0f);
  if (!ended) p->t_ms += (float)dt_ms * slow;

  /* blocked / pushed far off the path for a while: stop */
  if (p->err_mm > TRR_OFF_PATH_MM) {
    if (p->off_since == 0U) p->off_since = in->now ? in->now : 1U;
    if ((uint32_t)(in->now - p->off_since) >= TRR_OFF_PATH_MS) {
      p->st = TRR_FAULT;
      return;
    }
  } else {
    p->off_since = 0U;
  }

  if (ended) { /* hold the last pose until close enough, then done */
    if (p->st != TRR_END) {
      p->st = TRR_END;
      p->end_since = in->now;
    }
    if ((p->err_mm <= TRR_END_POS_MM && fabsf(eh) <= TRR_END_HDG_DEG) || (uint32_t)(in->now - p->end_since) >= TRR_END_MAX_MS) {
      p->st = TRR_DONE;
      return;
    }
  }

  /* field-frame velocity: feed-forward (slowed with the clock) + correction, limited */
  float vx = ffx * slow + TRR_KP_POS * ex, vy = ffy * slow + TRR_KP_POS * ey;
  const float v = sqrtf(vx * vx + vy * vy);
  if (v > TRR_VMAX_MM_S) {
    vx *= TRR_VMAX_MM_S / v;
    vy *= TRR_VMAX_MM_S / v;
  }
  /* into the robot's own axes: forward = (cos h, sin h), right = (-sin h, cos h) */
  const float hr = h_now * DEG2RAD, c = cosf(hr), s = sinf(hr);
  o->y = (vx * c + vy * s) / TRR_FULL_SCALE_MM_S;
  o->x = (-vx * s + vy * c) / TRR_FULL_SCALE_MM_S;
  /* rotation: + clockwise rate -> command with the drive's sign convention (as the routines' heading hold) */
  const float rate = ffh * slow + TRR_KP_HDG * eh; /* deg/s clockwise */
  o->w = in->heading_sign * clampf(rate / TRR_DEG_S_PER_W, -TRR_W_MAX, TRR_W_MAX);
  /* arm: move toward the recorded pulse (smaller pulse = closing, as the routines' +1) */
  const float da = ta - (float)in->arm_us;
  o->arm = da < -(float)TRR_ARM_DEADBAND_US ? 1 : (da > (float)TRR_ARM_DEADBAND_US ? -1 : 0);
  o->active = true;
}

const char *TRR_PlayStateText(TRR_State s) {
  switch (s) {
    case TRR_IDLE: return "READY";
    case TRR_RUN: return "REPLAYING";
    case TRR_END: return "FINISHING";
    case TRR_DONE: return "DONE";
    case TRR_FAULT: return "OFF PATH STOP";
  }
  return "?";
}

/* ---------------- CRC-32 ---------------- */
uint32_t TRR_Crc32(const void *data, uint32_t len, uint32_t crc) {
  const uint8_t *b = (const uint8_t *)data;
  crc = ~crc;
  while (len--) {
    crc ^= *b++;
    for (int i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
