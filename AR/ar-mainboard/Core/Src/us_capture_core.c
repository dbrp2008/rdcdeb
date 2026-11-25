#include "us_capture_core.h"

typedef enum { ST_IDLE = 0, ST_WAIT_RISE, ST_WAIT_FALL, ST_DONE } State;
typedef struct {
  volatile State state;
  volatile uint32_t rise;
  volatile uint32_t width;
  uint32_t start_ms;
  US2_Reading result;
  uint32_t seq, taken;
} Chan;
static Chan ch[US_COUNT];

void USC_Reset(void) {
  for (int i = 0; i < US_COUNT; i++) ch[i] = (Chan){0};
}
void USC_Arm(US_Id id, uint32_t now_ms) {
  ch[id].start_ms = now_ms;
  ch[id].state = ST_WAIT_RISE;
}
bool USC_Waiting(US_Id id) { return ch[id].state != ST_IDLE; }
void USC_Fail(US_Id id, US2_Status status) {
  ch[id].state = ST_IDLE;
  ch[id].result = (US2_Reading){.status = status};
  ch[id].seq++;
}
bool USC_OnCapture(US_Id id, uint32_t ccr, uint32_t width_mask) {
  Chan *c = &ch[id];
  if (c->state == ST_WAIT_RISE) {
    c->rise = ccr;
    c->state = ST_WAIT_FALL;
    return true;
  }
  if (c->state == ST_WAIT_FALL) {
    c->width = (ccr - c->rise) & width_mask;
    c->state = ST_DONE;
  }
  return false;
}
bool USC_Poll(US_Id id, uint32_t now_ms, uint32_t timeout_ms) {
  Chan *c = &ch[id];
  if (c->state == ST_IDLE) return true;
  if (c->state == ST_DONE) {
    const uint32_t w = c->width;
    if (w < 100u) c->result = (US2_Reading){.status = US2_SHORT_PULSE, .pulse_us = w};
    else c->result = (US2_Reading){.status = US2_OK, .pulse_us = w, .raw_mm = (w * 343u + 1000u) / 2000u};
    c->state = ST_IDLE;
    c->seq++;
    return true;
  }
  if ((uint32_t)(now_ms - c->start_ms) >= timeout_ms) {
    c->result = (US2_Reading){.status = c->state == ST_WAIT_RISE ? US2_NO_RISE : US2_NO_FALL};
    c->state = ST_IDLE; /* a late edge after this is ignored */
    c->seq++;
    return true;
  }
  return false;
}
bool USC_Take(US_Id id, US2_Reading *out) {
  Chan *c = &ch[id];
  if (c->seq == c->taken) return false;
  c->taken = c->seq;
  *out = c->result;
  return true;
}
const char *US2_StatusText(US2_Status status) {
  switch (status) {
    case US2_OK: return "ECHO OK";
    case US2_ECHO_BUSY: return "ECHO HIGH";
    case US2_NO_RISE: return "NO ECHO RISE";
    case US2_NO_FALL: return "NO ECHO FALL";
    case US2_SHORT_PULSE: return "SHORT PULSE";
    default: return "CHECK TIM2";
  }
}
