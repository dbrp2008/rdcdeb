#include "enc_watch.h"

void EncWatch_Reset(EncWatch *w) { *w = (EncWatch){0}; }

static bool stuck(EncWatch *w, unsigned i, uint32_t now, float cmd, uint32_t raw) {
  const float mag = cmd < 0.0f ? -cmd : cmd;
  if (mag < ENC_WATCH_MIN_PCT) { w->armed[i] = false; return false; }
  if (!w->armed[i] || raw != w->last[i]) { w->armed[i] = true; w->last[i] = raw; w->since[i] = now; return false; }
  return (uint32_t)(now - w->since[i]) >= ENC_WATCH_MS;
}

const char *EncWatch_Update(EncWatch *w, uint32_t now, float cmd_r, uint32_t raw_r, float cmd_l, uint32_t raw_l) {
  const bool r = stuck(w, 0u, now, cmd_r, raw_r), l = stuck(w, 1u, now, cmd_l, raw_l);
  return r && l ? "NO PULSES L+R" : r ? "R NO PULSES" : l ? "L NO PULSES" : (const char *)0;
}
