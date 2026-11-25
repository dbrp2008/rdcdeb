#include "ir_calibration.h"
#include <stddef.h>
#include <string.h>

static IR_Threshold make_threshold(IR_Range white, IR_Range black)
{
  bool black_high = white.max < black.min;
  uint16_t lower_max = black_high ? white.max : black.max;
  uint16_t upper_min = black_high ? black.min : white.min;
  if ((uint32_t)upper_min < (uint32_t)lower_max + IR_CAL_MIN_GAP)
    return (IR_Threshold){0};
  unsigned margin = ((unsigned)upper_min - lower_max) / 4U;
  return (IR_Threshold){(uint16_t)(lower_max + margin),
                        (uint16_t)(upper_min - margin), black_high, true};
}

static IR_Surface classify(uint16_t raw, const IR_Threshold *threshold, IR_Surface previous)
{
  if (!threshold->valid) return IR_SURFACE_UNKNOWN;
  if (raw >= threshold->high) return threshold->black_high ? IR_SURFACE_BLACK : IR_SURFACE_WHITE;
  if (raw <= threshold->low) return threshold->black_high ? IR_SURFACE_WHITE : IR_SURFACE_BLACK;
  return previous; /* Preserve prior classification inside the hysteresis band. */
}

void IR_CalibrationInit(IR_Calibration *cal)
{
  if (cal != NULL) memset(cal, 0, sizeof(*cal));
}

bool IR_CalibrationBegin(IR_Calibration *cal, IR_Surface surface, uint32_t now_ms)
{
  if (cal == NULL || cal->snapshot.capturing != IR_SURFACE_UNKNOWN ||
      (surface != IR_SURFACE_WHITE && surface != IR_SURFACE_BLACK)) return false;
  /* A failed or unfinished recapture must not leave old thresholds usable. */
  if (surface == IR_SURFACE_WHITE) cal->snapshot.have_white = false;
  else cal->snapshot.have_black = false;
  for (unsigned i = 0U; i < IR_CAL_SENSOR_COUNT; ++i) {
    cal->snapshot.threshold[i].valid = false;
    cal->snapshot.surface[i] = IR_SURFACE_UNKNOWN;
  }
  cal->snapshot.capturing = surface;
  cal->snapshot.capture_count = 0U;
  cal->snapshot.capture_failed = false;
  cal->capture_started_ms = now_ms;
  return true;
}

void IR_CalibrationFeed(IR_Calibration *cal, uint32_t now_ms,
                        const uint16_t raw[IR_CAL_SENSOR_COUNT], bool frame_ok)
{
  if (cal == NULL) return;
  IR_CalibrationSnapshot *s = &cal->snapshot;
  if (!frame_ok || raw == NULL) {
    s->frame_valid = false;
    for (unsigned i = 0U; i < IR_CAL_SENSOR_COUNT; ++i) s->surface[i] = IR_SURFACE_UNKNOWN;
    if (s->capturing != IR_SURFACE_UNKNOWN) {
      s->capturing = IR_SURFACE_UNKNOWN;
      s->capture_failed = true;
    }
    return;
  }
  s->frame_valid = true;
  s->frame_tick_ms = now_ms;
  for (unsigned i = 0U; i < IR_CAL_SENSOR_COUNT; ++i) s->raw[i] = raw[i];

  if (s->capturing != IR_SURFACE_UNKNOWN &&
      (uint32_t)(now_ms - cal->capture_started_ms) >= IR_CAL_SETTLE_MS) {
    for (unsigned i = 0U; i < IR_CAL_SENSOR_COUNT; ++i) {
      if (s->capture_count == 0U || raw[i] < cal->capture[i].min) cal->capture[i].min = raw[i];
      if (s->capture_count == 0U || raw[i] > cal->capture[i].max) cal->capture[i].max = raw[i];
    }
    if (++s->capture_count == IR_CAL_CAPTURE_SAMPLES) {
      bool is_white = s->capturing == IR_SURFACE_WHITE;
      for (unsigned i = 0U; i < IR_CAL_SENSOR_COUNT; ++i) {
        if (is_white) s->white[i] = cal->capture[i];
        else s->black[i] = cal->capture[i];
      }
      if (is_white) s->have_white = true;
      else s->have_black = true;
      s->capturing = IR_SURFACE_UNKNOWN;
      if (s->have_white && s->have_black) {
        for (unsigned i = 0U; i < IR_CAL_SENSOR_COUNT; ++i) {
          s->threshold[i] = make_threshold(s->white[i], s->black[i]);
          /* Do not carry a classification across changed thresholds. */
          s->surface[i] = IR_SURFACE_UNKNOWN;
        }
      }
    }
  }
  for (unsigned i = 0U; i < IR_CAL_SENSOR_COUNT; ++i)
    s->surface[i] = classify(raw[i], &s->threshold[i], s->surface[i]);
}
