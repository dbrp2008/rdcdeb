#ifndef IR_CALIBRATION_H
#define IR_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#define IR_CAL_SENSOR_COUNT 3U
#define IR_CAL_CAPTURE_SAMPLES 100U
#define IR_CAL_SETTLE_MS 250U
#define IR_CAL_MIN_GAP 32U

typedef enum { IR_SURFACE_UNKNOWN = 0, IR_SURFACE_WHITE, IR_SURFACE_BLACK } IR_Surface;
typedef struct { uint16_t min, max; } IR_Range;
typedef struct {
  uint16_t low, high;
  bool black_high;
  bool valid;
} IR_Threshold;

/* All values are RAM-only and ordered left, centre, right. A valid threshold does not
 * prove that a sensor or its wiring is healthy. */
typedef struct {
  uint16_t raw[IR_CAL_SENSOR_COUNT];
  IR_Range white[IR_CAL_SENSOR_COUNT], black[IR_CAL_SENSOR_COUNT];
  IR_Threshold threshold[IR_CAL_SENSOR_COUNT];
  IR_Surface surface[IR_CAL_SENSOR_COUNT];
  IR_Surface capturing;
  uint32_t frame_tick_ms;
  unsigned capture_count;
  bool have_white, have_black, frame_valid, capture_failed;
} IR_CalibrationSnapshot;

typedef struct {
  IR_CalibrationSnapshot snapshot;
  IR_Range capture[IR_CAL_SENSOR_COUNT];
  uint32_t capture_started_ms;
} IR_Calibration;

void IR_CalibrationInit(IR_Calibration *cal);
bool IR_CalibrationBegin(IR_Calibration *cal, IR_Surface surface, uint32_t now_ms);
/* Feed a complete DMA scan; raw may be NULL only when frame_ok is false. */
void IR_CalibrationFeed(IR_Calibration *cal, uint32_t now_ms,
                        const uint16_t raw[IR_CAL_SENSOR_COUNT], bool frame_ok);

#endif
