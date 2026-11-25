#ifndef TR_RECORD_H
#define TR_RECORD_H

/**
 * Teach and replay for the TR (pure logic, no HAL; the glue is tr_auto_app.c, the flash storage is tr_store.c).
 *
 * RECORD: while a driver drives the TR with the joystick, the TR logs where it is (x, y from the wheel encoders, turned
 *         into field coordinates with the heading), its heading and the arm servo pulse, every TRR_PERIOD_MS.
 * REPLAY: the TR follows the recorded PATH (not the joystick inputs): every step it compares where it is with where the
 *         recording says it should be at this moment, and drives toward it (feed-forward speed from the recording plus a
 *         correction). Because the recording was measured with the same encoders, a wheel-size / scale error cancels out.
 *         If it falls behind (e.g. the recording was faster than the auto speed limit) the replay clock slows down, so it
 *         follows the path instead of cutting corners. MIRROR replays it reflected left <-> right (the other field side).
 *
 * Field frame: X = the TR's forward direction when the recording / replay starts, Y = its right; heading clockwise
 * positive, 0 at the start. The TR must start each replay in the same place and direction as the recording.
 */

#include <stdbool.h>
#include <stdint.h>
#include "tr_auto.h"

#ifndef TRR_PERIOD_MS
#define TRR_PERIOD_MS 20U /* 50 samples per second */
#endif
#ifndef TRR_MAX_SAMPLES
#define TRR_MAX_SAMPLES 1500U /* 30 s */
#endif
#ifndef TRR_FULL_SCALE_MM_S
#define TRR_FULL_SCALE_MM_S 1037.0f /* auto command 1.0 = TRA_TRANSLATE_RPM (130) on 152.4 mm wheels */
#endif
#ifndef TRR_DEG_S_PER_W
#define TRR_DEG_S_PER_W 100.5f /* auto rotation command 1.0 = TR_ROTATE_RPM (60) rim speed / 0.273 m radius */
#endif
#ifndef TRR_VMAX_MM_S
#define TRR_VMAX_MM_S 1000.0f /* replay speed limit */
#endif
#ifndef TRR_KP_POS
#define TRR_KP_POS 2.5f /* mm/s of correction per mm of position error */
#endif
#ifndef TRR_KP_HDG
#define TRR_KP_HDG 4.0f /* deg/s of correction per degree of heading error */
#endif
#ifndef TRR_W_MAX
#define TRR_W_MAX 0.8f
#endif
#ifndef TRR_SLOW_FROM_MM
#define TRR_SLOW_FROM_MM 40.0f /* behind by more than this: the replay clock starts to slow down ... */
#endif
#ifndef TRR_SLOW_SPAN_MM
#define TRR_SLOW_SPAN_MM 160.0f /* ... down to TRR_SLOW_MIN at TRR_SLOW_FROM_MM + this */
#endif
#ifndef TRR_SLOW_MIN
#define TRR_SLOW_MIN 0.25f
#endif
#ifndef TRR_END_POS_MM
#define TRR_END_POS_MM 15.0f /* after the last sample: done once this close ... */
#endif
#ifndef TRR_END_HDG_DEG
#define TRR_END_HDG_DEG 2.0f
#endif
#ifndef TRR_END_MAX_MS
#define TRR_END_MAX_MS 1500U /* ... or after this long */
#endif
#ifndef TRR_OFF_PATH_MM
#define TRR_OFF_PATH_MM 300.0f /* further than this from the path ... */
#endif
#ifndef TRR_OFF_PATH_MS
#define TRR_OFF_PATH_MS 2000U /* ... for this long (blocked, pushed away): stop with a fault */
#endif
#ifndef TRR_STILL_MM
#define TRR_STILL_MM 3 /* recording: moved less than this (and the heading / arm did not change) = still waiting: trimmed */
#endif
#ifndef TRR_STILL_DDEG
#define TRR_STILL_DDEG 5 /* 0.5 degree */
#endif
#ifndef TRR_ARM_DEADBAND_US
#define TRR_ARM_DEADBAND_US 20
#endif

typedef struct {
  int16_t x_mm, y_mm;  /* field position of the turning centre */
  int16_t hdg_ddeg;    /* heading, 0.1 degree, clockwise positive, continuous (not wrapped) */
  uint16_t arm_us;     /* arm servo pulse */
} TRR_Sample;          /* 8 bytes */

/* Field pose from the encoder distances (forward / right, mm, with the rotation already removed by the drive) and the
 * heading. Feed it every loop. */
typedef struct {
  float x, y;          /* mm, field frame */
  float f_prev, s_prev, h_prev;
  float h0;            /* heading at the start */
  bool started;
} TRR_Pose;
void TRR_PoseStart(TRR_Pose *p, float fwd_mm, float strafe_mm, float heading_deg);
void TRR_PoseUpdate(TRR_Pose *p, float fwd_mm, float strafe_mm, float heading_deg);
float TRR_PoseHeading(const TRR_Pose *p, float heading_deg); /* heading relative to the start */

typedef struct {
  TRR_Sample buf[TRR_MAX_SAMPLES];
  uint16_t n;
  bool on;
  uint32_t t0, next;
  float vmax_mm_s;     /* fastest recorded speed (the replay is limited to TRR_VMAX_MM_S) */
  TRR_Pose pose;
} TRR_Recorder;
void TRR_RecStart(TRR_Recorder *r, uint32_t now, float fwd_mm, float strafe_mm, float heading_deg);
/* Call every loop while recording; returns false once full (it stops by itself). */
bool TRR_RecStep(TRR_Recorder *r, uint32_t now, float fwd_mm, float strafe_mm, float heading_deg, uint16_t arm_us);
void TRR_RecStop(TRR_Recorder *r); /* also trims the standing still at the start and the end (n = 0 if it never moved) */

typedef enum { TRR_IDLE = 0, TRR_RUN, TRR_END, TRR_DONE, TRR_FAULT } TRR_State;
typedef struct {
  TRR_State st;
  const TRR_Sample *s;
  uint16_t n;
  bool mirror;
  float t_ms;          /* replay clock (slows down when behind) */
  uint32_t last_ms, end_since, off_since;
  TRR_Pose pose;
  float err_mm, max_err_mm; /* distance from the path now / worst during the replay */
} TRR_Player;
/* s / n: the recording (e.g. straight from flash). Returns false if there is nothing to replay. */
bool TRR_PlayStart(TRR_Player *p, const TRR_Sample *s, uint16_t n, bool mirror, uint32_t now, float fwd_mm, float strafe_mm,
                   float heading_deg);
/* One step: fills out (x / y / w command ratios, arm) like the routines. in: dist_mm, strafe_mm, heading_deg, heading_sign,
 * arm_us, now. */
void TRR_PlayStep(TRR_Player *p, const TRAuto_In *in, TRAuto_Out *out);
bool TRR_PlayRunning(const TRR_Player *p);
const char *TRR_PlayStateText(TRR_State s);

/* CRC-32 (IEEE) for the stored recording */
uint32_t TRR_Crc32(const void *data, uint32_t len, uint32_t crc);

#endif
