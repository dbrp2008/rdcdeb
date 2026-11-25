#ifndef ENC_WATCH_H
#define ENC_WATCH_H
#include <stdbool.h>
#include <stdint.h>
/* Dead-encoder watch. While a wheel is commanded at |duty| >= ENC_WATCH_MIN_PCT, its raw FB pulse count
 * (SideIR_EncoderRaw: every pulse, any speed) must change at least once every ENC_WATCH_MS. If it doesn't, the encoder is
 * dead or unplugged - or the wheel is stalled (against a wall, say): either way the encoder distances and angles are
 * wrong, so the run stops with a message instead of driving or spinning on.
 * Below ENC_WATCH_MIN_PCT a wheel is not checked: the inside wheel of a held turn (9.9 %), the F2 pivot's reverse wheel
 * (9.45 %), the 180 pivot (11.25 %) and the launch (13.5 %) may legitimately barely turn. The 180 pivot still has its
 * 10 s limit. ENC_WATCH_MS is longer than F1_STALL_MS (500 ms), so "HOME AT WALL" still ends a run at the zone's wall. */
#ifndef ENC_WATCH_MS
#define ENC_WATCH_MS 1000u
#endif
#ifndef ENC_WATCH_MIN_PCT
#define ENC_WATCH_MIN_PCT 15.0f
#endif
typedef struct {
  bool armed[2];      /* [0] right (J1), [1] left (J3): commanded at/above the minimum since `since` */
  uint32_t last[2];   /* raw count at the last change */
  uint32_t since[2];  /* when it last changed (or the check armed) */
} EncWatch;
void EncWatch_Reset(EncWatch *w);
/* Every tick while running. cmd_* = the commanded duty (sign ignored), raw_* = the raw pulse counts.
 * Returns NULL, or the stop status: "R NO PULSES", "L NO PULSES", "NO PULSES L+R". */
const char *EncWatch_Update(EncWatch *w, uint32_t now, float cmd_r, uint32_t raw_r, float cmd_l, uint32_t raw_l);
#endif
