#ifndef SIDE_IR_H
#define SIDE_IR_H
#include "branch_latch.h"
#include <stdbool.h>
#include <stdint.h>

/* Raw levels (both high on black). Counts are serviced rising-edge IRQs since
 * reset (uint32_t wrap). Each field is independent, not a simultaneous frame. */
typedef struct {
  bool left_high, right_high;
  uint32_t left_rises, right_rises;
} SideIR_Reading;
/* Foreground, single consumer. Drain at accepted start (use only .high), then
 * once per run scan. Never call this from the TFT: SideIR_Read is non-consuming. */
BranchSample SideIR_TakeBranchSample(void);
SideIR_Reading SideIR_Read(void);          /* foreground only */
/* Every encoder FB pulse since reset (after the bounce filter), counted at any speed - unlike the driver's pulse_count,
 * which ignores pulses while the wheel is commanded to 0 %. right = J1, else J3. Goes UP whichever way the wheel turns. */
uint32_t SideIR_EncoderRaw(bool right);
/* Odometry count: every filtered pulse, + while the wheel is commanded forward (> 0 %), - in reverse, ignored at 0 %. */
int32_t SideIR_EncoderSigned(bool right);
/* F4: also count pulses at 0 % in the last commanded direction (coast). Off unless F4 is running. */
void SideIR_CountCoast(bool on);
/* The wheel's commanded speed when a pulse arrives (J1 right / J3 left). Weak default 0; the drive app overrides it. */
float SideIR_WheelCommandPct(bool right);
void SideIR_DrawRow(uint8_t row);          /* 16 columns: level / rises modulo 100 */
#endif
