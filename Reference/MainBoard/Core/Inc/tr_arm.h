#ifndef TR_ARM_H
#define TR_ARM_H

/**
 * Titan Robot arm/gripper: one hobby servo (50 Hz PWM, pulse width in us).
 *
 *   command +1 (Controller button 5) : move toward GRIP while held
 *   command -1 (Controller button 8) : move toward OUT while held
 *   command  0                       : hold where it is
 *
 * It moves smoothly at TR_ARM_SPEED_US_PER_S and never goes past the GRIP/OUT limits.
 */

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ---- Master switch. 0 = arm code compiled out (no servo timer needed in CubeMX).
 * Set to 1 ONLY after you have enabled the servo's PWM timer in CubeMX and know the pin. ---- */
#ifndef TR_ARM_ENABLED
#define TR_ARM_ENABLED 1
#endif

/* ---- Which timer/channel the servo is on. The servo signal wire is in the PB7 slot of header J8
 * ("PB7 PB6 PB5 PB4"), and PB7 is TIM4 channel 2. Enable TIM4 CH2 as PWM in CubeMX.
 * (Other pins on that header: PB6 = TIM4 CH1; on header J5: PB8 = TIM4 CH3, PB9 = TIM4 CH4.) ---- */
#define TR_ARM_TIM      htim4
#ifndef TR_ARM_CHANNEL
#define TR_ARM_CHANNEL  TIM_CHANNEL_2
#endif

/* ---- Pulse widths in microseconds. CALIBRATE: find the real grip/out positions on your arm.
 * Servo safe range is 500..2500 us. Keep these inside what the mechanism can physically reach,
 * otherwise the servo stalls against the end stop (hot, high current). ---- */
/* Arm travel: GRIP (closed, smallest angle) to OUT (open). The grip stops a little past where the jaw meets the block, so the
 * servo keeps pressing on it; a smaller GRIP angle presses harder (and makes the servo work harder while holding). */
#ifndef TR_ARM_GRIP_US
#define TR_ARM_GRIP_US   1611U   /* 100 deg (smallest angle) on a 500..2500 us = 0..180 deg servo (was 110 = 1722 us, then 105 = 1667 us; firmer grip) */
#endif
#ifndef TR_ARM_OUT_US
#define TR_ARM_OUT_US    2500U   /* 180 deg (largest angle) */
#endif
#ifndef TR_ARM_START_US
#define TR_ARM_START_US  2500U           /* position it jumps to at power-up: 2500 us = 180 deg on a 500..2500 us servo (= OUT) */
#endif
#define TR_ARM_SPEED_US_PER_S  600.0f    /* pulse change per second while a button is held */
#define TR_ARM_AUTO_US_PER_S   1500.0f   /* ...and while the autonomous routine moves it (full open <-> grip ~0.6 s: a hobby servo
                                          * does that sweep in ~0.5 s; the routine also waits TRA_ARM_SETTLE_MS after) */

/** Set the start pulse and start PWM. Call once after the generated MX_TIM4_Init(). */
void TRArm_Init(void);

/** cmd: +1 toward GRIP, -1 toward OUT, 0 hold. Call every loop pass. */
void TRArm_Update(int8_t cmd);
/** true: the autonomous routine drives the arm (TR_ARM_AUTO_US_PER_S), false: the buttons (TR_ARM_SPEED_US_PER_S). */
void TRArm_SetAuto(bool on);

/** Current commanded pulse width in microseconds. */
uint16_t TRArm_GetPulseUs(void);

#endif
