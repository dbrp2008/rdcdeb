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

/* ---- Pulse widths in microseconds for the commanded 110..180 degree range.
 * Assumes the usual 500..2500 us mapping to 0..180 degrees; calibrate for the actual servo. ---- */
#ifndef TR_ARM_GRIP_US
#define TR_ARM_GRIP_US   1722U           /* approximately 110 degrees */
#endif
#ifndef TR_ARM_OUT_US
#define TR_ARM_OUT_US    2500U           /* 180 degrees */
#endif
#ifndef TR_ARM_START_US
#define TR_ARM_START_US  2500U           /* start at 180 degrees */
#endif
#define TR_ARM_SPEED_US_PER_S  600.0f    /* pulse change per second while a button is held */

/** Set the start pulse and start PWM. Call once after the generated MX_TIM4_Init(). */
void TRArm_Init(void);

/** cmd: +1 toward GRIP, -1 toward OUT, 0 hold. Call every loop pass. */
void TRArm_Update(int8_t cmd);

/** Current commanded pulse width in microseconds. */
uint16_t TRArm_GetPulseUs(void);

#endif
