#ifndef ELCP_ZERO_PWM_H
#define ELCP_ZERO_PWM_H
#include <stdbool.h>

/* True means initialization was requested for a valid timer handle only.
 * Reference init discards the HAL status; physical behavior is not verified. */
typedef struct { bool j1_ok, j3_ok; } ELCP_ZeroPWM_Result;
/* Startup-only zero-PWM request. Requires MX_TIM12_Init and MX_TIM8_Init.
 * Motor power must be disconnected: zero PWM is not a power cutoff. */
ELCP_ZeroPWM_Result ELCP_ZeroPWM_Init(void);
/* Reassert zero compare on each valid timer and both DIR pins low. Does not
 * (re)start PWM, reset encoder counts or issue any motion command. */
void ELCP_ZeroPWM_Hold(void);
#endif
