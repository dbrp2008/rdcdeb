#ifndef ELCP_MOTOR_H
#define ELCP_MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "tim.h"

/**
 * @brief FB pulses per output-shaft revolution, used for rpm = pps * 60 / ppr. 
 * @note NOTICE: This number is for power source of 12V
 */
#ifndef ELCP_FB_PULSES_PER_REV
#define ELCP_FB_PULSES_PER_REV 114.0f
#endif

/**
 * @brief Motor connector selector.
 */
typedef enum {
  MOTOR_J1 = 0, /**< PWM PB14 / DIR PC6 / FB PB15 */
  MOTOR_J3,     /**< PWM PC7 / DIR PC9 / FB PC8 */
  ELCP_MOTOR_COUNT
} ELCP_Motor_ID;

/**
 * @brief Read-only view of a motor's runtime state.
 */
typedef struct {
  volatile float current_speed_pct; /**< Commanded speed, -100.0 (full CCW) to +100.0 (full CW) */
  volatile int32_t pulse_count;     /**< Signed FB pulse count */
  volatile float pps;               /**< FB pulses per second (updated once per second) */
  volatile float rpm;               /**< Signed speed in RPM (updated once per second) */
} ELCP_Motor_State_t;

/* ----- Core API ----- */

/**
  * @brief  Initializes a motor: clears its state and starts PWM output.
  * @param  id  Motor to initialize (MOTOR_J1 or MOTOR_J3)
  * @retval None
  */
void ELCP_Motor_Init(ELCP_Motor_ID id);

/* True when HAL_TIM_PWM_Start reported success; not physical verification. */
bool ELCP_Motor_IsInitialized(ELCP_Motor_ID id);

/* Raw rising EXTI events since boot, including when a motor is not initialized.
 * Wraps at 2^32; noise and missed edges can make this inaccurate. */
/* FB edges closer together than this (us) are treated as bounce and ignored; 0 = off. At 150 rpm real edges are ~3.5 ms apart. */
#ifndef ELCP_FB_MIN_DT_US
#define ELCP_FB_MIN_DT_US 250u
#endif
/* Diagnostics: smallest spacing between two FB edges seen (us, 0xFFFFFFFF = none yet) and number of edges dropped as bounce. */
extern volatile uint32_t elcp_j1_fb_min_dt_us, elcp_j3_fb_min_dt_us, elcp_j1_fb_rejected, elcp_j3_fb_rejected;
void ELCP_FB_ResetStats(void);
extern volatile uint32_t elcp_j1_fb_edges;
extern volatile uint32_t elcp_j3_fb_edges;

/**
  * @brief  Sets the motor speed and drives the DIR pin.
  * @param  id             Motor to drive (MOTOR_J1 or MOTOR_J3)
  * @param  speed_percent  -100.0 (full CCW) to +100.0 (full CW)
  * @retval None
  */
void ELCP_Motor_SetSpeed(ELCP_Motor_ID id, float speed_percent);

/**
  * @brief  Requests zero software speed (same as ELCP_Motor_SetSpeed(id, 0)).
  *         Driver response is unverified; this is not a guaranteed physical stop.
  * @param  id  Motor to stop
  * @retval None
  */
void ELCP_Motor_Stop(ELCP_Motor_ID id);

/**
  * @brief  Gets the motor's live state (read-only).
  * @param  id  Motor to query
  * @retval Pointer to the state, or NULL if id is invalid
  */
const ELCP_Motor_State_t *ELCP_Motor_GetState(ELCP_Motor_ID id);

/**
  * @brief  Overrides the FB pulses-per-revolution used for the rpm estimate.
  * @param  id              Motor to configure
  * @param  pulses_per_rev  FB pulses per output-shaft revolution (> 0)
  * @retval None
  */
void ELCP_Motor_SetPulsesPerRev(ELCP_Motor_ID id, float pulses_per_rev);

/* ----- Background / ISR (wired up in stm32f4xx_it.c) ----- */

/**
  * @brief  Handles one FB pulse; updates the motor's pulse count.
  * @param  id        Motor that raised the interrupt
  * @param  GPIO_Pin  Pin that triggered the EXTI (must match the motor's FB pin)
  * @retval None
  */
void ELCP_Motor_FB_IRQHandler(ELCP_Motor_ID id, uint16_t GPIO_Pin);

/**
  * @brief  Publishes pps and rpm into the motor state (refreshes once per second).
  *         Call once per main-loop pass; rpm uses ELCP_FB_PULSES_PER_REV or the
  *         value set by ELCP_Motor_SetPulsesPerRev().
  * @param  id  Motor to update
  * @retval None
  */
void ELCP_Motor_UpdatePPS(ELCP_Motor_ID id);

#endif
