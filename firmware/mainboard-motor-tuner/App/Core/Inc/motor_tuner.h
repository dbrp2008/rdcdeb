#ifndef MOTOR_TUNER_H
#define MOTOR_TUNER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  TUNER_PROFILE_TR = 0,
  TUNER_PROFILE_AR = 1
} TunerProfile;

typedef struct {
  TunerProfile profile;
  uint8_t motor_index;
  uint8_t running;
  uint8_t feedback_fresh;
  uint8_t settling;
  uint8_t safety_fault; /* Nonzero locks out new runs until reset. */
  uint8_t can_tx_fault;
  uint8_t results_ready;
  uint8_t motor_temp_c;
  uint8_t experiment_index;
  uint8_t pass_streak;
  uint32_t trial_number;
  uint32_t elapsed_ms;
  float target_rpm;
  float measured_rpm;
  float current_a; /* commanded current */
  float measured_current_a; /* ESC-reported motor current */
  float kp;
  float ki;
  float kd;
  float iae_rpm_seconds;
  float peak_rpm;
} TunerStatus;

void MotorTuner_Init(void);
void MotorTuner_SelectProfile(TunerProfile profile);
void MotorTuner_SelectNextMotor(void);
TunerProfile MotorTuner_GetProfile(void);
uint8_t MotorTuner_GetMotorCount(void);
void MotorTuner_Start(void);
void MotorTuner_Abort(void);
void MotorTuner_Tick(uint32_t now_ms);
bool MotorTuner_GetStatus(TunerStatus *status);

#endif
