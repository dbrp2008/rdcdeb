#ifndef MOTOR_SEQUENCE_H
#define MOTOR_SEQUENCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  MOTOR_SEQ_ZERO,
  MOTOR_SEQ_J1,
  MOTOR_SEQ_J3,
  MOTOR_SEQ_BOTH_25,
  MOTOR_SEQ_BOTH_75,
  MOTOR_SEQ_WAIT,
  MOTOR_SEQ_ERROR
} MotorSequencePhase;

/* Both ELCP timers/motors must be initialized first. Zero PWM is not a
   verified stop; never use this sequence as a safety or power interlock. */
void MotorSequence_Init(uint32_t now_ms, bool btn1_pressed);
void MotorSequence_Update(uint32_t now_ms, bool btn1_pressed);
MotorSequencePhase MotorSequence_GetPhase(void);
uint32_t MotorSequence_RemainingMs(uint32_t now_ms);

#endif
