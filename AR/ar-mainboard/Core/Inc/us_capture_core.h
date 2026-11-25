#ifndef US_CAPTURE_CORE_H
#define US_CAPTURE_CORE_H
/* Pure logic of the interrupt-driven ultrasonic readers (no HAL; host-testable). The timer capture interrupt calls
 * USC_OnCapture() with the captured counter value on each echo edge; the main loop calls USC_Poll() / USC_Take(). */
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  US2_OK, US2_ECHO_BUSY, US2_NO_RISE, US2_NO_FALL, US2_SHORT_PULSE, US2_TIMER_ERROR
} US2_Status;
typedef struct {
  US2_Status status;
  uint32_t pulse_us;
  uint32_t raw_mm;
} US2_Reading;
/* Front-centre (PA1 trig / PA0 echo, TIM2 CH1), front-right (UART1 header: PA9 trig / PA10 echo, TIM1 CH3), back-right (PB9 / PB8, TIM4 CH3). */
typedef enum { US_FC = 0, US_FR = 1, US_BR = 2, US_COUNT = 3 } US_Id;

void USC_Reset(void);
void USC_Arm(US_Id id, uint32_t now_ms);                    /* trigger about to be sent: wait for the rising edge */
bool USC_Waiting(US_Id id);                                 /* armed or just captured, result not yet published */
void USC_Fail(US_Id id, US2_Status status);                 /* publish a failure now (e.g. echo already high) */
/* Interrupt context. Returns true when the NEXT capture edge must be FALLING, false when it must be RISING.
 * width_mask is 0xFFFFFFFF for a 32-bit timer, 0xFFFF for a 16-bit one. Edges while not armed are ignored. */
bool USC_OnCapture(US_Id id, uint32_t ccr, uint32_t width_mask);
/* Main loop. True when nothing is in flight any more (finished, timed out, or idle). */
bool USC_Poll(US_Id id, uint32_t now_ms, uint32_t timeout_ms);
bool USC_Take(US_Id id, US2_Reading *out);                  /* true once per newly published reading */
const char *US2_StatusText(US2_Status status);

#endif
