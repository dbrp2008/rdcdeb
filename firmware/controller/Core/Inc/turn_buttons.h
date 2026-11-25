#ifndef TURN_BUTTONS_H
#define TURN_BUTTONS_H
#include "rdc_protocol.h"
#include <stdint.h>
#define TURN_CHORD_MS 120U
#define TURN_REPEAT_MS 200U

typedef struct {
  RdcMessageType pattern, output;
  uint8_t latched;
  uint32_t pattern_since, send_until;
} TurnButtons;

/* Input buttons are already debounced. One turn per release, with 120 ms to
 * recognise a chord and a brief retransmit window for packet loss. */
static inline void turn_buttons_update(TurnButtons *b, uint8_t left, uint8_t right, uint32_t now)
{
  RdcMessageType pattern = left && right ? RDC_MESSAGE_TURN_HALF :
                           (left ? RDC_MESSAGE_TURN_LEFT :
                            (right ? RDC_MESSAGE_TURN_RIGHT : RDC_MESSAGE_TURN_IDLE));
  if (pattern != b->pattern) { b->pattern = pattern; b->pattern_since = now; }
  if (pattern == RDC_MESSAGE_TURN_IDLE) b->latched = 0U;
  if (b->latched && (int32_t)(now - b->send_until) < 0) return;
  if (b->latched || pattern == RDC_MESSAGE_TURN_IDLE || (uint32_t)(now - b->pattern_since) < TURN_CHORD_MS) {
    b->output = RDC_MESSAGE_TURN_IDLE;
  } else {
    b->output = pattern;
    b->latched = 1U;
    b->send_until = now + TURN_REPEAT_MS;
  }
}
#endif
