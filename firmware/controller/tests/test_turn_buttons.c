#include "turn_buttons.h"
#include <assert.h>
#include <stdint.h>

int main(void)
{
  TurnButtons b = {RDC_MESSAGE_TURN_IDLE, RDC_MESSAGE_TURN_IDLE, 0, 0, 0};
  turn_buttons_update(&b, 1, 0, 10); assert(b.output == RDC_MESSAGE_TURN_IDLE);
  turn_buttons_update(&b, 1, 1, 60); assert(b.output == RDC_MESSAGE_TURN_IDLE);
  turn_buttons_update(&b, 1, 1, 179); assert(b.output == RDC_MESSAGE_TURN_IDLE);
  turn_buttons_update(&b, 1, 1, 180); assert(b.output == RDC_MESSAGE_TURN_HALF);
  turn_buttons_update(&b, 1, 1, 379); assert(b.output == RDC_MESSAGE_TURN_HALF);
  turn_buttons_update(&b, 1, 1, 380); assert(b.output == RDC_MESSAGE_TURN_IDLE);
  turn_buttons_update(&b, 1, 0, 500); assert(b.output == RDC_MESSAGE_TURN_IDLE);
  turn_buttons_update(&b, 0, 0, 501);
  turn_buttons_update(&b, 1, 0, 510);
  turn_buttons_update(&b, 1, 0, 630); assert(b.output == RDC_MESSAGE_TURN_LEFT);
  turn_buttons_update(&b, 0, 0, 631); assert(b.output == RDC_MESSAGE_TURN_IDLE);
  turn_buttons_update(&b, 0, 1, UINT32_MAX - 100U);
  turn_buttons_update(&b, 0, 1, 19U); assert(b.output == RDC_MESSAGE_TURN_RIGHT);
  turn_buttons_update(&b, 0, 1, 219U); assert(b.output == RDC_MESSAGE_TURN_IDLE);
  return 0;
}
