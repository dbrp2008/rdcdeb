#include "tr_turn.h"
#include <assert.h>
#include <stdint.h>
int main(void)
{
  TRTurn t = {0};
  tr_turn_start(&t, 90, 0, 0, false); assert(t.state == TR_TURN_REJECT);
  tr_turn_start(&t, 90, 359, 0, true);
  assert(tr_turn_step(&t, 10, 1, true) > 0); /* 359 -> 1 is +2 degrees */
  assert(fabsf(t.travelled_deg - 2) < .001f);
  assert(tr_turn_step(&t, 100, 89, true) == 0 && t.state == TR_TURN_DONE);
  tr_turn_start(&t, -90, 0, 0, true);
  assert(tr_turn_step(&t, 1, 0, true) < 0);
  assert(tr_turn_step(&t, 2, 6, true) == 0 && t.state == TR_TURN_WRONG_WAY);
  tr_turn_start(&t, 180, 0, 0, true);
  assert(tr_turn_step(&t, 601, 0, true) == 0 && t.state == TR_TURN_NOFB);
  tr_turn_start(&t, 90, 0, 0, true);
  assert(tr_turn_step(&t, 1, 0, false) == 0 && t.state == TR_TURN_ABORT);
  tr_turn_start(&t, 90, 0, UINT32_MAX-10U, true);
  assert(tr_turn_step(&t, 5, 2, true) > 0);
  assert(tr_turn_step(&t, 8000, 3, true) == 0 && t.state == TR_TURN_ABORT);
  return 0;
}
