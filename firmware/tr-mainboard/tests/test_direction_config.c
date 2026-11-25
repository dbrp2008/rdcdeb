/* Host check of the checked-in direction defaults, without STM32 HAL. */
#include <stdint.h>
#define __MAIN_H
#include "tr_drive.h"
#include <assert.h>

int main(void)
{
  assert(TR_INVERT_VX == 0);
  assert(TR_INVERT_VY == 0);
  assert(TR_INVERT_W == 1);
  assert(TR_FL_SIGN == 1.0f && TR_FR_SIGN == 1.0f && TR_B_SIGN == 1.0f);
  /* MainBoard parser negates incoming R; mixer applies TR_INVERT_W. */
  const float received_r = 0.5f;
  const float command_w = -received_r;
  const float mixer_w = TR_INVERT_W ? command_w : -command_w;
  assert(mixer_w < 0.0f); /* opposite to the former default, TR_INVERT_W=0 */
  return 0;
}
