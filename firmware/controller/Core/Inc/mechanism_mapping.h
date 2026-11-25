#ifndef MECHANISM_MAPPING_H
#define MECHANISM_MAPPING_H

#include <stdint.h>
#include "rdc_protocol.h"

/* Exactly one debounced button must be pressed to command the mechanism. */
static inline RdcMessageType mechanism_command_for_buttons(uint8_t button7, uint8_t button8)
{
  if (button7 && !button8) return RDC_MESSAGE_MECHANISM_CLOSE;
  if (button8 && !button7) return RDC_MESSAGE_MECHANISM_OPEN;
  return RDC_MESSAGE_MECHANISM_STOP;
}
#endif
