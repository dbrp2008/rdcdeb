#include "mechanism_mapping.h"
#include <assert.h>
#include <string.h>

static void check(uint8_t b7, uint8_t b8, RdcMessageType expected, const char *line)
{
  char frame[RDC_PROTOCOL_MAX_FRAME];
  RdcMessageType actual = mechanism_command_for_buttons(b7, b8);
  assert(actual == expected);
  size_t n = rdc_encode_mechanism(frame, sizeof(frame), actual);
  assert(n == strlen(line));
  assert(memcmp(frame, line, n) == 0);
}
int main(void)
{
  check(0, 0, RDC_MESSAGE_MECHANISM_STOP, "GS\n");
  check(1, 0, RDC_MESSAGE_MECHANISM_CLOSE, "GC\n");
  check(0, 1, RDC_MESSAGE_MECHANISM_OPEN, "GO\n");
  check(1, 1, RDC_MESSAGE_MECHANISM_STOP, "GS\n");
  return 0;
}
