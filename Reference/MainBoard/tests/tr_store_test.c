/* Host test of the recording storage (tr_store.c) on a fake flash sector (writes can only clear bits, as on the chip).
 * Build: gcc -DTRS_HOST -ICore/Inc -o tr_store_test tests/tr_store_test.c Core/Src/tr_store.c Core/Src/tr_record.c -lm */
#include <stdio.h>
#include <string.h>
#include "tr_store.h"
static int fails;
#define CHECK(c) do{ if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);fails++;} }while(0)
uint8_t trs_host_flash[TRS_SIZE];
uint32_t trs_host_program_end = 0x08040000u; /* the program ends well below sector 11 */
static TRR_Sample s[TRR_MAX_SAMPLES];
int main(void) {
  uint16_t n = 0;
  memset(trs_host_flash, 0xFF, sizeof trs_host_flash);            /* an erased sector */
  CHECK(TRStore_Load(&n) == NULL && n == 0);                       /* "no recording" */
  for (unsigned i = 0; i < TRR_MAX_SAMPLES; i++) { s[i].x_mm = (int16_t)(i * 3); s[i].y_mm = (int16_t)(-(int)i); s[i].hdg_ddeg = (int16_t)(i % 900); s[i].arm_us = (uint16_t)(1611 + i % 889); }
  CHECK(TRStore_Save(s, 1) == TRS_ERR_EMPTY);
  CHECK(TRStore_Save(s, 700) == TRS_OK);
  const TRR_Sample *r = TRStore_Load(&n);
  CHECK(r != NULL && n == 700 && memcmp(r, s, 700 * sizeof(TRR_Sample)) == 0);
  /* a second save replaces the first (the sector is erased first, so the old bits do not mix in) */
  s[5].x_mm = 12345;
  CHECK(TRStore_Save(s, TRR_MAX_SAMPLES) == TRS_OK);
  r = TRStore_Load(&n);
  CHECK(r != NULL && n == TRR_MAX_SAMPLES && r[5].x_mm == 12345 && memcmp(r, s, sizeof s) == 0);
  /* one flipped byte in the data: rejected */
  trs_host_flash[sizeof(TRS_Header) + 100] ^= 0x01;
  CHECK(TRStore_Load(&n) == NULL && n == 0);
  /* a save interrupted before the header (power loss): rejected */
  memset(trs_host_flash, 0xFF, sizeof trs_host_flash);
  memcpy(trs_host_flash + sizeof(TRS_Header), s, 700 * sizeof(TRR_Sample));
  CHECK(TRStore_Load(&n) == NULL);
  /* header written except the magic word (the last thing written): rejected */
  CHECK(TRStore_Save(s, 700) == TRS_OK);
  memset(trs_host_flash, 0xFF, 4);
  CHECK(TRStore_Load(&n) == NULL);
  /* the program reaching sector 11: never erased */
  CHECK(TRStore_Save(s, 700) == TRS_OK);
  trs_host_program_end = TRS_ADDR + 4u;
  memset(trs_host_flash + 1000, 0x00, 8);                          /* mark: must survive the refused save */
  CHECK(TRStore_Save(s, 700) == TRS_ERR_OVERLAP);
  CHECK(trs_host_flash[1000] == 0x00);
  printf("  sizeof header %u, sample %u, max recording %u bytes of %u\n", (unsigned)sizeof(TRS_Header), (unsigned)sizeof(TRR_Sample),
         (unsigned)(sizeof(TRS_Header) + TRR_MAX_SAMPLES * sizeof(TRR_Sample)), (unsigned)TRS_SIZE);
  CHECK(sizeof(TRS_Header) == 20 && sizeof(TRR_Sample) == 8);
  printf(fails ? "%d FAILED\n" : "TR STORE PASS\n", fails);
  return fails != 0;
}
