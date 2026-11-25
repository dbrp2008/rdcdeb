#ifndef TR_STORE_H
#define TR_STORE_H

/**
 * Keeps ONE teach-and-replay recording in the STM32F405's last flash sector (sector 11, 128 KB at 0x080E0000), so it
 * survives power-off and normal ST-Link flashing (which only erases the sectors the new program needs). The linker script
 * reserves that sector (FLASH LENGTH = 896K), and TRStore_Save also refuses to erase it if the program ever reaches it.
 * A "full chip erase" (e.g. STM32CubeProgrammer "Erase all") deletes the recording.
 *
 * Layout: header (magic, version, period, count, CRC of the samples, CRC of the header), then the samples. The header is
 * written LAST: a save cut short by a power loss leaves no valid header, so TRStore_Load reports "no recording".
 * Saving erases the sector first: that takes 1-2 s with the CPU FROZEN (the program runs from the same flash), so only
 * save with the TR standing still.
 */

#include <stdint.h>
#include "tr_record.h"

#ifndef TRS_ADDR
#define TRS_ADDR 0x080E0000u /* sector 11 */
#endif
#define TRS_SIZE (128u * 1024u)
#define TRS_MAGIC 0x54525243u /* "TRRC" */
#define TRS_VERSION 1u

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t period_ms;
  uint32_t count;
  uint32_t crc;      /* of the samples */
  uint32_t hcrc;     /* of the 16 bytes above */
} TRS_Header;        /* 20 bytes, then count x TRR_Sample */

typedef enum {
  TRS_OK = 0,
  TRS_ERR_EMPTY,      /* nothing to save */
  TRS_ERR_OVERLAP,    /* the program reaches the recording sector: refusing to erase it */
  TRS_ERR_ERASE,
  TRS_ERR_WRITE,
  TRS_ERR_VERIFY      /* written, but reading it back does not match */
} TRS_Result;

TRS_Result TRStore_Save(const TRR_Sample *s, uint16_t n);
/* Returns the samples (pointing straight into flash) and their number, or 0 / NULL if there is no valid recording. */
const TRR_Sample *TRStore_Load(uint16_t *n);
const char *TRStore_ResultText(TRS_Result r);

#endif
