#include "tr_store.h"
#include <stddef.h>
#include <string.h>

/* ---- the flash itself: real HAL on the board, a RAM array in the host test (TRS_HOST) ---- */
#ifdef TRS_HOST
extern uint8_t trs_host_flash[TRS_SIZE];   /* the test's fake sector */
extern uint32_t trs_host_program_end;      /* the test's fake "end of the program in flash" */
#define SECTOR_PTR ((const uint8_t *)trs_host_flash)
static uint32_t program_end(void) { return trs_host_program_end; }
static int flash_erase(void) { memset(trs_host_flash, 0xFF, TRS_SIZE); return 0; }
static int flash_word(uint32_t off, uint32_t w) {
  uint32_t old;
  memcpy(&old, &trs_host_flash[off], 4);
  w &= old; /* flash can only clear bits */
  memcpy(&trs_host_flash[off], &w, 4);
  return 0;
}
static void flash_begin(void) {}
static void flash_end(void) {}
#else
#include "main.h" /* HAL */
#define SECTOR_PTR ((const uint8_t *)TRS_ADDR)
/* end of what the program occupies in flash: the copy of the initialised data is the last thing the linker puts there */
extern uint32_t _sidata, _sdata, _edata;
static uint32_t program_end(void) { return (uint32_t)&_sidata + ((uint32_t)&_edata - (uint32_t)&_sdata); }
static void flash_begin(void) {
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                         FLASH_FLAG_PGSERR);
}
static void flash_end(void) {
  HAL_FLASH_Lock();
  /* drop anything the flash data cache kept from before the write, so the read-back sees the new data */
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_DATA_CACHE_ENABLE();
}
static int flash_erase(void) {
  FLASH_EraseInitTypeDef e;
  uint32_t bad = 0;
  memset(&e, 0, sizeof e);
  e.TypeErase = FLASH_TYPEERASE_SECTORS;
  e.Sector = FLASH_SECTOR_11;
  e.NbSectors = 1;
  e.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7-3.6 V: 32-bit writes */
  return (HAL_FLASHEx_Erase(&e, &bad) == HAL_OK && bad == 0xFFFFFFFFu) ? 0 : -1;
}
static int flash_word(uint32_t off, uint32_t w) {
  return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, TRS_ADDR + off, (uint64_t)w) == HAL_OK ? 0 : -1;
}
#endif

static uint32_t header_crc(const TRS_Header *h) { return TRR_Crc32(h, 16u, 0u); }

/* write len bytes (a multiple of 4) at offset off */
static int write_bytes(uint32_t off, const void *src, uint32_t len) {
  const uint8_t *b = (const uint8_t *)src;
  for (uint32_t i = 0; i < len; i += 4u) {
    uint32_t w;
    memcpy(&w, b + i, 4);
    if (flash_word(off + i, w) != 0) return -1;
  }
  return 0;
}

TRS_Result TRStore_Save(const TRR_Sample *s, uint16_t n) {
  if (s == NULL || n < 2u) return TRS_ERR_EMPTY;
  if (n > TRR_MAX_SAMPLES) n = TRR_MAX_SAMPLES;
  if (program_end() > TRS_ADDR) return TRS_ERR_OVERLAP; /* never erase a sector the program lives in */

  TRS_Header h;
  memset(&h, 0, sizeof h);
  h.magic = TRS_MAGIC;
  h.version = TRS_VERSION;
  h.period_ms = (uint16_t)TRR_PERIOD_MS;
  h.count = n;
  h.crc = TRR_Crc32(s, (uint32_t)n * (uint32_t)sizeof(TRR_Sample), 0u);
  h.hcrc = header_crc(&h);

  TRS_Result r = TRS_OK;
  flash_begin();
  if (flash_erase() != 0) r = TRS_ERR_ERASE;
  /* the samples first, the header last (the magic word last of all): an interrupted save leaves no valid header */
  else if (write_bytes(sizeof(TRS_Header), s, (uint32_t)n * (uint32_t)sizeof(TRR_Sample)) != 0) r = TRS_ERR_WRITE;
  else if (write_bytes(4u, (const uint8_t *)&h + 4, sizeof(TRS_Header) - 4u) != 0) r = TRS_ERR_WRITE;
  else if (write_bytes(0u, &h, 4u) != 0) r = TRS_ERR_WRITE;
  flash_end();
  if (r != TRS_OK) return r;

  uint16_t back = 0;
  const TRR_Sample *rs = TRStore_Load(&back);
  if (rs == NULL || back != n || memcmp(rs, s, (size_t)n * sizeof(TRR_Sample)) != 0) return TRS_ERR_VERIFY;
  return TRS_OK;
}

const TRR_Sample *TRStore_Load(uint16_t *n) {
  TRS_Header h;
  memcpy(&h, SECTOR_PTR, sizeof h);
  if (n) *n = 0;
  if (h.magic != TRS_MAGIC || h.version != TRS_VERSION || h.hcrc != header_crc(&h)) return NULL;
  if (h.period_ms != TRR_PERIOD_MS || h.count < 2u || h.count > TRR_MAX_SAMPLES) return NULL;
  const TRR_Sample *s = (const TRR_Sample *)(SECTOR_PTR + sizeof(TRS_Header));
  if (TRR_Crc32(s, h.count * (uint32_t)sizeof(TRR_Sample), 0u) != h.crc) return NULL;
  if (n) *n = (uint16_t)h.count;
  return s;
}

const char *TRStore_ResultText(TRS_Result r) {
  switch (r) {
    case TRS_OK: return "SAVED OK";
    case TRS_ERR_EMPTY: return "NOTHING TO SAVE";
    case TRS_ERR_OVERLAP: return "CODE IN SECTOR11";
    case TRS_ERR_ERASE: return "ERASE FAILED";
    case TRS_ERR_WRITE: return "WRITE FAILED";
    case TRS_ERR_VERIFY: return "VERIFY FAILED";
  }
  return "?";
}
