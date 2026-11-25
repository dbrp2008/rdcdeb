#include "enc_watch.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void) {
  EncWatch w; EncWatch_Reset(&w);
  uint32_t t = 1000, r = 0, l = 0;
  /* both turning: never trips, however long */
  for (int i = 0; i < 5000; i++, t++) { if (i % 7 == 0) { r++; l++; } assert(!EncWatch_Update(&w, t, 22.5f, r, -22.5f, l)); }
  /* right stops giving pulses while still commanded: trips at ENC_WATCH_MS after its last pulse, not before */
  const uint32_t last = t - 1u + 0u; /* r last changed at most 7 ticks ago */
  const char *s = NULL; uint32_t at = 0;
  for (int i = 0; i < 3000 && !s; i++, t++) { if (i % 7 == 0) l++; s = EncWatch_Update(&w, t, 22.5f, r, -22.5f, l); at = t; }
  assert(s && !strcmp(s, "R NO PULSES") && at - last >= ENC_WATCH_MS - 7u && at - last <= ENC_WATCH_MS);
  /* a slow wheel below the minimum duty is not checked (the inside of a held turn, a pivot's reverse wheel) */
  EncWatch_Reset(&w);
  for (int i = 0; i < 5000; i++, t++) { if (i % 7 == 0) r++; assert(!EncWatch_Update(&w, t, 23.85f, r, 9.9f, l)); }
  /* a pause at 0 % (target hold) restarts the timing: the first second after it is allowed to spin up */
  EncWatch_Reset(&w);
  for (int i = 0; i < 900; i++, t++) assert(!EncWatch_Update(&w, t, 22.5f, r, 22.5f, l));
  for (int i = 0; i < 2000; i++, t++) assert(!EncWatch_Update(&w, t, 0.0f, r, 0.0f, l));
  for (int i = 0; i < (int)ENC_WATCH_MS; i++, t++) assert(!EncWatch_Update(&w, t, 22.5f, r, 22.5f, l)); /* armed at the first */
  s = EncWatch_Update(&w, t, 22.5f, r, 22.5f, l); assert(s && !strcmp(s, "NO PULSES L+R"));
  /* only the left dead; a reversing wheel (negative duty) counts the same */
  EncWatch_Reset(&w);
  for (int i = 0; i <= (int)ENC_WATCH_MS && !(s = EncWatch_Update(&w, t, -18.0f, r, -18.0f, l)); i++, t++) if (i % 5 == 0) r++;
  assert(s && !strcmp(s, "L NO PULSES"));
  for (unsigned i = 0; i < 3u; i++) assert(strlen(i == 0 ? "R NO PULSES" : i == 1 ? "L NO PULSES" : "NO PULSES L+R") <= 16u);
  puts("Encoder watch: trips after ENC_WATCH_MS without pulses at >= ENC_WATCH_MIN_PCT, per wheel; slow wheels and pauses exempt PASS");
  return 0;
}
