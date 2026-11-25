#ifndef ENCODER_MONITOR_H
#define ENCODER_MONITOR_H
#include <stdbool.h>
#include <stdint.h>

/* One feedback wire per wheel: sign is rate above/below sampled idle, not
 * independently sensed shaft direction. No motor or direction command is used. */
typedef enum { ENCODER_SETTLING, ENCODER_PARTIAL, ENCODER_MEASURING } EncoderMode;
typedef struct {
  uint32_t edges, pps;            /* live unsigned edge total, last raw rate */
  uint32_t idle_pps, deadzone_pps; /* frozen BG +/- deadzone; both zero while waiting */
  int64_t net_edges, net_pps;     /* signed COUNT and NET/s on either wheel */
  bool rise_pending;             /* NET/s held at zero pending positive confirmation */
} EncoderWheel;
typedef struct { EncoderMode mode; EncoderWheel left_j3, right_j1; } EncoderReading;

/* Foreground, single consumer. Init clears profiles and net counts, not IRQ
 * totals, and starts waiting for stable feedback. A ready wheel measures even
 * while the other is settling. Its BG and deadzone stay fixed until re-arm. */
void EncoderMonitor_Init(void);
/* Re-arm both wheels, keeping credited counts. Ignored if neither is ready. */
bool EncoderMonitor_StartIdleSample(void);
/* Service between TFT frames; no IRQ masking if not due. */
void EncoderMonitor_Poll(uint32_t now_ms);
/* Also services due samples. RAW/s is unfiltered in all modes. */
EncoderReading EncoderMonitor_Read(void);
#endif
