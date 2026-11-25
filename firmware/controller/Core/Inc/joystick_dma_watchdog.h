#ifndef JOYSTICK_DMA_WATCHDOG_H
#define JOYSTICK_DMA_WATCHDOG_H

#include <stdint.h>

/* DMA transfer-complete is polled, not interrupted: a continuous four-channel
 * scan would otherwise generate an interrupt for every scan. A fault is latched
 * until controller restart so a stalled joystick cannot resume motion alone. */
#define JOYSTICK_DMA_TIMEOUT_MS 200U

typedef enum { JOYSTICK_DMA_WAIT, JOYSTICK_DMA_OK, JOYSTICK_DMA_FAULT } JoystickDmaState;
typedef struct {
  JoystickDmaState state;
  uint32_t last_progress_tick;
} JoystickDmaWatchdog;

static inline void joystick_dma_begin(JoystickDmaWatchdog *watchdog, uint32_t now, int started)
{
  watchdog->state = started ? JOYSTICK_DMA_WAIT : JOYSTICK_DMA_FAULT;
  watchdog->last_progress_tick = now;
}

static inline void joystick_dma_observe(JoystickDmaWatchdog *watchdog, uint32_t now,
                                        int running, int transfer_error, int completed_scan)
{
  if (watchdog->state == JOYSTICK_DMA_FAULT) return;
  /* Test timeout before accepting a potentially old, latched completion flag. */
  if (!running || transfer_error ||
      (uint32_t)(now - watchdog->last_progress_tick) > JOYSTICK_DMA_TIMEOUT_MS) {
    watchdog->state = JOYSTICK_DMA_FAULT;
  } else if (completed_scan) {
    watchdog->last_progress_tick = now;
    watchdog->state = JOYSTICK_DMA_OK;
  }
}
#endif
