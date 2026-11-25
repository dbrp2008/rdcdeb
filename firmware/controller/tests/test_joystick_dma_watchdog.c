#include "joystick_dma_watchdog.h"
#include <assert.h>
#include <stdint.h>

int main(void)
{
  JoystickDmaWatchdog w;
  joystick_dma_begin(&w, 10U, 0);
  joystick_dma_observe(&w, 11U, 1, 0, 1);
  assert(w.state == JOYSTICK_DMA_FAULT); /* failed start latches */
  joystick_dma_begin(&w, 10U, 1);
  assert(w.state == JOYSTICK_DMA_WAIT);
  joystick_dma_observe(&w, 11U, 1, 0, 0);
  assert(w.state == JOYSTICK_DMA_WAIT); /* no complete first scan */
  joystick_dma_observe(&w, 12U, 1, 0, 1);
  assert(w.state == JOYSTICK_DMA_OK);
  joystick_dma_observe(&w, 212U, 1, 0, 0);
  assert(w.state == JOYSTICK_DMA_OK);
  joystick_dma_observe(&w, 213U, 1, 0, 1);
  assert(w.state == JOYSTICK_DMA_FAULT); /* even a late latched flag cannot re-arm */
  joystick_dma_begin(&w, 0U, 1);
  joystick_dma_observe(&w, 1U, 1, 1, 1);
  assert(w.state == JOYSTICK_DMA_FAULT); /* transfer error wins */
  joystick_dma_begin(&w, 0U, 1);
  joystick_dma_observe(&w, 1U, 0, 0, 1);
  assert(w.state == JOYSTICK_DMA_FAULT); /* disabled DMA/ADC wins */
  joystick_dma_begin(&w, UINT32_MAX - 100U, 1);
  joystick_dma_observe(&w, 50U, 1, 0, 1);
  assert(w.state == JOYSTICK_DMA_OK); /* HAL tick wrap */
  joystick_dma_observe(&w, 251U, 1, 0, 0);
  assert(w.state == JOYSTICK_DMA_FAULT);
  return 0;
}
