#ifndef MOCK_PRIMASK_H
#define MOCK_PRIMASK_H
#include <stdint.h>
/* Single-thread host stand-in for the Cortex-M interrupt mask. */
extern uint32_t mock_primask;
static inline uint32_t __get_PRIMASK(void) { return mock_primask; }
static inline void __disable_irq(void) { mock_primask = 1u; }
static inline void __set_PRIMASK(uint32_t mask) { mock_primask = mask; }
#endif
