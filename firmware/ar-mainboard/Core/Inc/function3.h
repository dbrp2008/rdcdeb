#ifndef FUNCTION3_H
#define FUNCTION3_H
#include <stdint.h>

/* Dormant F3 tape policy. No HAL or motor writes. raw/filtered masks use
 * bit 0 = left, bit 1 = centre, bit 2 = right. Only fresh scans advance it. */
#define F3_US1_STOP_MM 40            /* immediate terminal stop; not the F1/F2 approach */
#define F3_CENTRE_CONFIRM_MS 20u
#define F3_FILTER_WAIT_MS 50u
#define F3_TURN_LIMIT_MS 2500u
#define F3_PIVOT_LEFT_PERCENT 15.0f   /* physical forward */
#define F3_PIVOT_RIGHT_PERCENT 12.0f  /* physical reverse */

typedef enum {
  F3_WAIT_TAPE, F3_STRAIGHT, F3_STEER_LEFT, F3_STEER_RIGHT, F3_PIVOT_RIGHT,
  F3_LOST_LINE, F3_UNSTABLE_LINE, F3_TURN_TIMEOUT
} F3_Command;

void F3_Reset(void); /* initialize, or begin a newly accepted run */
void F3_Scan(uint32_t now, unsigned raw, unsigned filtered);
/* Check deadlines again before moving (a blocking sensor poll may have run). */
F3_Command F3_CommandAt(uint32_t now);
unsigned F3_RightTurns(void); /* selections this run, not unique junctions */
#endif
