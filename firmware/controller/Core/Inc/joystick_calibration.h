#ifndef RDC_JOYSTICK_CALIBRATION_H
#define RDC_JOYSTICK_CALIBRATION_H

#include <stdint.h>

/*
 * Replace these starter readings with measured ADC counts from your joystick.
 * For each axis, record the low endpoint, released/stick-center reading, and
 * high endpoint. Keep MIN < CENTRE < MAX. Values are not saved to flash or
 * transmitted over USART1; this header only configures the build.
 */
#define RDC_JOY_LX_MIN       2U
#define RDC_JOY_LX_CENTRE 2075U
#define RDC_JOY_LX_MAX    4095U

#define RDC_JOY_LY_MIN       0U
#define RDC_JOY_LY_CENTRE 1993U
#define RDC_JOY_LY_MAX    4094U

#define RDC_JOY_RX_MIN       1U
#define RDC_JOY_RX_CENTRE 1944U
#define RDC_JOY_RX_MAX    4095U

/* Right-stick Y is currently not used by the RDC wire mapping. */
#define RDC_JOY_RY_MIN       0U
#define RDC_JOY_RY_CENTRE 2083U
#define RDC_JOY_RY_MAX    4094U

/* Deadband is in raw ADC counts and is applied around each axis centre. */
#define RDC_JOY_DEADBAND 120U

/* Snap to full scale within this many ADC counts of a calibrated endpoint. */
#define RDC_JOY_ENDPOINT_SNAP_COUNTS 16U

#endif /* RDC_JOYSTICK_CALIBRATION_H */
