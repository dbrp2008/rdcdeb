#ifndef TR_DRIVE_H
#define TR_DRIVE_H

/**
 * Titan Robot (TR) drive: 3 omni wheels (2 front, 1 back, 120 deg apart), each
 * driven by an RM M3508 + C620 over CAN.
 *
 * Receives text lines (LF-terminated) from the Controller over the HC-05 (USART1, 115200):
 *   M+0.50/-0.25   left stick:  X = strafe (+ right), Y = forward (+ forward), ratios -1.00..+1.00
 *   R-0.40         right stick: rotation ratio -1.00..+1.00 (physical direction must be verified)
 *   GO / GC / GS   mechanism (arm servo): open / close / stop. Repeated every 50 ms while a button is held.
 * The user Controller is `firmware/controller/`; it uses the same command lines and parses MainBoard ACKs.
 * For each valid command, this MainBoard returns a checksummed `@A,<sequence>,00*<xor>\n` ACK.
 *
 * Each wheel has a velocity PID (the RM motors only take a CURRENT command).
 * Failsafe: a command not refreshed for TR_LINK_TIMEOUT_MS is treated as released (movement 0, mechanism stop).
 */

#include "main.h"
#include <stdbool.h>
#include "tr_turn.h"

/* ---- Speed targets, in WHEEL-shaft rpm (gearbox output). The motor tuner
 * targets 4,000 motor-shaft rpm (about 210.5 output rpm using its 19:1 ratio).
 * TR uses its existing 19.2:1 M3508 conversion, so the matching wheel target
 * is 4,000 / 19.2 = 208.3 rpm. The tuner's 4,370 motor-rpm ceiling converts
 * to a 227.6 wheel-rpm command cap here, below its 230 output-rpm ceiling.
 * Rotation is limited to 60% of translation. Wheel targets are applied
 * directly, without an acceleration-slew limiter. ---- */
#define TR_MOTOR_GEAR_RATIO       19.2f
#define TR_TUNER_TARGET_MOTOR_RPM 4000.0f
#define TR_TUNER_MAX_MOTOR_RPM    4370.0f
#define TR_TRANSLATE_RPM (TR_TUNER_TARGET_MOTOR_RPM / TR_MOTOR_GEAR_RATIO)
#define TR_ROTATE_SPEED_RATIO 0.60f
#define TR_ROTATE_RPM    (TR_TRANSLATE_RPM * TR_ROTATE_SPEED_RATIO)
#define TR_MAX_WHEEL_RPM (TR_TUNER_MAX_MOTOR_RPM / TR_MOTOR_GEAR_RATIO)

/* Piecewise-linear joystick response: 60% stick deflection reaches 40% speed;
 * the remaining 40% of travel spans 40-100% output. */
#define TR_STICK_CURVE_BREAKPOINT    0.60f
#define TR_STICK_CURVE_INNER_OUTPUT  0.40f

/* ---- Shared velocity PID gains applied to all three wheels.
 * Input: wheel-shaft RPM error; output: commanded current in amps.
 * Restored to the prior TR set: Kp 0.106, Ki 0.041, Kd 0.0. ---- */
#define TR_KP   0.106f  /* A per wheel-rpm of error */
#define TR_KI   0.041f  /* A per (wheel-rpm * s) */
#define TR_KD   0.0f    /* A per (wheel-rpm / s) */
#define TR_CURRENT_LIMIT_A  5.0f  /* same ±5 A software cap per motor */

#define TR_LINK_TIMEOUT_MS  350U  /* a command not refreshed for this long is treated as released */

/* ---- Wheel polarity: +1 or -1 per wheel. Calibrate on the robot (see instructions). ---- */
#define TR_FL_SIGN  (1.0f)
#define TR_FR_SIGN  (1.0f)
#define TR_B_SIGN   (1.0f)

/* ---- Direction flips (quick fixes for a robot that moves the wrong way). 0 = as coded, 1 = reversed.
 *   TR_INVERT_VX : left-stick strafe
 *   TR_INVERT_VY : left-stick forward/back
 *   TR_INVERT_W  : right-stick rotation
 * These flip the COMMAND, not individual wheels. If rotation itself is wrong or the robot wobbles instead of
 * spinning cleanly, an individual wheel sign (TR_*_SIGN above) is wrong instead. */
#ifndef TR_INVERT_VX
#define TR_INVERT_VX 0
#endif
#ifndef TR_INVERT_VY
#define TR_INVERT_VY 0
#endif
#ifndef TR_INVERT_W
#define TR_INVERT_W 1 /* reverse the previous warmaster726 rotation mapping */
#endif

/* ---- Which CAN bus the back wheel's ESC (ID 3) is on ----
 * The board has two separate CAN buses. On this robot the front-left (ID 1) and front-right (ID 2) ESCs are on the
 * CAN1 connectors and the BACK ESC (ID 3) is on a CAN2 connector. 1 = drive the back wheel over CAN2 (needs CAN2
 * enabled in CubeMX: 1 Mbps, PB12/PB13, RX0 interrupt). 0 = everything on CAN1. */
#ifndef TR_BACK_ON_CAN2
#define TR_BACK_ON_CAN2 1
#endif

/* Index order used everywhere: front-left, front-right, back. */
enum { TR_WHEEL_FL = 0, TR_WHEEL_FR = 1, TR_WHEEL_B = 2, TR_WHEEL_COUNT = 3 };

typedef struct {
  TRTurnState turn_state;
  bool heading_requested;
  bool heading_valid;
  int16_t heading_deg;
  bool link_ok;                        /* a valid packet arrived within TR_LINK_TIMEOUT_MS */
  int8_t vx, vy, w, a;                 /* sign of the last command: -1 / 0 / +1 (0 if link lost) */
  int16_t move_x_pct, move_y_pct, rot_pct; /* joystick ratios in percent, -100..100 (strafe right, forward, clockwise) */
  uint16_t arm_us;                     /* arm servo pulse width in microseconds */
  uint32_t rx_bytes;                   /* debug: bytes received on USART1 */
  uint32_t pkt_ok;                     /* debug: valid packets */
  uint32_t pkt_bad;                    /* debug: complete but invalid lines */
  uint32_t uart_err;                   /* debug: UART hardware errors */
  float target_rpm[TR_WHEEL_COUNT];    /* wheel-shaft rpm target after motion averaging, with sign */
  float actual_rpm[TR_WHEEL_COUNT];    /* measured wheel-shaft rpm */
  float current_a[TR_WHEEL_COUNT];     /* commanded current for each ESC */
  uint8_t can1_ids, can2_ids;          /* fresh feedback: bit n = ESC ID n+1 replied within 100 ms */
  uint8_t can1_init_ok, can2_init_ok;  /* CAN filter/start/notification setup completed */
  uint8_t can1_tx_ok, can2_tx_ok;      /* most recent pair of command frames queued successfully */
  uint32_t can1_tx_failures, can2_tx_failures; /* latched command-queue failure counts */
  uint8_t temp_c[TR_WHEEL_COUNT];      /* ESC temperature from CAN feedback; 0 can mean no reply */
} TRDrive_Status;

/** Start CAN + UART reception. Call once, after the generated MX_CAN1_Init() and MX_USART1_UART_Init(). */
void TRDrive_Init(void);

/** Call every main-loop pass (runs the control loop every 1 ms). */
void TRDrive_Update(void);

/** Copy out the current status (for the TFT). */
void TRDrive_GetStatus(TRDrive_Status *out);

#endif
