#ifndef TR_DRIVE_H
#define TR_DRIVE_H

/**
 * Titan Robot (TR) drive: 3 omni wheels (2 front, 1 back, 120 deg apart), each
 * driven by an RM M3508 + C620 over CAN.
 *
 * Receives text lines (LF-terminated) from the Controller over the HC-05 (USART1, 115200):
 *   M+0.50/-0.25   left stick:  X = strafe (+ right), Y = forward (+ forward), ratios -1.00..+1.00
 *   R-0.40         right stick: rotation, + = clockwise, ratio -1.00..+1.00
 *   GO / GC / GS   mechanism (arm servo): open / close / stop. Repeated every 50 ms while a button is held.
 * (This is the format used by the warmaster726 Controller firmware.)
 * The older fixed-step format "DRV:<vx>,<vy>,<w>,<a>" is still accepted (bench tests).
 *
 * Each wheel has a velocity PID (the RM motors only take a CURRENT command).
 * Failsafe: a command not refreshed for TR_LINK_TIMEOUT_MS is treated as released (movement 0, mechanism stop).
 */

#include "main.h"
#include <stdbool.h>

/* ---- Speeds, in WHEEL-shaft rpm (after the 19:1 gearbox). Start low. ---- */
#define TR_TRANSLATE_RPM    100.0f  /* wheel rpm for pure forward/back/strafe (~0.8 m/s with 152 mm omni wheels) */
#define TR_ROTATE_RPM        60.0f  /* wheel rpm for pure rotation */
/* Speed levels 1..10 chosen with the Controller's speed buttons. Wheel rpm at full stick, per level.
 * Level 4 equals the TR_TRANSLATE_RPM / TR_ROTATE_RPM above. TR_MAX_WHEEL_RPM below still caps every wheel. */
#define TR_SPEED_LEVELS 10
#define TR_SPEED_DEFAULT_LEVEL 4
#define TR_SPEED_TRANSLATE_RPM {40.0f, 60.0f, 80.0f, TR_TRANSLATE_RPM, 130.0f, 160.0f, 200.0f, 250.0f, 320.0f, 400.0f}
#define TR_SPEED_ROTATE_RPM {24.0f, 36.0f, 48.0f, TR_ROTATE_RPM, 78.0f, 96.0f, 120.0f, 150.0f, 192.0f, 240.0f}
#define TR_MAX_WHEEL_RPM    400.0f  /* if a mix needs more than this on any wheel, all are scaled down together */
#define TR_ACCEL_RPM_PER_S  600.0f  /* acceleration limit on the wheel targets (wheel rpm per second) */

/* ---- Velocity PID. Input: speed error in WHEEL-shaft rpm (after the gearbox). Output: motor current in amps.
 * Same units as the motor tuner on the warmaster726 branch, so its results can be used as they are.
 * Current values (from a groupmate): Kp 0.106, Ki 0.041, Kd 0. TUNE on the real robot if a wheel hums or lags. ---- */
#define TR_KP   0.106f   /* A per wheel-rpm of error */
#define TR_KI   0.041f   /* A per (wheel-rpm * s) */
#define TR_KD   0.0f     /* A per (wheel-rpm / s) */
#define TR_CURRENT_LIMIT_A  5.0f  /* our own cap per motor (rm_motor.h also caps at 5 A) */

#define TR_LINK_TIMEOUT_MS  350U  /* a command not refreshed for this long is treated as released */

/* ---- Bench test (no Bluetooth / no Controller) ----
 * 0 = normal: drive from the Controller over Bluetooth.
 * 1 = bench DRIVE: ignore the Controller; MainBoard BTN1 held = forward, BTN2 held = back.
 * 2 = bench ARM:   ignore the Controller; MainBoard BTN1 held = arm toward GRIP, BTN2 held = toward OUT.
 * 3 = bench BOTH:  ignore the Controller; BTN1 held = wheels forward AND arm toward GRIP,
 *                  BTN2 held = wheels back AND arm toward OUT. KEEP THE ROBOT ON BLOCKS.
 *     (needs TR_ARM_ENABLED 1 in tr_arm.h and the servo timer enabled in CubeMX)
 * Both bench modes use the normal code underneath (PID, ramp, CAN, servo limits).
 * SET THIS TO 0 once the Controller link is in use, or the robot will ignore the Controller. */
#ifndef TR_BENCH_TEST
#define TR_BENCH_TEST 0   /* normal: Controller over Bluetooth. Use 1/2/3 only for bench tests without it. */
#endif

/* ---- Wheel polarity: +1 or -1 per wheel. Calibrate on the robot (see instructions). ---- */
#define TR_FL_SIGN  (1.0f)
#define TR_FR_SIGN  (1.0f)
#define TR_B_SIGN   (1.0f)

/* ---- Direction flips (quick fixes for a robot that moves the wrong way). 0 = as coded, 1 = reversed.
 *   TR_INVERT_VX : strafe  (buttons 3/2 slide the wrong way)
 *   TR_INVERT_VY : forward (buttons 4/1 go the wrong way)
 *   TR_INVERT_W  : rotate  (buttons 7/6 spin the wrong way)
 * These flip the COMMAND, not individual wheels. If rotation itself is wrong or the robot wobbles instead of
 * spinning cleanly, an individual wheel sign (TR_*_SIGN above) is wrong instead. */
#ifndef TR_INVERT_VX
#define TR_INVERT_VX 0
#endif
#ifndef TR_INVERT_VY
#define TR_INVERT_VY 0
#endif
#ifndef TR_INVERT_W
#define TR_INVERT_W 0
#endif

/* ---- Encoder distance test ----
 * 1 = Controller button 4 drives the robot 50 cm FORWARD by itself and button 1 drives it 50 cm BACK ('T+' / 'T-'
 *     lines), using the wheel encoders for distance. The screen shows the measured distance in mm so you can
 *     compare it with a tape measure. The arm (buttons 7/8) still works. Moving a joystick, or losing the link,
 *     aborts the move.
 * 0 = disabled. */
#ifndef TR_ENCODER_TEST
#define TR_ENCODER_TEST 0
#endif
#define TR_WHEEL_DIAMETER_M   0.1524f  /* 152.4 mm omni wheels */
#define TR_ENCODER_SIGN       (1.0f)   /* set to (-1.0f) if the reported distance has the wrong sign (e.g. -1000 mm when it drove forward) */
#define TR_AUTO_DISTANCE_M    0.5f     /* length of the automatic move (0.5 m = 50 cm) */
#define TR_AUTO_MAX_FRAC      0.5f     /* top speed of the automatic move, as a fraction of TR_TRANSLATE_RPM (0.5 = ~0.4 m/s) */
#define TR_AUTO_MIN_FRAC      0.12f    /* creep speed near the end so it can still finish */
#define TR_AUTO_DECEL_ZONE_M  0.25f    /* start slowing down this far from the target */
#define TR_AUTO_TOL_M         0.010f   /* stop when within 10 mm */
#define TR_AUTO_TIMEOUT_MS    12000U

/* Automatic-move state (shown on the TFT). */
/* ---- Heading estimate, turn moves and heading-relative driving ----
 * The heading is integrated from the three wheel speeds (no gyro): the mean of the three wheel speeds in the mix's
 * convention is the rotation, because the three drive directions sum to zero. It drifts when the wheels slip. */
#define TR_ROBOT_RADIUS_M     0.273f   /* effective centre-to-wheel radius. CALIBRATE: new = old x (commanded angle / measured angle). Was 0.25: a "90" turned only ~82 deg */
#define TR_HEADING_SIGN       (-1.0f)  /* heading is clockwise-positive. -1 = a positive R command turns the robot counter-clockwise.
                                          If the on-screen heading goes DOWN when the robot turns clockwise, flip this. */
#define TR_TURN_ACCEL_RPM_PER_S 200.0f /* gentler wheel ramp while turning: hard starts make omni wheels slip and the turn comes up short */
#define TR_TURN_MAX_FRAC      0.5f     /* fastest turn, fraction of TR_ROTATE_RPM */
#define TR_TURN_MIN_FRAC      0.15f    /* creep speed near the end */
#define TR_TURN_DECEL_DEG     40.0f    /* start slowing this far from the target */
#define TR_TURN_TOL_DEG       0.7f     /* stop when within this many degrees (each turn ends up to this much short, so 4 x 90 adds up) */
#define TR_TURN_TIMEOUT_MS    8000U
enum { TR_TURN_IDLE = 0, TR_TURN_RUN = 1, TR_TURN_DONE = 2, TR_TURN_ABORT = 3, TR_TURN_NOFB = 4 };

/* ---- SIDEWAYS TEST on Controller buttons 2 / 3 (the Controller is unchanged: it still sends 'YL' / 'YR') ----
 * 1 = button 2 slides the TR TR_SIDE_TEST_M to the LEFT, button 3 slides it TR_SIDE_TEST_M to the RIGHT, with exactly the
 *     motion routine 2's diagonal uses (TRA_TRANSLATE_RPM, up to TR_SIDE_MAX_FRAC, the last 150 mm at 0.3, heading held).
 *     Tape-measure how far it really went and compare with the drive page's row 2: "SD" = the encoders' sideways mm,
 *     "F" = forward drift mm, "H" = heading change seen by the wheels. Both together (180) still turns. A stick movement
 *     stops it. 0 = buttons 2 / 3 turn 90 left / right as before. */
#ifndef TR_SIDE_TEST
#define TR_SIDE_TEST 1
#endif
#define TR_SIDE_TEST_M     0.5f
#define TR_SIDE_MAX_FRAC   0.7f   /* = TRB_MOVE_SPEED, routine 2's diagonal */
#define TR_SIDE_TIMEOUT_MS 6000U

enum { TR_AUTO_IDLE = 0, TR_AUTO_RUN = 1, TR_AUTO_DONE = 2, TR_AUTO_ABORT = 3, TR_AUTO_NOFB = 4 };

/* ---- Which CAN bus the back wheel's ESC (ID 3) is on ----
 * The board has two separate CAN buses. On this robot the front-left (ID 1) and front-right (ID 2) ESCs are on the
 * CAN1 connectors and the BACK ESC (ID 3) is on a CAN2 connector. 1 = drive the back wheel over CAN2 (needs CAN2
 * enabled in CubeMX: 1 Mbps, PB12/PB13, RX0 interrupt). 0 = everything on CAN1. */
/* Which CAN bus each wheel's ESC is wired to (the ESC IDs stay 1 = left, 2 = right, 3 = back on their own bus).
 * Current robot: ALL THREE motors (left ID 1, right ID 2, back ID 3) are on CAN2 (ID 2 moved from CAN1 for hardware
 * reliability). CAN1 must stay enabled in CubeMX anyway: on the STM32F405 CAN2 shares CAN1's filters and clock.
 * Set the flag to 1 for CAN2, 0 for CAN1. The MainBoard screen row "C1:xx C2:xx" shows which IDs it hears on each bus. */
#ifndef TR_LEFT_ON_CAN2
#define TR_LEFT_ON_CAN2 1
#endif
#ifndef TR_RIGHT_ON_CAN2
#define TR_RIGHT_ON_CAN2 1
#endif
#ifndef TR_BACK_ON_CAN2
#define TR_BACK_ON_CAN2 1
#endif
#define TR_USE_CAN2 (TR_LEFT_ON_CAN2 || TR_RIGHT_ON_CAN2 || TR_BACK_ON_CAN2) /* CAN2 must be started if any wheel uses it */
#define TR_USE_CAN1 (!(TR_LEFT_ON_CAN2 && TR_RIGHT_ON_CAN2 && TR_BACK_ON_CAN2)) /* commands are sent on CAN1 only if a wheel is on it
                                                                                   (CAN1 is still started: CAN2 needs it) */

/* Index order used everywhere: front-left, front-right, back. */
enum { TR_WHEEL_FL = 0, TR_WHEEL_FR = 1, TR_WHEEL_B = 2, TR_WHEEL_COUNT = 3 };

typedef struct {
  bool link_ok;                        /* a valid packet arrived within TR_LINK_TIMEOUT_MS */
  int8_t vx, vy, w, a;                 /* sign of the last command: -1 / 0 / +1 (0 if link lost) */
  int16_t move_x_pct, move_y_pct, rot_pct; /* joystick ratios in percent, -100..100 (strafe right, forward, clockwise) */
  uint16_t arm_us;                     /* arm servo pulse width in microseconds */
  uint32_t rx_bytes;                   /* debug: bytes received on USART1 */
  uint32_t pkt_ok;                     /* debug: valid packets */
  uint32_t pkt_bad;                    /* debug: complete but invalid lines */
  uint32_t uart_err;                   /* debug: UART hardware errors */
  uint8_t auto_state;                  /* TR_AUTO_* */
  int32_t dist_mm;                     /* encoder distance moved since the automatic move started, mm (+ = commanded forward) */
  float target_rpm[TR_WHEEL_COUNT];    /* wheel-shaft rpm the PID is chasing (after ramp, with sign) */
  float actual_rpm[TR_WHEEL_COUNT];    /* measured wheel-shaft rpm */
  float current_a[TR_WHEEL_COUNT];     /* current command sent to each ESC */
  int16_t heading_deg;                 /* estimated heading in degrees, clockwise positive, since power-up or the last zero */
  uint8_t heading_mode;                /* 1 = heading-relative driving is on */
  uint8_t turn_state;                  /* TR_TURN_* */
  uint8_t side_state;                  /* sideways test: TR_TURN_* codes (0 = never run) */
  int16_t side_mm, side_fwd_mm;        /* sideways test: encoder sideways / forward mm since its start (keeps counting the coast) */
  int16_t side_hdg_deg;                /* sideways test: heading change the wheels saw */
  uint8_t speed_level;                 /* 1..10, set by the Controller's speed buttons ('S<n>' line, n = level-1) */
  uint8_t can1_ids, can2_ids;          /* bit n set = an ESC with ID n+1 has answered on that bus (any ID 1-8) */
  uint8_t temp_c[TR_WHEEL_COUNT];      /* ESC temperature from CAN feedback; 0 = no feedback ever received from that motor */
} TRDrive_Status;

/** Start CAN + UART reception. Call once, after the generated MX_CAN1_Init() and MX_USART1_UART_Init(). */
void TRDrive_Init(void);

/** Call every main-loop pass (runs the control loop every 1 ms). */
void TRDrive_Update(void);

/** Copy out the current status (for the TFT). */
void TRDrive_GetStatus(TRDrive_Status *out);

/* ---- Hooks for the autonomous routine (tr_auto_app.c) ----
 * While active, the joystick is ignored: the robot is driven by these ratios (same meaning as the stick: x strafe
 * right, y forward, w rotation) and the arm command (+1 close, -1 open, 0 hold). */
void TRDrive_AutoSet(bool active, float x, float y, float w, int8_t arm);
/* Start a heading turn by delta degrees (clockwise positive); progress is in TRDrive_Status.turn_state. */
void TRDrive_AutoTurn(float delta_cw_deg);
void TRDrive_AutoTurnAt(float delta_cw_deg, float max_frac); /* the same, with its own top speed (fraction of TR_ROTATE_RPM) */
float TRDrive_ForwardDistanceM(void);  /* encoder distance along the robot's forward direction, since power-up */
/* Each wheel's travel from its encoder since power-up, in mm at the wheel rim ([0] FL, [1] FR, [2] B; + = the wheel's
 * forward rotation as the mixer commands it). */
void TRDrive_WheelTravelMm(int32_t out_mm[TR_WHEEL_COUNT]);
float TRDrive_StrafeDistanceM(void);   /* encoder distance sideways (+ = right, the stick's strafe direction), since power-up */
float TRDrive_HeadingDeg(void);        /* unwrapped heading estimate, clockwise positive */
void TRDrive_ZeroHeading(void);

#endif
