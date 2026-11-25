#ifndef MOTOR_TUNER_CONFIG_H
#define MOTOR_TUNER_CONFIG_H

/*
 * Hardware settings for the separate motor-tuning firmware.
 * TR settings currently target the RM C620-style CAN backend. AR hardware is
 * unidentified and its profile is disabled by default; do not infer its
 * interface or IDs from the profile name.
 */
#define TUNER_CAN_BITRATE_HZ          1000000U
#define TUNER_PROFILE_TR_MOTOR_COUNT  3U
/* AR interface is unverified and reported possibly non-CAN. Keep disabled. */
#define TUNER_PROFILE_AR_ENABLED     0U
#define TUNER_PROFILE_AR_MOTOR_COUNT 2U
#define TUNER_TR_MOTOR_1_CAN_ID       1U /* CAN1 */
#define TUNER_TR_MOTOR_2_CAN_ID       2U /* CAN1 */
#define TUNER_TR_MOTOR_3_CAN_ID       3U /* trailing motor: CAN2 */
#define TUNER_AR_MOTOR_1_CAN_ID       1U
#define TUNER_AR_MOTOR_2_CAN_ID       2U

/* Gear reduction is motor-shaft turns : gearbox-output turns. RM feedback
 * velocity is treated as motor-shaft RPM; the tuner converts it to output RPM
 * before comparing with its target. Verify this convention for the fitted ESC. */
#define TUNER_GEAR_RATIO_MOTOR_TO_OUTPUT 19.0f

/* User-provided software current ceiling; it is not a hardware protection. */
#define TUNER_CURRENT_LIMIT_A          5.0f
#define TUNER_SPEED_STEP_MOTOR_RPM   4000.0f /* requested motor-shaft target; about 210.5 output RPM at 19:1 */
#define TUNER_SPEED_STEP_RPM (TUNER_SPEED_STEP_MOTOR_RPM / TUNER_GEAR_RATIO_MOTOR_TO_OUTPUT)
#define TUNER_MAX_FEEDBACK_MOTOR_RPM 4370.0f /* 230 output RPM safe ceiling at 19:1 */
#define TUNER_MAX_FEEDBACK_RPM (TUNER_MAX_FEEDBACK_MOTOR_RPM / TUNER_GEAR_RATIO_MOTOR_TO_OUTPUT)
#define TUNER_MAX_MOTOR_TEMP_C         70U
#define TUNER_FEEDBACK_TIMEOUT_MS     100U
#define TUNER_SAMPLE_PERIOD_MS         20U
#define TUNER_CURRENT_PULSE_MS        250U
#define TUNER_SETTLED_SPEED_RPM        2.0f /* output shaft; must stay below this */
#define TUNER_SETTLE_STABLE_MS       250U
#define TUNER_SETTLE_TIMEOUT_MS     5000U
#define TUNER_MIN_REST_MS           1000U
#define TUNER_RESTART_TEMP_C          50U
#define TUNER_PROFILE_HOLD_MS        2000U

#endif
