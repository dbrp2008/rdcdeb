#include "motor_tuner.h"
#include "motor_tuner_config.h"
#include "rdc_can_bus.h"
#include "rm_motor.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TUNER_ZERO 0.0f
#define TUNER_CURRENT_PHASES 3U
#define TUNER_PID_PHASES 1U
#define TUNER_RISE_LIMIT_MS 500U
#define TUNER_OVERSHOOT_LIMIT_PCT 8.0f
#define TUNER_STEADY_ERROR_LIMIT_PCT 3.0f
#define TUNER_STEADY_DWELL_MS 300U
#define TUNER_RATIO_STABLE_DWELL_MS 300U
#define TUNER_RATIO_STABILITY_BAND_PCT 3.0f
#define TUNER_REQUIRED_PASS_STREAK 3U
#define TUNER_MIN_KP 0.001f
#define TUNER_MAX_KP 0.225f
#define TUNER_MAX_KI 0.045f
#define TUNER_MAX_KD 0.0045f
#define TUNER_KP_FINE_TRIALS 2U
#define TUNER_KP_FINE_STEP_MIN 0.05f
#define TUNER_KP_FINE_STEP_MID 0.10f
#define TUNER_KP_FINE_STEP_MAX 0.20f
#define TUNER_I_STAGE_MIN_RATIO 0.95f /* normalized ARPM/TRPM gate */
#define TUNER_KP_STEP_NEAR 0.10f
#define TUNER_KP_STEP_MID 0.25f
#define TUNER_KP_STEP_FAR 0.50f
#define TUNER_KI_STEP_SMALL 0.0010f
#define TUNER_KI_STEP_MEDIUM 0.0030f
#define TUNER_KI_STEP_LARGE 0.0060f
#define TUNER_KD_STEP_SMALL 0.0001f
#define TUNER_KD_STEP_MEDIUM 0.0004f
#define TUNER_KD_STEP_LARGE 0.0008f

typedef enum { TUNER_STAGE_P = 0U, TUNER_STAGE_I, TUNER_STAGE_D } TunerStage;
static TunerStage tuning_stage;
static uint8_t kp_refinement_active;
static uint8_t p_rebalance_active;
static uint8_t kp_fine_trials;
static float kp_last_passing;
static float kp_last_passing_ratio;

typedef struct { float kp; float ki; float kd; } GainCandidate;
static const float current_steps_a[TUNER_CURRENT_PHASES] = {0.25f, 0.50f, 0.75f};
static const GainCandidate initial_gain_candidates[TUNER_PID_PHASES] = {
  {0.012f, 0.000f, 0.0000f}
};
static GainCandidate gain_candidates[TUNER_PID_PHASES];
#define TUNER_RM_CHANNEL_TO_ENUM(channel) ((RM_Motor)((channel) - 1U))
#if (TUNER_TR_MOTOR_1_CAN_ID < 1U) || (TUNER_TR_MOTOR_1_CAN_ID > 8U) || \
    (TUNER_TR_MOTOR_2_CAN_ID < 1U) || (TUNER_TR_MOTOR_2_CAN_ID > 8U) || \
    (TUNER_TR_MOTOR_3_CAN_ID < 1U) || (TUNER_TR_MOTOR_3_CAN_ID > 8U) || \
    (TUNER_AR_MOTOR_1_CAN_ID < 1U) || (TUNER_AR_MOTOR_1_CAN_ID > 8U) || \
    (TUNER_AR_MOTOR_2_CAN_ID < 1U) || (TUNER_AR_MOTOR_2_CAN_ID > 8U)
#error "RM CAN channel IDs must be in the range 1..8"
#endif
#if (TUNER_TR_MOTOR_1_CAN_ID == TUNER_TR_MOTOR_2_CAN_ID) || \
    (TUNER_TR_MOTOR_1_CAN_ID == TUNER_TR_MOTOR_3_CAN_ID) || \
    (TUNER_TR_MOTOR_2_CAN_ID == TUNER_TR_MOTOR_3_CAN_ID)
#error "TR RM CAN channel IDs must be unique"
#endif
#if (TUNER_AR_MOTOR_1_CAN_ID == TUNER_AR_MOTOR_2_CAN_ID)
#error "AR RM CAN channel IDs must be unique"
#endif
static const RM_Motor tr_motor_ids[TUNER_PROFILE_TR_MOTOR_COUNT] = {
  TUNER_RM_CHANNEL_TO_ENUM(TUNER_TR_MOTOR_1_CAN_ID),
  TUNER_RM_CHANNEL_TO_ENUM(TUNER_TR_MOTOR_2_CAN_ID),
  TUNER_RM_CHANNEL_TO_ENUM(TUNER_TR_MOTOR_3_CAN_ID)
};
#if TUNER_PROFILE_AR_ENABLED
static const RM_Motor ar_motor_ids[TUNER_PROFILE_AR_MOTOR_COUNT] = {
  TUNER_RM_CHANNEL_TO_ENUM(TUNER_AR_MOTOR_1_CAN_ID),
  TUNER_RM_CHANNEL_TO_ENUM(TUNER_AR_MOTOR_2_CAN_ID)
};
#endif

typedef enum {
  TUNER_FAULT_NONE = 0U,
  TUNER_FAULT_TX,
  TUNER_FAULT_FEEDBACK,
  TUNER_FAULT_OVERSPEED,
  TUNER_FAULT_OVERTEMP,
  TUNER_FAULT_SETTLE_TIMEOUT
} TunerFault;

static TunerProfile selected_profile = TUNER_PROFILE_TR;
static uint8_t selected_motor;
static uint8_t running;
static uint8_t settling;
static uint8_t settled_tracking;
static uint8_t feedback_fresh;
static uint8_t tx_error;
static uint8_t have_csv_header;
static uint8_t results_ready;
static uint8_t best_candidate_index;
static uint8_t active_candidate_index;
static uint8_t pass_streak;
static uint32_t trial_number;
static uint8_t trial_passed;
static uint8_t rise10_seen;
static uint8_t rise90_seen;
static uint8_t accuracy_tracking;
static uint8_t rpm_stability_tracking;
static uint8_t ratio_measurement_recorded;
static float rpm_stability_start;
static float active_target_rpm;
static float actual_target_ratio;
static uint32_t rise10_ms;
static uint32_t rise90_ms;
static uint32_t accuracy_since_ms;
static uint32_t rpm_stability_since_ms;
static float overshoot_pct;
static TunerFault fault;
static uint8_t experiment_index;
static float candidate_iae[TUNER_PID_PHASES];
static float candidate_peak[TUNER_PID_PHASES];
static float candidate_actual_rpm[TUNER_PID_PHASES];
static float candidate_command_current_a[TUNER_PID_PHASES];
static float candidate_feedback_current_a[TUNER_PID_PHASES];
static uint8_t candidate_temperature_c[TUNER_PID_PHASES];
static uint32_t candidate_rise_time_ms[TUNER_PID_PHASES];
static float candidate_overshoot_pct[TUNER_PID_PHASES];
static float candidate_steady_error_pct[TUNER_PID_PHASES];
static uint32_t started_ms;
static uint32_t last_sample_ms;
static uint32_t last_control_ms;
static uint32_t phase_started_ms;
static uint32_t settle_started_ms;
static uint32_t settled_since_ms;
static uint32_t last_idle_can_ms;
static float iae;
static float peak_rpm;
static float current_a;
static float measured_current_a;
static uint8_t measured_temperature_c;
static float measured_rpm;
static float integral_term;
static float previous_measured_rpm;

static uint8_t selected_count(void)
{
#if TUNER_PROFILE_AR_ENABLED
  if (selected_profile == TUNER_PROFILE_AR) return TUNER_PROFILE_AR_MOTOR_COUNT;
#endif
  return TUNER_PROFILE_TR_MOTOR_COUNT;
}

static RM_Motor selected_motor_id(void)
{
#if TUNER_PROFILE_AR_ENABLED
  if (selected_profile == TUNER_PROFILE_AR) return ar_motor_ids[selected_motor];
#endif
  return tr_motor_ids[selected_motor];
}

static void all_outputs_zero(void)
{
  for (uint8_t i = 0U; i < (uint8_t)NUM_OF_RM_MOTORS; ++i) {
    RM_SetCurrentA((RM_Motor)i, TUNER_ZERO);
  }
  current_a = TUNER_ZERO;
}

static void log_line(const char *line)
{
  if (line == NULL) return;
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)line,
                          (uint16_t)strlen(line), 50U);
}

static void log_csv_header(void)
{
  if (have_csv_header != 0U) return;
  log_line("profile,motor,trial,phase,kp_x1000,ki_x1000,kd_x1000000,t_ms,target_output_rpm,actual_output_rpm,actual_target_ratio_x1000,command_current_mA,esc_current_mA,esc_temp_c,iae_x1000,peak_abs_output_rpm,rise_time_ms,overshoot_pct_x1000,steady_error_pct_x1000,pass_streak\r\n");
  log_line("#TRIAL,trial,candidate,kp_x1000,ki_x1000,kd_x1000000,iae_x1000,peak_rpm,rise_ms,overshoot_pct_x1000,steady_error_pct_x1000,command_mA,esc_current_mA,temp_c,passed,pass_streak\r\n");
  log_line("#RESULT,profile,motor,trials,candidate,kp_x1000,ki_x1000,kd_x1000000,iae_x1000,peak_rpm,target_output_rpm,actual_output_rpm,command_mA,esc_current_mA,temp_c,rise_ms,overshoot_pct_x1000,steady_error_pct_x1000,pass_streak\r\n");
  log_line("#ACCEPTANCE,nominal_target_motor_rpm=4000,nominal_target_output_rpm=210.5,rise_10_90_ms<=500,overshoot_pct<=8,steady_error_pct<=3,steady_dwell_ms=300,required_pass_streak=3,max_motor_rpm=4370,max_output_rpm=230,current_limit_A=5\r\n");
  log_line("#SEARCH_ORDER,P_first_with_2_adaptive_fine_refinements_then_I_then_D; I_gate_ARPM_over_TRPM>=0.95; backward_rebalance_enabled; backtrack_preserves_learned_I_D; dynamic_leaps=Kp_10-50pct,fine_Kp_5-20pct,Ki_0.001-0.006,Kd_0.0001-0.0008\r\n");
  log_line("#RATIO_CAL,ARPM_over_TRPM; stability_drift_pct<=3_for_300ms; one_measurement_per_run; diagnostic_only_no_target_scaling\r\n");
  log_line("#GAIN_BOUNDS,kp_min=0.001,kp_max=0.225,ki_min=0,ki_max=0.045,kd_min=0,kd_max=0.0045\r\n");
  have_csv_header = 1U;
}

static float current_steady_error_pct(float target_rpm)
{
  if (target_rpm <= 0.0f) return 100.0f;
  return fabsf(target_rpm - measured_rpm) * 100.0f / target_rpm;
}

static int32_t current_rise_time_ms(void)
{
  if ((rise10_seen == 0U) || (rise90_seen == 0U) || (rise90_ms < rise10_ms)) {
    return -1;
  }
  return (int32_t)(rise90_ms - rise10_ms);
}

static void log_sample(uint32_t now_ms, float target_rpm,
                       float kp, float ki, float kd)
{
  char line[224];
  const char *profile = (selected_profile == TUNER_PROFILE_AR) ? "AR" : "TR";
  const unsigned long trial = (experiment_index >= TUNER_CURRENT_PHASES)
      ? (unsigned long)(trial_number + 1U) : 0UL;
  const int written = snprintf(line, sizeof(line),
      "%s,%u,%lu,%u,%ld,%ld,%ld,%lu,%d,%d,%ld,%d,%d,%u,%ld,%d,%ld,%ld,%ld,%u\r\n",
      profile, (unsigned)(selected_motor + 1U), trial,
      (unsigned)((experiment_index < TUNER_CURRENT_PHASES)
          ? (experiment_index + 1U) : (TUNER_CURRENT_PHASES + (uint8_t)tuning_stage + 1U)),
      (long)(kp * 1000.0f), (long)(ki * 1000.0f), (long)(kd * 1000000.0f),
      (unsigned long)(now_ms - started_ms), (int)target_rpm,
      (int)measured_rpm, (long)((target_rpm > 0.0f)
          ? (fabsf(measured_rpm) / target_rpm * 1000.0f) : 0.0f),
      (int)(current_a * 1000.0f),
      (int)(measured_current_a * 1000.0f), (unsigned)measured_temperature_c,
      (long)(iae * 1000.0f), (int)peak_rpm,
      (long)current_rise_time_ms(), (long)(overshoot_pct * 1000.0f),
      (long)(current_steady_error_pct(target_rpm) * 1000.0f),
      (unsigned)pass_streak);
  if ((written > 0) && ((size_t)written < sizeof(line))) log_line(line);
}

static void log_trial_summary(uint8_t passed)
{
  char line[224];
  const GainCandidate *candidate = &gain_candidates[active_candidate_index];
  const int written = snprintf(line, sizeof(line),
      "TRIAL,%lu,%u,%ld,%ld,%ld,%ld,%d,%ld,%ld,%ld,%d,%d,%u,%u,%u\r\n",
      (unsigned long)trial_number, (unsigned)(active_candidate_index + 1U),
      (long)(candidate->kp * 1000.0f), (long)(candidate->ki * 1000.0f),
      (long)(candidate->kd * 1000000.0f), (long)(iae * 1000.0f),
      (int)peak_rpm, (long)current_rise_time_ms(),
      (long)(overshoot_pct * 1000.0f),
      (long)(current_steady_error_pct(active_target_rpm) * 1000.0f),
      (int)(current_a * 1000.0f), (int)(measured_current_a * 1000.0f),
      (unsigned)measured_temperature_c, (unsigned)passed,
      (unsigned)pass_streak);
  if ((written > 0) && ((size_t)written < sizeof(line))) log_line(line);
}

static void log_result_summary(void)
{
  char line[240];
  const GainCandidate *best = &gain_candidates[best_candidate_index];
  const char *profile = (selected_profile == TUNER_PROFILE_AR) ? "AR" : "TR";
  const int written = snprintf(line, sizeof(line),
      "RESULT,%s,%u,%lu,%u,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%u,%lu,%ld,%ld,%u\r\n",
      profile, (unsigned)(selected_motor + 1U), (unsigned long)trial_number,
      (unsigned)(best_candidate_index + 1U),
      (long)(best->kp * 1000.0f), (long)(best->ki * 1000.0f),
      (long)(best->kd * 1000000.0f), (long)(candidate_iae[best_candidate_index] * 1000.0f),
      (int)candidate_peak[best_candidate_index], (int)active_target_rpm,
      (int)candidate_actual_rpm[best_candidate_index],
      (int)(candidate_command_current_a[best_candidate_index] * 1000.0f),
      (int)(candidate_feedback_current_a[best_candidate_index] * 1000.0f),
      (unsigned)candidate_temperature_c[best_candidate_index],
      (unsigned long)candidate_rise_time_ms[best_candidate_index],
      (long)(candidate_overshoot_pct[best_candidate_index] * 1000.0f),
      (long)(candidate_steady_error_pct[best_candidate_index] * 1000.0f),
      (unsigned)pass_streak);
  if ((written > 0) && ((size_t)written < sizeof(line))) log_line(line);
}

static void finish_experiment(void)
{
  all_outputs_zero();
  running = 0U;
  settling = 0U;
  if (RM_CAN_ServiceMotor(selected_motor_id()) != HAL_OK) {
    tx_error = 1U;
    fault = TUNER_FAULT_TX;
    return;
  }
  tx_error = 0U;
  best_candidate_index = active_candidate_index;
  results_ready = 1U;
  log_result_summary();
}

static float kp_up_step(float response_ratio)
{
  const float ratio_gap = TUNER_I_STAGE_MIN_RATIO - response_ratio;
  if (ratio_gap >= 0.15f) return TUNER_KP_STEP_FAR;
  if (ratio_gap >= 0.05f) return TUNER_KP_STEP_MID;
  return TUNER_KP_STEP_NEAR;
}

static float kp_fine_step(float response_ratio)
{
  const float ratio_margin = response_ratio - TUNER_I_STAGE_MIN_RATIO;
  if (ratio_margin >= 0.08f) return TUNER_KP_FINE_STEP_MAX;
  if (ratio_margin >= 0.03f) return TUNER_KP_FINE_STEP_MID;
  return TUNER_KP_FINE_STEP_MIN;
}

static float kp_backoff_step(void)
{
  if (overshoot_pct >= (TUNER_OVERSHOOT_LIMIT_PCT + 15.0f)) return 0.35f;
  if (overshoot_pct >= (TUNER_OVERSHOOT_LIMIT_PCT + 7.0f)) return 0.25f;
  return 0.20f;
}

static float ki_step_for_error(void)
{
  const float error_pct = current_steady_error_pct(active_target_rpm);
  if (error_pct >= 15.0f) return TUNER_KI_STEP_LARGE;
  if (error_pct >= 8.0f) return TUNER_KI_STEP_MEDIUM;
  return TUNER_KI_STEP_SMALL;
}

static float kd_step_for_overshoot(void)
{
  const float excess = overshoot_pct - TUNER_OVERSHOOT_LIMIT_PCT;
  if (excess >= 15.0f) return TUNER_KD_STEP_LARGE;
  if (excess >= 7.0f) return TUNER_KD_STEP_MEDIUM;
  return TUNER_KD_STEP_SMALL;
}

static void adapt_candidate(uint8_t candidate_index, int32_t rise_ms,
                            uint8_t steady_ok)
{
  GainCandidate *candidate = &gain_candidates[candidate_index];
  const uint8_t rise_slow = (rise_ms < 0) ||
      ((uint32_t)rise_ms > TUNER_RISE_LIMIT_MS);
  const uint8_t overshoot_high = (overshoot_pct > TUNER_OVERSHOOT_LIMIT_PCT);

  const float response_ratio = fabsf(measured_rpm) / active_target_rpm;
  const uint8_t ratio_allows_i = (response_ratio >= TUNER_I_STAGE_MIN_RATIO);
  const float fine_step = kp_fine_step(response_ratio);

  /* Enforce the ARPM threshold on every path into I, including backward
   * re-entry from D. Below threshold, return to P and do not adjust Ki. */
  if ((tuning_stage == TUNER_STAGE_I) && (ratio_allows_i == 0U)) {
    candidate->ki = 0.0f;
    candidate->kd = 0.0f;
    candidate->kp = fminf(TUNER_MAX_KP, candidate->kp * (1.0f + kp_up_step(response_ratio)));
    tuning_stage = TUNER_STAGE_P;
    kp_refinement_active = 0U;
    p_rebalance_active = 0U;
    return;
  }

  if (tuning_stage == TUNER_STAGE_P) {
    /* Fully tune P first. Once dynamics pass, only lower/refine P while actual
     * RPM remains above the normalized I-entry ratio of 0.95. */
    if (p_rebalance_active == 0U) {
      candidate->ki = 0.0f;
      candidate->kd = 0.0f;
    }
    if (kp_refinement_active != 0U) {
      if ((overshoot_high == 0U) && (rise_slow == 0U) && (ratio_allows_i != 0U)) {
        kp_last_passing = candidate->kp;
        kp_last_passing_ratio = response_ratio;
        ++kp_fine_trials;
        if ((kp_fine_trials >= TUNER_KP_FINE_TRIALS) ||
            (candidate->kp <= (TUNER_MIN_KP * (1.0f + fine_step)))) {
          candidate->kp = kp_last_passing;
          tuning_stage = TUNER_STAGE_I;
          if (candidate->ki <= 0.0f) candidate->ki = ki_step_for_error();
          p_rebalance_active = 0U;
          kp_refinement_active = 0U;
        } else {
          candidate->kp = fmaxf(TUNER_MIN_KP,
              candidate->kp * (1.0f - fine_step));
        }
      } else if ((rise_slow != 0U) || (overshoot_high != 0U)) {
        candidate->kp = kp_last_passing;
        kp_refinement_active = 0U;
        if (kp_last_passing_ratio >= TUNER_I_STAGE_MIN_RATIO) {
          tuning_stage = TUNER_STAGE_I;
          if (candidate->ki <= 0.0f) candidate->ki = ki_step_for_error();
          p_rebalance_active = 0U;
        } else {
          candidate->kp = fminf(TUNER_MAX_KP,
              kp_last_passing * (1.0f + kp_up_step(response_ratio)));
        }
      } else {
        /* The current fine Kp is dynamically acceptable but actual RPM is
         * below the 0.95 ratio gate: restore the passing value and raise P; do not enter I. */
        candidate->kp = fminf(TUNER_MAX_KP,
            kp_last_passing * (1.0f + kp_up_step(response_ratio)));
        kp_refinement_active = 0U;
      }
    } else if (overshoot_high) {
      candidate->kp = fmaxf(TUNER_MIN_KP, candidate->kp * (1.0f - kp_backoff_step()));
    } else if (rise_slow) {
      candidate->kp = fminf(TUNER_MAX_KP,
          candidate->kp * (1.0f + kp_up_step(response_ratio)));
      if ((candidate->kp >= TUNER_MAX_KP) && (ratio_allows_i != 0U)) {
        kp_last_passing = candidate->kp;
        kp_last_passing_ratio = response_ratio;
        tuning_stage = TUNER_STAGE_I;
        if (candidate->ki <= 0.0f) candidate->ki = ki_step_for_error();
        p_rebalance_active = 0U;
      }
    } else if (ratio_allows_i == 0U) {
      /* P dynamics are acceptable, but do not move to I until ARPM/TRPM reaches 0.95; keep pushing P upward to close the remaining tracking gap. */
      candidate->kp = fminf(TUNER_MAX_KP,
          candidate->kp * (1.0f + kp_up_step(response_ratio)));
    } else {
      kp_last_passing = candidate->kp;
      kp_last_passing_ratio = response_ratio;
      kp_fine_trials = 0U;
      kp_refinement_active = 1U;
      candidate->kp = fmaxf(TUNER_MIN_KP,
          candidate->kp * (1.0f - fine_step));
      if (candidate->kp >= kp_last_passing) {
        tuning_stage = TUNER_STAGE_I;
        if (candidate->ki <= 0.0f) candidate->ki = ki_step_for_error();
        p_rebalance_active = 0U;
        kp_refinement_active = 0U;
      }
    }
  } else if (tuning_stage == TUNER_STAGE_I) {
    /* Use a larger bounded Ki jump to avoid many tiny integral-only trials.
     * If this causes overshoot, backtrack both Ki and P instead of persisting. */
    if (overshoot_high) {
      candidate->ki *= 0.50f;
      candidate->kp = fmaxf(TUNER_MIN_KP, candidate->kp * (1.0f - kp_backoff_step()));
      tuning_stage = TUNER_STAGE_P;
      kp_refinement_active = 0U;
      p_rebalance_active = 1U;
    } else if (steady_ok == 0U) {
      if (candidate->ki < TUNER_MAX_KI) {
        candidate->ki = fminf(TUNER_MAX_KI, candidate->ki + ki_step_for_error());
      } else {
        candidate->kp = fminf(TUNER_MAX_KP, candidate->kp * (1.0f + kp_up_step(response_ratio)));
        candidate->ki = 0.0f;
        tuning_stage = TUNER_STAGE_P;
        kp_refinement_active = 0U;
        p_rebalance_active = 0U;
      }
    } else if (rise_slow) {
      /* I met accuracy but damaged the transient: lower I, revisit P. */
      candidate->ki *= 0.50f;
      candidate->kp = fminf(TUNER_MAX_KP, candidate->kp * (1.0f + kp_up_step(response_ratio)));
      tuning_stage = TUNER_STAGE_P;
      kp_refinement_active = 0U;
      p_rebalance_active = 1U;
    } else {
      tuning_stage = TUNER_STAGE_D;
    }
  } else {
    /* Add D only for damping. If D harms rise or I accuracy, flow backward
     * and revisit P or I; every failed trial changes the next gain vector. */
    if (overshoot_high) {
      if (candidate->kd < TUNER_MAX_KD) {
        candidate->kd = fminf(TUNER_MAX_KD, candidate->kd + kd_step_for_overshoot());
      } else {
        candidate->ki *= 0.80f;
        candidate->kp = fmaxf(TUNER_MIN_KP, candidate->kp * (1.0f - kp_backoff_step()));
        candidate->kd = 0.0f;
        if (ratio_allows_i != 0U) {
          tuning_stage = TUNER_STAGE_I;
        } else {
          candidate->ki = 0.0f;
          candidate->kp = fminf(TUNER_MAX_KP,
              candidate->kp * (1.0f + kp_up_step(response_ratio)));
          tuning_stage = TUNER_STAGE_P;
          kp_refinement_active = 0U;
          p_rebalance_active = 0U;
        }
      }
    } else if (rise_slow) {
      candidate->kd *= 0.50f;
      candidate->kp = fminf(TUNER_MAX_KP, candidate->kp * (1.0f + kp_up_step(response_ratio)));
      tuning_stage = TUNER_STAGE_P;
      kp_refinement_active = 0U;
      p_rebalance_active = 1U;
    } else if (steady_ok == 0U) {
      candidate->kd = 0.0f;
      if (ratio_allows_i != 0U) {
        tuning_stage = TUNER_STAGE_I;
      } else {
        candidate->ki = 0.0f;
        candidate->kp = fminf(TUNER_MAX_KP,
            candidate->kp * (1.0f + kp_up_step(response_ratio)));
        tuning_stage = TUNER_STAGE_P;
        kp_refinement_active = 0U;
        p_rebalance_active = 0U;
      }
    }
  }
}

static void begin_phase(uint32_t now_ms)
{
  settling = 0U;
  settled_tracking = 0U;
  phase_started_ms = now_ms;
  last_sample_ms = now_ms;
  last_control_ms = now_ms;
  iae = 0.0f;
  peak_rpm = fabsf(measured_rpm);
  integral_term = 0.0f;
  previous_measured_rpm = measured_rpm;
  rise10_seen = 0U;
  rise90_seen = 0U;
  accuracy_tracking = 0U;
  rise10_ms = 0U;
  rise90_ms = 0U;
  accuracy_since_ms = 0U;
  rpm_stability_tracking = 0U;
  rpm_stability_since_ms = now_ms;
  rpm_stability_start = measured_rpm;
  overshoot_pct = 0.0f;
}

static void enter_settle(uint32_t now_ms)
{
  all_outputs_zero();
  settling = 1U;
  settled_tracking = 0U;
  settle_started_ms = now_ms;
  if (RM_CAN_ServiceMotor(selected_motor_id()) != HAL_OK) {
    tx_error = 1U;
    fault = TUNER_FAULT_TX;
    running = 0U;
    settling = 0U;
  } else {
    tx_error = 0U;
  }
}

static void stop_with_fault(TunerFault reason)
{
  fault = reason;
  running = 0U;
  settling = 0U;
  all_outputs_zero();
  if (RM_CAN_ServiceMotor(selected_motor_id()) != HAL_OK) {
    tx_error = 1U;
    fault = TUNER_FAULT_TX;
  } else {
    tx_error = 0U;
  }
}

void MotorTuner_Init(void)
{
  selected_profile = TUNER_PROFILE_TR;
  selected_motor = 0U;
  active_target_rpm = TUNER_SPEED_STEP_RPM;
  actual_target_ratio = 1.0f;
  ratio_measurement_recorded = 0U;
  running = 0U;
  settling = 0U;
  settled_tracking = 0U;
  feedback_fresh = 0U;
  tx_error = 0U;
  fault = TUNER_FAULT_NONE;
  experiment_index = 0U;
  results_ready = 0U;
  best_candidate_index = 0U;
  active_candidate_index = 0U;
  trial_number = 0U;
  pass_streak = 0U;
  (void)memcpy(gain_candidates, initial_gain_candidates, sizeof(gain_candidates));
  tuning_stage = TUNER_STAGE_P;
  kp_refinement_active = 0U;
  p_rebalance_active = 0U;
  kp_fine_trials = 0U;
  kp_last_passing = gain_candidates[0].kp;
  kp_last_passing_ratio = 0.0f;
  measured_current_a = TUNER_ZERO;
  all_outputs_zero();
  last_idle_can_ms = HAL_GetTick();
  log_csv_header();
}

void MotorTuner_SelectProfile(TunerProfile profile)
{
  if (running != 0U) return;
#if TUNER_PROFILE_AR_ENABLED
  selected_profile = (profile == TUNER_PROFILE_AR) ? TUNER_PROFILE_AR : TUNER_PROFILE_TR;
#else
  (void)profile;
  selected_profile = TUNER_PROFILE_TR;
#endif
  selected_motor = 0U;
}

void MotorTuner_SelectNextMotor(void)
{
  if (running != 0U) return;
  selected_motor = (uint8_t)((selected_motor + 1U) % selected_count());
}

TunerProfile MotorTuner_GetProfile(void)
{
  return selected_profile;
}

uint8_t MotorTuner_GetMotorCount(void)
{
  return selected_count();
}

void MotorTuner_Start(void)
{
  if (running != 0U || fault != TUNER_FAULT_NONE) return;
#if !TUNER_PROFILE_AR_ENABLED
  if (selected_profile == TUNER_PROFILE_AR) return;
#endif
  all_outputs_zero();
  experiment_index = 0U;
  results_ready = 0U;
  best_candidate_index = 0U;
  active_candidate_index = 0U;
  trial_number = 0U;
  pass_streak = 0U;
  trial_passed = 0U;
  active_target_rpm = TUNER_SPEED_STEP_RPM;
  actual_target_ratio = 1.0f;
  ratio_measurement_recorded = 0U;
  (void)memcpy(gain_candidates, initial_gain_candidates, sizeof(gain_candidates));
  tuning_stage = TUNER_STAGE_P;
  kp_refinement_active = 0U;
  p_rebalance_active = 0U;
  kp_fine_trials = 0U;
  kp_last_passing = gain_candidates[0].kp;
  kp_last_passing_ratio = 0.0f;
  (void)memset(candidate_iae, 0, sizeof(candidate_iae));
  (void)memset(candidate_peak, 0, sizeof(candidate_peak));
  (void)memset(candidate_actual_rpm, 0, sizeof(candidate_actual_rpm));
  (void)memset(candidate_command_current_a, 0, sizeof(candidate_command_current_a));
  (void)memset(candidate_feedback_current_a, 0, sizeof(candidate_feedback_current_a));
  (void)memset(candidate_temperature_c, 0, sizeof(candidate_temperature_c));
  (void)memset(candidate_rise_time_ms, 0, sizeof(candidate_rise_time_ms));
  (void)memset(candidate_overshoot_pct, 0, sizeof(candidate_overshoot_pct));
  (void)memset(candidate_steady_error_pct, 0, sizeof(candidate_steady_error_pct));
  iae = 0.0f;
  peak_rpm = 0.0f;
  integral_term = 0.0f;
  started_ms = HAL_GetTick();
  last_sample_ms = started_ms;
  last_control_ms = started_ms;
  running = 1U;
  enter_settle(started_ms);
}

void MotorTuner_Abort(void)
{
  running = 0U;
  settling = 0U;
  all_outputs_zero();
  if (RM_CAN_ServiceMotor(selected_motor_id()) != HAL_OK) {
    tx_error = 1U;
    fault = TUNER_FAULT_TX;
  } else {
    tx_error = 0U;
  }
}

void MotorTuner_Tick(uint32_t now_ms)
{
  if (running == 0U) {
    all_outputs_zero();
    if ((uint32_t)(now_ms - last_idle_can_ms) >= TUNER_SAMPLE_PERIOD_MS) {
      /* No-motor bench mode must keep UI controls alive if CAN has no ACK. */
      tx_error = (RM_CAN_ServiceAll() == HAL_OK) ? 0U : 1U;
      last_idle_can_ms = now_ms;
    }
    return;
  }

  if (RM_CAN_ServiceMotor(selected_motor_id()) != HAL_OK) {
    tx_error = 1U;
    feedback_fresh = 0U;
    stop_with_fault(TUNER_FAULT_TX);
    return;
  }
  tx_error = 0U;
  const volatile RM_Feedback *feedback = RM_GetFeedback(selected_motor_id());
  if (feedback == NULL || feedback->received == 0U ||
      (uint32_t)(now_ms - feedback->last_rx_tick) > TUNER_FEEDBACK_TIMEOUT_MS) {
    feedback_fresh = 0U;
    stop_with_fault(TUNER_FAULT_FEEDBACK);
    return;
  }
  feedback_fresh = 1U;
  measured_current_a = feedback->current_a;
  measured_temperature_c = feedback->temperature_c;
  measured_rpm = (float)feedback->velocity_rpm / TUNER_GEAR_RATIO_MOTOR_TO_OUTPUT;
  if (fabsf(measured_rpm) > TUNER_MAX_FEEDBACK_RPM) {
    stop_with_fault(TUNER_FAULT_OVERSPEED);
    return;
  }
  if (feedback->temperature_c >= TUNER_MAX_MOTOR_TEMP_C) {
    stop_with_fault(TUNER_FAULT_OVERTEMP);
    return;
  }

  if (settling != 0U) {
    all_outputs_zero();
    if ((uint32_t)(now_ms - settle_started_ms) >= TUNER_SETTLE_TIMEOUT_MS) {
      stop_with_fault(TUNER_FAULT_SETTLE_TIMEOUT);
      return;
    }
    if (fabsf(measured_rpm) <= TUNER_SETTLED_SPEED_RPM) {
      if (settled_tracking == 0U) {
        settled_tracking = 1U;
        settled_since_ms = now_ms;
      } else if (((uint32_t)(now_ms - settled_since_ms) >= TUNER_SETTLE_STABLE_MS) &&
                 ((uint32_t)(now_ms - settle_started_ms) >= TUNER_MIN_REST_MS) &&
                 (measured_temperature_c < TUNER_RESTART_TEMP_C)) {
        begin_phase(now_ms);
      }
    } else {
      settled_tracking = 0U;
    }
    return;
  }

  uint32_t phase_elapsed = now_ms - phase_started_ms;
  float target_rpm = TUNER_ZERO;
  float kp = TUNER_ZERO;
  float ki = TUNER_ZERO;
  float kd = TUNER_ZERO;
  float derivative_term = TUNER_ZERO;
  if (experiment_index < TUNER_CURRENT_PHASES) {
    current_a = current_steps_a[experiment_index];
    RM_SetCurrentA(selected_motor_id(), current_a);
  } else {
    const uint8_t pi_trial = (uint8_t)(experiment_index - TUNER_CURRENT_PHASES);
    if (pi_trial >= TUNER_PID_PHASES) {
      MotorTuner_Abort();
      return;
    }
    target_rpm = active_target_rpm;
    kp = gain_candidates[pi_trial].kp;
    ki = gain_candidates[pi_trial].ki;
    kd = gain_candidates[pi_trial].kd;
    const float error = target_rpm - measured_rpm;
    const uint32_t dt_ms = now_ms - last_control_ms;
    const float dt = (float)dt_ms * 0.001f;
    last_control_ms = now_ms;
    iae += fabsf(error) * dt;
    const float proposed_integral = integral_term + ki * error * dt;
    if (dt > 0.0f) {
      derivative_term = -kd * (measured_rpm - previous_measured_rpm) / dt;
    }
    previous_measured_rpm = measured_rpm;
    float requested = kp * error + proposed_integral + derivative_term;
    if (requested > TUNER_CURRENT_LIMIT_A) requested = TUNER_CURRENT_LIMIT_A;
    if (requested < -TUNER_CURRENT_LIMIT_A) requested = -TUNER_CURRENT_LIMIT_A;
    if ((fabsf(kp * error + proposed_integral + derivative_term) <= TUNER_CURRENT_LIMIT_A) ||
        ((requested >= TUNER_CURRENT_LIMIT_A) && (error < 0.0f)) ||
        ((requested <= -TUNER_CURRENT_LIMIT_A) && (error > 0.0f))) {
      integral_term = proposed_integral;
    }
    current_a = kp * error + integral_term + derivative_term;
    if (current_a > TUNER_CURRENT_LIMIT_A) current_a = TUNER_CURRENT_LIMIT_A;
    if (current_a < -TUNER_CURRENT_LIMIT_A) current_a = -TUNER_CURRENT_LIMIT_A;
    RM_SetCurrentA(selected_motor_id(), current_a);
  }

  if (fabsf(measured_rpm) > peak_rpm) peak_rpm = fabsf(measured_rpm);
  if (target_rpm > 0.0f) {
    const uint32_t phase_time_ms = now_ms - phase_started_ms;
    if ((rise10_seen == 0U) && (measured_rpm >= 0.10f * target_rpm)) {
      rise10_seen = 1U;
      rise10_ms = phase_time_ms;
    }
    if ((rise90_seen == 0U) && (measured_rpm >= 0.90f * target_rpm)) {
      rise90_seen = 1U;
      rise90_ms = phase_time_ms;
    }
    overshoot_pct = (peak_rpm > target_rpm)
        ? ((peak_rpm - target_rpm) * 100.0f / target_rpm) : 0.0f;
    /* Record the actual/target RPM ratio once the response has low drift.
     * This is diagnostic telemetry only: the requested target is not multiplied
     * by the ratio or its inverse. */
    if (ratio_measurement_recorded == 0U) {
      if (rpm_stability_tracking == 0U) {
        rpm_stability_tracking = 1U;
        rpm_stability_start = measured_rpm;
        rpm_stability_since_ms = now_ms;
      } else if ((uint32_t)(now_ms - rpm_stability_since_ms) >= TUNER_RATIO_STABLE_DWELL_MS) {
        const float stability_band = fmaxf(1.0f,
            target_rpm * TUNER_RATIO_STABILITY_BAND_PCT * 0.01f);
        if (fabsf(measured_rpm - rpm_stability_start) <= stability_band) {
          const float actual_rpm = fabsf(measured_rpm);
          if (actual_rpm >= (0.10f * target_rpm)) {
            char ratio_line[128];
            actual_target_ratio = actual_rpm / target_rpm;
            ratio_measurement_recorded = 1U;
            const int ratio_written = snprintf(ratio_line, sizeof(ratio_line),
                "#RATIO,%lu,ARPM_x1000=%ld,TRPM_x1000=%ld,ratio_x100000=%ld\r\n",
                (unsigned long)(trial_number + 1U), (long)(actual_rpm * 1000.0f),
                (long)(target_rpm * 1000.0f),
                (long)(actual_target_ratio * 100000.0f));
            if ((ratio_written > 0) && ((size_t)ratio_written < sizeof(ratio_line))) {
              log_line(ratio_line);
            }
          }
        } else {
          rpm_stability_start = measured_rpm;
          rpm_stability_since_ms = now_ms;
        }
      }
    }
    if (current_steady_error_pct(target_rpm) <= TUNER_STEADY_ERROR_LIMIT_PCT) {
      if (accuracy_tracking == 0U) {
        accuracy_tracking = 1U;
        accuracy_since_ms = now_ms;
      }
    } else {
      accuracy_tracking = 0U;
    }
  }
  if ((uint32_t)(now_ms - last_sample_ms) >= TUNER_SAMPLE_PERIOD_MS) {
    log_sample(now_ms, target_rpm, kp, ki, kd);
    last_sample_ms = now_ms;
  }

  const uint32_t dwell = (experiment_index < TUNER_CURRENT_PHASES)
      ? TUNER_CURRENT_PULSE_MS : TUNER_PROFILE_HOLD_MS;
  if (phase_elapsed >= dwell) {
    if (experiment_index < TUNER_CURRENT_PHASES) {
      RM_SetCurrentA(selected_motor_id(), TUNER_ZERO);
      current_a = TUNER_ZERO;
      ++experiment_index;
      enter_settle(now_ms);
      return;
    }

    const uint8_t candidate = active_candidate_index;
    const int32_t rise_ms = current_rise_time_ms();
    const float steady_error_pct = current_steady_error_pct(active_target_rpm);
    const uint8_t steady_ok = (accuracy_tracking != 0U) &&
        ((uint32_t)(now_ms - accuracy_since_ms) >= TUNER_STEADY_DWELL_MS) &&
        (steady_error_pct <= TUNER_STEADY_ERROR_LIMIT_PCT);
    trial_passed = (rise_ms >= 0) &&
        ((uint32_t)rise_ms <= TUNER_RISE_LIMIT_MS) &&
        (overshoot_pct <= TUNER_OVERSHOOT_LIMIT_PCT) && steady_ok;
    candidate_iae[candidate] = iae;
    candidate_peak[candidate] = peak_rpm;
    candidate_actual_rpm[candidate] = measured_rpm;
    candidate_command_current_a[candidate] = current_a;
    candidate_feedback_current_a[candidate] = measured_current_a;
    candidate_temperature_c[candidate] = measured_temperature_c;
    candidate_rise_time_ms[candidate] = (rise_ms >= 0) ? (uint32_t)rise_ms : UINT32_MAX;
    candidate_overshoot_pct[candidate] = overshoot_pct;
    candidate_steady_error_pct[candidate] = steady_error_pct;
    ++trial_number;
    if (trial_passed != 0U) {
      ++pass_streak;
    } else {
      pass_streak = 0U;
    }
    log_trial_summary(trial_passed);

    if (pass_streak >= TUNER_REQUIRED_PASS_STREAK) {
      finish_experiment();
      return;
    }
    if (trial_passed == 0U) {
      adapt_candidate(candidate, rise_ms, steady_ok);
      active_candidate_index = 0U;
    }
    experiment_index = (uint8_t)(TUNER_CURRENT_PHASES + active_candidate_index);
    enter_settle(now_ms);
  }
}

bool MotorTuner_GetStatus(TunerStatus *status)
{
  if (status == NULL) return false;
  status->profile = selected_profile;
  status->motor_index = selected_motor;
  status->running = running;
  status->settling = settling;
  status->feedback_fresh = feedback_fresh;
  status->safety_fault = (uint8_t)fault;
  status->can_tx_fault = (tx_error != 0U || fault == TUNER_FAULT_TX) ? 1U : 0U;
  status->results_ready = results_ready;
  status->experiment_index = experiment_index;
  status->pass_streak = pass_streak;
  status->trial_number = trial_number +
      ((running != 0U && experiment_index >= TUNER_CURRENT_PHASES) ? 1U : 0U);
  status->elapsed_ms = running ? (HAL_GetTick() - started_ms) : 0U;
  if (results_ready != 0U) {
    status->target_rpm = active_target_rpm;
    status->measured_rpm = candidate_actual_rpm[best_candidate_index];
    status->current_a = candidate_command_current_a[best_candidate_index];
    status->measured_current_a = candidate_feedback_current_a[best_candidate_index];
    status->motor_temp_c = candidate_temperature_c[best_candidate_index];
  } else {
    status->target_rpm = (!settling && experiment_index >= TUNER_CURRENT_PHASES)
        ? active_target_rpm : TUNER_ZERO;
    status->measured_rpm = measured_rpm;
    status->current_a = current_a;
    status->measured_current_a = measured_current_a;
    status->motor_temp_c = measured_temperature_c;
  }
  if (results_ready != 0U) {
    status->kp = gain_candidates[best_candidate_index].kp;
    status->ki = gain_candidates[best_candidate_index].ki;
    status->kd = gain_candidates[best_candidate_index].kd;
  } else if (experiment_index >= TUNER_CURRENT_PHASES) {
    status->kp = gain_candidates[active_candidate_index].kp;
    status->ki = gain_candidates[active_candidate_index].ki;
    status->kd = gain_candidates[active_candidate_index].kd;
  } else {
    status->kp = TUNER_ZERO;
    status->ki = TUNER_ZERO;
    status->kd = TUNER_ZERO;
  }
  status->iae_rpm_seconds = (results_ready != 0U)
      ? candidate_iae[best_candidate_index] : iae;
  status->peak_rpm = (results_ready != 0U)
      ? candidate_peak[best_candidate_index] : peak_rpm;
  return true;
}
