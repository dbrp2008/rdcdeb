#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "lcd/lcd.h"
#include "lcd/lcd_graphics.h"
#include "can.h"
#include "rm_motor.h"
#include "motor_tuner.h"
#include "motor_tuner_config.h"
#include <stdio.h>
#include <string.h>

#define BUTTON_DEBOUNCE_MS 30U
#define PROFILE_HOLD_MS 900U
#define SCREEN_PERIOD_MS 100U

/* Startup operator guide and safety notice are emitted on USART2. */
static const char operator_guide[] =
  "\r\n=== RM MOTOR PID TUNER: OPERATOR GUIDE ===\r\n"
  "B1 hold >=0.9s: select motor. AR profile disabled pending hardware\r\n"
  "protocol confirmation. B2 press: start; press again to abort.\r\n"
  "CSV samples are output on USART2.\r\n"
  "TFT FAULT locks out new runs until reset; inspect the cause first.\r\n"
  "TX:ERR or stale feedback: stop and use physical disconnect as needed.\r\n"
  "TR map: IDs 1/2 on CAN1; trailing motor ID 3 on CAN2 only.\r\n"
  "Before power: verify TR profile, motor index, CAN ID and wiring.\r\n"
  "Gear reduction is 19:1; speed target/status are gearbox-output RPM.\r\n"
  "SAFETY: software current ceiling is 5.0 A; not hardware protection.\r\n"
  "Secure chassis, wheels raised clear, people/tools kept away. Use a\r\n"
  "current-limited supply and physical power disconnect within reach.\r\n"
  "Never rely on software abort as an emergency stop. Stop for unexpected\r\n"
  "motion, heat, noise, smoke, wiring faults or stale feedback; disconnect\r\n"
  "power before changing wiring. Software limits are not safety certification.\r\n"
  "AR profile is disabled until its motor/controller interface and limits\r\n"
  "are verified; it currently uses no AR-specific control backend.\r\n"
  "CAN RPM is converted by 19:1. Verify feedback is motor-side RPM on your ESC.\r\n"
  "CSV speed fields are gearbox-output RPM; current ceiling is 5.0 A.\r\n"
  "Trials adapt until 3 strict passes; B2 aborts; save the CSV.\r\n"
  "==========================================\r\n";

static uint32_t next_screen_ms;
static uint32_t btn1_down_ms;
static uint8_t btn1_prev, btn2_prev, btn1_long_action;

void SystemClock_Config(void);

static uint8_t button_pressed(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void draw_status(void)
{
  TunerStatus status;
  char profile[4];
  if (!MotorTuner_GetStatus(&status)) return;
  (void)strncpy(profile, status.profile == TUNER_PROFILE_AR ? "AR" : "TR", sizeof(profile));
  profile[sizeof(profile) - 1U] = '\0';

  const char *state = status.safety_fault ? "FAULT"
      : (status.results_ready ? "DONE"
         : (status.running ? (status.settling ? "WAIT" : "RUN") : "READY"));
  const unsigned current_step = (status.experiment_index < 3U)
      ? (unsigned)(status.experiment_index + 1U) : 3U;
  const int command_deci_a = (int)(status.current_a * 10.0f);
  const int feedback_deci_a = (int)(status.measured_current_a * 10.0f);
  const char *command_sign = (command_deci_a < 0) ? "-" : "+";
  const char *feedback_sign = (feedback_deci_a < 0) ? "-" : "+";
  const int command_abs_deci_a = (command_deci_a < 0) ? -command_deci_a : command_deci_a;
  const int feedback_abs_deci_a = (feedback_deci_a < 0) ? -feedback_deci_a : feedback_deci_a;

  /* ST7735 portrait text area is 16 columns by 10 rows (y = 0..9). */
  tft_prints(0, 0, "%s M%d/%d RM TUNER", profile,
             (int)(status.motor_index + 1U),
             (int)MotorTuner_GetMotorCount());
  tft_prints(0, 1, "%s F:%s TX%s", state,
             status.feedback_fresh ? "OK" : "--",
             status.can_tx_fault ? "ERR" : "OK");
  if (status.trial_number == 0U) {
    tft_prints(0, 2, "CUR:%d/3", (int)current_step);
  } else {
    tft_prints(0, 2, "TRY:%d PASS:%d", (int)status.trial_number,
               (int)status.pass_streak);
  }
  tft_prints(0, 3, "OUT T:%d A:%d", (int)status.target_rpm,
             (int)status.measured_rpm);
  tft_prints(0, 4, "K:%d I:%d D:%d", (int)(status.kp * 1000.0f),
             (int)(status.ki * 1000.0f), (int)(status.kd * 1000000.0f));
  tft_prints(0, 5, "C:%s%d.%d E:%s%d.%dA",
             command_sign, command_abs_deci_a / 10, command_abs_deci_a % 10,
             feedback_sign, feedback_abs_deci_a / 10, feedback_abs_deci_a % 10);
  tft_prints(0, 6, "IAE:%d P:%d",
             (int)(status.iae_rpm_seconds * 100.0f), (int)status.peak_rpm);
  tft_prints(0, 7, "ESC TEMP:%dC", (int)status.motor_temp_c);
  tft_prints(0, 8, (status.motor_index == 2U) ? "M3 TRAIL CAN2" : "M1/M2 CAN1");
  tft_prints(0, 9, "B1:SEL B2:GO/ABT");
  tft_update(0U);
}

static void service_buttons(uint32_t now)
{
  const uint8_t b1 = button_pressed(BTN1_GPIO_Port, BTN1_Pin);
  const uint8_t b2 = button_pressed(BTN2_GPIO_Port, BTN2_Pin);

  if (b1 && !btn1_prev) {
    btn1_down_ms = now;
    btn1_long_action = 0U;
  }
  if (b1 && !btn1_long_action && (uint32_t)(now - btn1_down_ms) >= PROFILE_HOLD_MS) {
    MotorTuner_SelectNextMotor();
    btn1_long_action = 1U;
  }
  if (!b1 && btn1_prev && !btn1_long_action) {
#if TUNER_PROFILE_AR_ENABLED
    HAL_Delay(BUTTON_DEBOUNCE_MS);
    if (MotorTuner_GetProfile() == TUNER_PROFILE_TR) {
      MotorTuner_SelectProfile(TUNER_PROFILE_AR);
    } else {
      MotorTuner_SelectProfile(TUNER_PROFILE_TR);
    }
#endif
  }
  if (b2 && !btn2_prev) {
    HAL_Delay(BUTTON_DEBOUNCE_MS);
    if (button_pressed(BTN2_GPIO_Port, BTN2_Pin)) {
      TunerStatus status;
      (void)MotorTuner_GetStatus(&status);
      if (status.running) MotorTuner_Abort();
      else MotorTuner_Start();
    }
  }
  btn1_prev = b1;
  btn2_prev = b2;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  MX_CAN1_Init();
  MX_CAN2_Init();
  if ((RM_CAN_Init(&hcan1) != HAL_OK) ||
      (RM_CAN_Init(&hcan2) != HAL_OK)) {
    Error_Handler();
  }
  /* Bring up the UI even if no ESC is connected or the CAN bus has no ACK. */
  tft_init(PIN_ON_TOP, BLACK, WHITE, CYAN, DARK_GREY);
  tft_force_clear();
  tft_set_text_color(WHITE);
  tft_set_bg_color(BLACK);
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)operator_guide,
                          (uint16_t)(sizeof(operator_guide) - 1U), 1000U);
  MotorTuner_Init();
  next_screen_ms = HAL_GetTick();

  for (;;) {
    const uint32_t now = HAL_GetTick();
    service_buttons(now);
    /* Button debounce can block for 30 ms; use a fresh tick because starting
     * the tuner may have just timestamped feedback after the old `now`. */
    const uint32_t control_now = HAL_GetTick();
    MotorTuner_Tick(control_now);
    if ((int32_t)(control_now - next_screen_ms) >= 0) {
      draw_status();
      next_screen_ms = control_now + SCREEN_PERIOD_MS;
    }
    HAL_Delay(1U);
  }
}

/* Motor command outputs are zeroed and the control task is halted on init errors. */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 4;
  osc.PLL.PLLN = 168;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4;
  clk.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}
