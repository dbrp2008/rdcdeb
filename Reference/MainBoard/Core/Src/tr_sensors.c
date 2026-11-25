#include "tr_sensors.h"
#include "tr_drive.h"
#include "main.h"
#include "lcd/lcd.h"
#include "tr_auto_app.h"

#define US_TIMER_HZ 1000000u
#define US_SETTLE_US 2u
#define US_TRIGGER_US 12u
#define US_MIN_ECHO_US 100u
#define US_DELAY_WATCHDOG_MS 2u
#define US_WATCHDOG_MS (TR_US_TIMEOUT_US / 1000u + 3u)
#define BUTTON_DEBOUNCE_MS 30u

#define IR_COUNT 2u /* left and right */
static const struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  const char *name;
  const char *side;
} ir_pins[IR_COUNT] = {
    {GPIOB, GPIO_PIN_10, "PB10", "L"}, /* left sensor  */
    {GPIOB, GPIO_PIN_11, "PB11", "R"}, /* right sensor */
};

#define US_TRIG_PORT GPIOA
#define US_TRIG_PIN GPIO_PIN_0
#define US_ECHO_PORT GPIOA
#define US_ECHO_PIN GPIO_PIN_1

/* Per-sensor test: a sensor is judged OK once it has been seen both detecting and not detecting, each for at least
 * IR_TEST_STABLE_MS. A sensor that never changes is stuck, disconnected or mis-set. */
#define IR_TEST_STABLE_MS 30u
static bool ir_cand[IR_COUNT], ir_stable[IR_COUNT], ir_seen_on[IR_COUNT], ir_seen_off[IR_COUNT];
static uint32_t ir_since[IR_COUNT];
static uint8_t ir_changes[IR_COUNT];

static TRSensors_Status status;
static bool page_active = false;
static bool btn_stable = false, btn_candidate = false, btn2_stable = false, btn2_candidate = false;
static uint32_t btn_since = 0, btn2_since = 0, last_ping = 0, us_stamp = 0;
static bool us_keep = false, us_fast = false;

void TRSensors_UsKeepAlive(bool on, bool fast) {
  us_keep = on;
  us_fast = fast;
}

static void ir_test_reset(void) {
  for (unsigned i = 0; i < IR_COUNT; i++) {
    ir_seen_on[i] = status.ir[i];
    ir_seen_off[i] = !status.ir[i];
    ir_stable[i] = ir_cand[i] = status.ir[i];
    ir_since[i] = HAL_GetTick();
    ir_changes[i] = 0;
  }
}

static bool us_delay(uint32_t duration_us) {
  const uint32_t start_ms = HAL_GetTick();
  const uint32_t start_us = TIM2->CNT;
  while ((uint32_t)(TIM2->CNT - start_us) < duration_us) {
    if ((uint32_t)(HAL_GetTick() - start_ms) >= US_DELAY_WATCHDOG_MS)
      return false;
  }
  return true;
}

static bool us_wait(GPIO_PinState target, uint32_t start_us, uint32_t start_ms, uint32_t *edge_us) {
  for (;;) {
    const GPIO_PinState level = HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN);
    const uint32_t now_us = TIM2->CNT;
    if ((uint32_t)(now_us - start_us) >= TR_US_TIMEOUT_US || (uint32_t)(HAL_GetTick() - start_ms) >= US_WATCHDOG_MS)
      return false;
    if (level == target) {
      *edge_us = now_us;
      return true;
    }
  }
}

/* One ping. Returns 0 and fills the distance if a usable echo came back, otherwise a reason code (see us_reason). */
static uint8_t us_measure(int32_t *mm, uint32_t *echo_us) {
  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);
  if (HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == GPIO_PIN_SET)
    return 1; /* echo stuck high: wiring or sensor fault */
  if (!us_delay(US_SETTLE_US))
    return 5;
  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_SET);
  const bool ok = us_delay(US_TRIGGER_US);
  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);
  if (!ok)
    return 5;
  const uint32_t start_us = TIM2->CNT;
  const uint32_t start_ms = HAL_GetTick();
  uint32_t rise, fall;
  if (!us_wait(GPIO_PIN_SET, start_us, start_ms, &rise))
    return 2;
  if (!us_wait(GPIO_PIN_RESET, start_us, start_ms, &fall))
    return 3;
  const uint32_t width = fall - rise;
  *echo_us = width;
  if (width < US_MIN_ECHO_US)
    return 4;
  int32_t d = (int32_t)((width * 343u + 1000u) / 2000u) + TR_US_OFFSET_MM; /* 343 m/s, there and back */
  *mm = d < 0 ? 0 : d;
  return 0;
}

void TRSensors_Init(void) {
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  for (unsigned i = 0; i < IR_COUNT; i++) {
    gpio.Pin = ir_pins[i].pin;
    HAL_GPIO_Init(ir_pins[i].port, &gpio);
  }
  /* ultrasonic: TRIG low before it becomes an output */
  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET);
  gpio.Pin = US_TRIG_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US_TRIG_PORT, &gpio);
  gpio.Pin = US_ECHO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(US_ECHO_PORT, &gpio);

  /* TIM2 at 1 us per count (the timer clock is 2 x PCLK1 when APB1 is divided; 84 MHz on this robot) */
  uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1_2) != 0u)
    timer_hz *= 2u;
  if (timer_hz >= US_TIMER_HZ && timer_hz % US_TIMER_HZ == 0u) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();
    TIM2->PSC = timer_hz / US_TIMER_HZ - 1u;
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0u;
    TIM2->CR1 = TIM_CR1_CEN;
    status.us_timer_ok = true;
  } else {
    status.us_timer_ok = false;
  }
  last_ping = HAL_GetTick();
  for (unsigned i = 0; i < IR_COUNT; i++)
    status.ir[i] = (HAL_GPIO_ReadPin(ir_pins[i].port, ir_pins[i].pin) == GPIO_PIN_SET) == (TR_IR_DETECT_LEVEL != 0);
  ir_test_reset();
}

void TRSensors_Update(void) {
  const uint32_t now = HAL_GetTick();

  for (unsigned i = 0; i < IR_COUNT; i++) {
    const bool high = (HAL_GPIO_ReadPin(ir_pins[i].port, ir_pins[i].pin) == GPIO_PIN_SET);
    status.ir_raw[i] = high;
    status.ir[i] = (high == (TR_IR_DETECT_LEVEL != 0));
    /* debounced change tracking for the per-sensor test */
    if (status.ir[i] != ir_cand[i]) {
      ir_cand[i] = status.ir[i];
      ir_since[i] = now;
    }
    if (ir_cand[i] != ir_stable[i] && (uint32_t)(now - ir_since[i]) >= IR_TEST_STABLE_MS) {
      ir_stable[i] = ir_cand[i];
      if (ir_changes[i] < 99u) ir_changes[i]++;
      if (ir_stable[i]) ir_seen_on[i] = true; else ir_seen_off[i] = true;
    }
  }

  TRAutoApp_Update(); /* runs the autonomous routine when it is on (a no-op otherwise); AFTER the IR read, so it uses this loop's IRs */

#if TR_BENCH_TEST == 0
  /* BTN1 (debounced, press only) toggles the sensors page. In the bench modes BTN1 drives the motors instead. */
  const bool raw = btn_read(BTN1);
  if (raw != btn_candidate) {
    btn_candidate = raw;
    btn_since = now;
  }
  if (btn_candidate != btn_stable && (uint32_t)(now - btn_since) >= BUTTON_DEBOUNCE_MS) {
    btn_stable = btn_candidate;
    if (btn_stable) {
      if (TRAutoApp_PageActive()) {
        TRAutoApp_Btn1(); /* closes the auto page when it is not running */
      } else {
        page_active = !page_active;
        if (page_active)
          ir_test_reset(); /* a fresh test every time the page is opened */
      }
    }
  }
  /* BTN2 on the sensors page: restart the IR test */
  const bool raw2 = btn_read(BTN2);
  if (raw2 != btn2_candidate) {
    btn2_candidate = raw2;
    btn2_since = now;
  }
  if (btn2_candidate != btn2_stable && (uint32_t)(now - btn2_since) >= BUTTON_DEBOUNCE_MS) {
    btn2_stable = btn2_candidate;
    if (btn2_stable) {
      if (page_active)
        ir_test_reset(); /* on the sensors page BTN2 restarts the IR test */
      else
        TRAutoApp_Btn2(); /* otherwise it opens / starts / stops the autonomous routine */
    }
  }
#endif

  if (status.us_timer_ok && (TR_US_ALWAYS || page_active || us_keep) &&
      (uint32_t)(now - last_ping) >= (us_fast ? 50u : TR_US_PERIOD_MS)) {
    last_ping = now;
    int32_t mm;
    uint32_t echo;
    const uint8_t reason = us_measure(&mm, &echo);
    status.us_reason = reason;
    if (reason == 0u) {
      status.us_mm = mm;
      status.us_echo_us = echo;
      status.us_valid = true;
      status.us_seq++;
      us_stamp = now;
    } else if ((uint32_t)(now - us_stamp) > 400u) {
      status.us_valid = false;
    }
  }
}

void TRSensors_Get(TRSensors_Status *out) {
  if (out)
    *out = status;
}

bool TRSensors_PageActive(void) { return page_active || TRAutoApp_PageActive(); }

void TRSensors_DrawPage(void) {
  if (TRAutoApp_PageActive()) {
    TRAutoApp_DrawPage();
    return;
  }
  tft_printc(0, 0, "IR TEST B2:RESET");
  for (unsigned i = 0; i < IR_COUNT; i++) {
    const bool ok = ir_seen_on[i] && ir_seen_off[i];
    tft_prints(0, (uint8_t)(i + 1u), "%s %-4s %-3s x%-2u%s", ir_pins[i].side, ir_pins[i].name, status.ir[i] ? "ON" : "off",
               (unsigned)ir_changes[i], ok ? "OK" : ".."); /* the change count goes up to 99 */
  }
  { /* wheel encoders: travel of each wheel in mm since power-up (FL, FR on row 4; back + forward / sideways on row 6) */
    int32_t mm[TR_WHEEL_COUNT];
    TRDrive_WheelTravelMm(mm);
    tft_prints(0, 4, "L%6d R%6d  ", (int)mm[TR_WHEEL_FL], (int)mm[TR_WHEEL_FR]);
    tft_prints(0, 6, "B%6d S%6d  ", (int)mm[TR_WHEEL_B], (int)(TRDrive_StrafeDistanceM() * 1000.0f));
  }
  /* Servo PWM check (TIM4 channel 2 on PB7): the compare value being sent, and whether the timer and the channel
   * output are really switched on. "ON" with a sensible number means the MainBoard is outputting the signal. */
  {
    const bool run = (TIM4->CR1 & TIM_CR1_CEN) != 0u;
    const bool out = (TIM4->CCER & TIM_CCER_CC2E) != 0u;
    tft_prints(0, 5, "SRV %4u %s%s", (unsigned)TIM4->CCR2, run ? "RUN" : "OFF", out ? "+OUT" : ""); /* 16 chars: e.g. SRV 2500 RUN+OUT */
  }
  if (!status.us_timer_ok)
    tft_printc(0, 7, "US: CHECK TIM2  ");
  else if (status.us_valid)
    tft_prints(0, 7, "US %5d mm    ", (int)status.us_mm);
  else
    tft_printc(0, 7, "US NO ECHO      ");
  if (status.us_timer_ok && !status.us_valid) {
    static const char *const why[] = {"", "ECHO STUCK HIGH ", "NO ECHO START   ", "ECHO NEVER ENDS ", "ECHO TOO SHORT  ", "TRIGGER FAULT   "};
    tft_printc(0, 8, why[status.us_reason <= 5u ? status.us_reason : 0u]);
  } else {
    tft_printc(0, 8, "                ");
  }
  tft_printc(0, 9, "B1: BACK       ");
}
