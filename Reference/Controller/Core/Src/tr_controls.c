#include "tr_controls.h"
#include "main.h"
#include "usart.h"
#include "lcd/lcd.h"
#include <stdio.h>
#include <stdlib.h>

/* Joystick calibration (raw 12-bit ADC counts). MIN < CENTRE < MAX. The values below are the warmaster726 starter
 * readings; the TFT shows your raw readings so you can replace them with your own sticks' numbers. */
#define JOY_LX_MIN 2
#define JOY_LX_CENTRE 2075
#define JOY_LX_MAX 4095
#define JOY_LY_MIN 0
#define JOY_LY_CENTRE 1993
#define JOY_LY_MAX 4094
#define JOY_RX_MIN 1
#define JOY_RX_CENTRE 1944
#define JOY_RX_MAX 4095
#define JOY_DEADBAND 120      /* counts around centre treated as 0 */
#define JOY_ENDPOINT_SNAP 16  /* within this many counts of an end => full 100 */
#define MECH_DEBOUNCE_MS 30U
#define MECH_SWAP_78 1      /* 1 = Button 7 sends GC (close) and Button 8 sends GO (open); 0 = 7 = GO, 8 = GC */
#define JOY_INVERT_ROT 1     /* 1 = flip the right-stick rotation direction (the robot turned the wrong way with 0) */
#define SPEED_LEVELS 10
#define TURN_CHORD_MS 120U /* both turn buttons must be down this long to count as the 180 chord */
#define SPEED_REPEAT_DELAY_MS 400U /* hold a speed button this long before it starts repeating */
#define SPEED_REPEAT_MS 150U       /* then one step every this many ms while held */
#define SPEED_DEFAULT 4

/* Sticks wired to PA0..PA3 (ADC1 channels 0..3): LX, LY, RX, RY. */
#define ADC_CH_LX 0U
#define ADC_CH_LY 1U
#define ADC_CH_RX 2U

static void tft_signed(uint8_t x, uint8_t y, int v) { /* this tft printf has no %+d: draw the sign separately */
  tft_printc(x, y, v < 0 ? "-" : "+");
  tft_prints((uint8_t)(x + 1u), y, "%3d", v < 0 ? -v : v);
}

static uint8_t rx_byte;
static volatile uint32_t bt_rx_count = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart1)
    return;
  bt_rx_count++;
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart1)
    return;
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* ---- ADC1, polled, register level (no CubeMX ADC setup needed) ---- */
static void adc_init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* PA0..PA3 analog input: CNF=00 MODE=00 */
  GPIOA->CRL &= ~(0xFFFFU);
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6; /* 72/6 = 12 MHz */
  ADC1->CR2 = ADC_CR2_ADON;
  for (volatile int i = 0; i < 2000; i++) {
  }
  ADC1->CR2 |= ADC_CR2_RSTCAL;
  while (ADC1->CR2 & ADC_CR2_RSTCAL) {
  }
  ADC1->CR2 |= ADC_CR2_CAL;
  while (ADC1->CR2 & ADC_CR2_CAL) {
  }
  ADC1->SMPR2 = (6U << 0) | (6U << 3) | (6U << 6) | (6U << 9);
  ADC1->CR2 |= ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG; /* software trigger */
  ADC1->SQR1 = 0; /* 1 conversion */
}

static uint16_t adc_read(uint32_t ch) {
  ADC1->SQR3 = ch;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  uint32_t guard = 100000U;
  while (!(ADC1->SR & ADC_SR_EOC) && guard--) {
  }
  return (uint16_t)(ADC1->DR & 0x0FFFU);
}

/* Raw count -> -100..100 (hundredths of a full stick), deadband around the centre, snap to the ends. */
static int normalize_axis(int raw, int mn, int ctr, int mx) {
  if (mn >= ctr || ctr >= mx)
    return 0;
  int d = raw - ctr;
  if (d >= -JOY_DEADBAND && d <= JOY_DEADBAND)
    return 0;
  int v;
  if (d > 0) {
    int usable = mx - ctr - JOY_DEADBAND;
    if (usable <= 0)
      return 0;
    if (mx - raw <= JOY_ENDPOINT_SNAP)
      return 100;
    v = ((d - JOY_DEADBAND) * 100 + usable / 2) / usable;
  } else {
    int usable = ctr - mn - JOY_DEADBAND;
    if (usable <= 0)
      return 0;
    if (raw - mn <= JOY_ENDPOINT_SNAP)
      return -100;
    v = -((-d - JOY_DEADBAND) * 100 + usable / 2) / usable;
  }
  if (v > 100)
    v = 100;
  if (v < -100)
    v = -100;
  return v;
}

/* "+0.50" style, always sign + 1 digit + '.' + 2 digits. Writes 5 chars. */
static void put_ratio(char *p, int hundredths) {
  p[0] = (hundredths < 0) ? '-' : '+';
  int m = abs(hundredths);
  p[1] = (char)('0' + m / 100);
  p[2] = '.';
  p[3] = (char)('0' + (m / 10) % 10);
  p[4] = (char)('0' + m % 10);
}

typedef struct {
  uint8_t stable, cand;
  uint32_t since;
} Debounced;

static void debounce(Debounced *b, uint8_t raw, uint32_t now) {
  if (raw != b->cand) {
    b->cand = raw;
    b->since = now;
  }
  if (b->cand != b->stable && (now - b->since) >= MECH_DEBOUNCE_MS)
    b->stable = b->cand;
}

void TRControls_Init(void) {
  /* CubeMX leaves the buttons as NOPULL; they need a pull-up or they float. */
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  const struct { GPIO_TypeDef *port; uint16_t pin; } pins[] = {
      {Button7_GPIO_Port, Button7_Pin},
      {Button8_GPIO_Port, Button8_Pin},
      {Button5_GPIO_Port, Button5_Pin},
      {Button6_GPIO_Port, Button6_Pin},
      {Button1_GPIO_Port, Button1_Pin},
      {Button4_GPIO_Port, Button4_Pin},
      {Button2_GPIO_Port, Button2_Pin},
      {Button3_GPIO_Port, Button3_Pin},
  };
  for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
    gpio.Pin = pins[i].pin;
    HAL_GPIO_Init(pins[i].port, &gpio);
  }
  adc_init();
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void TRControls_Update(void) {
  static uint32_t last_send = 0;
  static Debounced b7, b8, b5, b6, b1, b2, b3, b4;
  static int heading_mode = 0;          /* 1 = heading-relative driving */
  static char turn_cmd = '0';           /* what is sent on the Y line */
  static int turn_latched = 0;          /* a turn was sent: send nothing until both turn buttons are released */
  static uint32_t turn_pat_since = 0, turn_send_until = 0;
  static char turn_pat = '0';
  static int speed = SPEED_DEFAULT;
  static char mech[3] = {'G', 'S', 0};
  static int started = 0;
  static int lx_raw, ly_raw, rx_raw, mx, my, rz;

  uint32_t now = HAL_GetTick();
  debounce(&b7, btn_read(Button7) ? 1U : 0U, now);
  debounce(&b8, btn_read(Button8) ? 1U : 0U, now);
  {
    /* Button 1: toggle heading-relative driving (one change per press). Button 4: zero the heading (while held). */
    uint8_t p1 = b1.stable;
    debounce(&b1, btn_read(Button1) ? 1U : 0U, now);
    debounce(&b4, btn_read(Button4) ? 1U : 0U, now);
    if (b1.stable && !p1)
      heading_mode = !heading_mode;

    /* Buttons 2 / 3: 2 = turn 90 left, 3 = turn 90 right, both = 180. The pattern must be steady for TURN_CHORD_MS so a
     * slightly staggered two-button press is read as 180 and not as a 90 followed by something else. After a turn
     * has been sent nothing more is sent until both buttons are released, so a held button turns only once. */
    debounce(&b2, btn_read(Button2) ? 1U : 0U, now);
    debounce(&b3, btn_read(Button3) ? 1U : 0U, now);
    char pat = (b2.stable && b3.stable) ? 'U' : (b2.stable ? 'L' : (b3.stable ? 'R' : '0'));
    if (pat != turn_pat) {
      turn_pat = pat;
      turn_pat_since = now;
    }
    if (pat == '0')
      turn_latched = 0;
    if (turn_latched && turn_send_until != 0 && (int32_t)(now - turn_send_until) < 0) {
      /* keep the turn command on the line for a few frames */
    } else if (turn_latched || pat == '0' || (now - turn_pat_since) < TURN_CHORD_MS) {
      turn_cmd = '0';
    } else {
      turn_cmd = pat;
      turn_latched = 1;
      turn_send_until = now + 200U; /* keep sending it for a few frames so a 50 ms send never misses it */
    }
  }
  {
    /* Buttons 5 / 6 change the speed level: one step on press, then auto-repeat while held. */
    static uint32_t next_step5 = 0, next_step6 = 0;
    uint8_t p5 = b5.stable, p6 = b6.stable;
    debounce(&b5, btn_read(Button5) ? 1U : 0U, now);
    debounce(&b6, btn_read(Button6) ? 1U : 0U, now);
    if (b5.stable && !b6.stable) {
      if (!p5) {
        if (speed < SPEED_LEVELS) speed++;
        next_step5 = now + SPEED_REPEAT_DELAY_MS;
      } else if ((int32_t)(now - next_step5) >= 0) {
        if (speed < SPEED_LEVELS) speed++;
        next_step5 = now + SPEED_REPEAT_MS;
      }
    }
    if (b6.stable && !b5.stable) {
      if (!p6) {
        if (speed > 1) speed--;
        next_step6 = now + SPEED_REPEAT_DELAY_MS;
      } else if ((int32_t)(now - next_step6) >= 0) {
        if (speed > 1) speed--;
        next_step6 = now + SPEED_REPEAT_MS;
      }
    }
  }

  /* Deadman: exactly one of 7/8 held => GO / GC. Neither, or both => GS. */
  char want = 'S';
  if (b7.stable && !b8.stable)
    want = MECH_SWAP_78 ? 'C' : 'O';
  else if (b8.stable && !b7.stable)
    want = MECH_SWAP_78 ? 'O' : 'C';

  {
    if ((now - last_send) >= TR_SEND_PERIOD_MS || !started || want != mech[1]) {
      lx_raw = (int)adc_read(ADC_CH_LX);
      ly_raw = (int)adc_read(ADC_CH_LY);
      rx_raw = (int)adc_read(ADC_CH_RX);
      mx = normalize_axis(lx_raw, JOY_LX_MIN, JOY_LX_CENTRE, JOY_LX_MAX);
      my = -normalize_axis(ly_raw, JOY_LY_MIN, JOY_LY_CENTRE, JOY_LY_MAX); /* stick up = forward = + */
      rz = normalize_axis(rx_raw, JOY_RX_MIN, JOY_RX_CENTRE, JOY_RX_MAX);  /* stick right = clockwise = + */
      if (JOY_INVERT_ROT)
        rz = -rz;
      mech[1] = want;
      started = 1;
      last_send = now;

      char line[16];
      line[0] = 'M';
      put_ratio(line + 1, mx);
      line[6] = '/';
      put_ratio(line + 7, my);
      line[12] = '\n';
      HAL_UART_Transmit(&huart1, (uint8_t *)line, 13, 10);
      line[0] = 'R';
      put_ratio(line + 1, rz);
      line[6] = '\n';
      HAL_UART_Transmit(&huart1, (uint8_t *)line, 7, 10);
      line[0] = 'G';
      line[1] = mech[1];
      line[2] = '\n';
      HAL_UART_Transmit(&huart1, (uint8_t *)line, 3, 10);
      line[0] = 'S'; /* speed level */
      line[1] = (char)('0' + speed - 1); /* S0..S9 = level 1..10 */
      line[2] = '\n';
      HAL_UART_Transmit(&huart1, (uint8_t *)line, 3, 10);
      line[0] = 'H'; /* heading mode: H0 normal, H1 heading-relative */
      line[1] = heading_mode ? '1' : '0';
      line[2] = '\n';
      HAL_UART_Transmit(&huart1, (uint8_t *)line, 3, 10);
      if (b4.stable) { /* button 4 held: zero the heading */
        line[1] = 'Z';
        HAL_UART_Transmit(&huart1, (uint8_t *)line, 3, 10);
      }
      line[0] = 'Y'; /* turn: YL / YR / YU, otherwise Y0 */
      line[1] = turn_cmd;
      line[2] = '\n';
      HAL_UART_Transmit(&huart1, (uint8_t *)line, 3, 10);
      led_toggle(LED1);
    }
  }

  if (mx || my || rz || want != 'S')
    led_on(LED2);
  else
    led_off(LED2);

  tft_prints(0, 0, "TR JOYSTICK");
  tft_printc(0, 1, "M");
  tft_signed(2, 1, mx);
  tft_signed(7, 1, my);
  tft_printc(0, 2, "R");
  tft_signed(2, 2, rz);
  tft_prints(0, 3, MECH_SWAP_78 ? "G%c 7:CLS 8:OPN" : "G%c 7:OPN 8:CLS", mech[1]);
  tft_prints(0, 5, heading_mode ? "1:HEAD  4:ZERO" : "1:NORM  4:ZERO");
  tft_prints(0, 6, "2:L90 3:R90 23:U");
  tft_prints(0, 7, "RAW %-4d%-4d%-4d", lx_raw, ly_raw, rx_raw);
  tft_prints(0, 4, "SPD %d/10 5+ 6-", speed);
  tft_prints(0, 9, "BT RX:%-6u", (unsigned)bt_rx_count);
  tft_update(50);
}
