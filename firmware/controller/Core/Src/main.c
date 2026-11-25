/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_uart.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd/lcd.h"
#include "lcd/lcd_graphics.h"
#include "rdc_protocol.h"
#include "joystick_calibration.h"
#include "joystick_dma_watchdog.h"
#include "mechanism_mapping.h"
#include "turn_buttons.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HOLD_RESEND_MS      100U   /* CW / CCW repeat period while held */
#define MECH_DEBOUNCE_MS     30U
#define UART_TX_TIMEOUT_MS   50U
#define RDC_TX_PERIOD_MS      50U
#define RDC_DISPLAY_PERIOD_MS 100U
#define RDC_ACK_QUEUE_LEN      8U
#define RDC_JOY_WARNING_THRESHOLD_PCT 60
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint16_t joystick_adc[4] __attribute__((aligned(4)));
static JoystickDmaWatchdog joystick_dma;
static int16_t ratio_x, ratio_y, ratio_z;
typedef struct {
  uint8_t stable_pressed;
  uint8_t candidate_pressed;
  uint32_t candidate_since;
} DebouncedButton;
static DebouncedButton mechanism_button7, mechanism_button8;
static DebouncedButton heading_toggle_button, heading_zero_button;
static uint8_t heading_mode;
static DebouncedButton turn_left_button, turn_right_button;
static TurnButtons turn_buttons = {RDC_MESSAGE_TURN_IDLE, RDC_MESSAGE_TURN_IDLE, 0U, 0U, 0U};
static RdcMessageType mechanism_command = RDC_MESSAGE_MECHANISM_STOP;
static RdcMessageType last_mechanism_sent = RDC_MESSAGE_NONE;
static uint8_t mechanism_sent_known;
static uint32_t last_mechanism_tx_tick;
static uint32_t next_tx_tick, next_display_tick;
static uint32_t g_tx_count, g_ack_count, g_ack_bad, g_uart_errors, g_ack_overflow;
static uint16_t g_last_ack_sequence;
static uint8_t g_last_ack_status;
static uint32_t g_last_ack_tick;
static char g_last_sent[24] = "-";

/* ACK RX uses an ISR-to-main-loop queue so checksum parsing stays out of IRQ context. */
static uint8_t ack_rx_byte;
static char ack_rx_line[RDC_PROTOCOL_MAX_FRAME];
static uint8_t ack_rx_length, ack_rx_discard;
static char ack_queue[RDC_ACK_QUEUE_LEN][RDC_PROTOCOL_MAX_FRAME];
static uint8_t ack_queue_length[RDC_ACK_QUEUE_LEN];
static volatile uint8_t ack_queue_head, ack_queue_tail;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Poll the latched DMA flags rather than enabling a high-rate scan interrupt.
 * A completion proves that at least one complete four-channel scan occurred,
 * not that the joystick wiring or voltages themselves are healthy. */
static void update_joystick_dma(uint32_t now)
{
  const uint32_t flags = DMA1->ISR;
  const int running = ((DMA1_Channel1->CCR & DMA_CCR_EN) != 0U) &&
                      ((ADC1->CR2 & (ADC_CR2_ADON | ADC_CR2_DMA)) ==
                       (ADC_CR2_ADON | ADC_CR2_DMA));
  const int completed = (flags & DMA_ISR_TCIF1) != 0U;
  if (completed) DMA1->IFCR = DMA_IFCR_CTCIF1;
  joystick_dma_observe(&joystick_dma, now, running,
                       (flags & DMA_ISR_TEIF1) != 0U, completed);
}

static int16_t normalize_axis(uint16_t raw, uint16_t minimum,
                              uint16_t centre, uint16_t maximum,
                              uint16_t deadband)
{
  if ((minimum >= centre) || (centre >= maximum)) return 0;

  const int32_t delta = (int32_t)raw - (int32_t)centre;
  if ((delta >= -(int32_t)deadband) && (delta <= (int32_t)deadband)) return 0;

  int32_t value;
  if (delta > 0) {
    const int32_t usable = (int32_t)maximum - (int32_t)centre - (int32_t)deadband;
    if (usable <= 0) return 0;
    if (((int32_t)maximum - (int32_t)raw) <= (int32_t)RDC_JOY_ENDPOINT_SNAP_COUNTS)
      return 100;
    const int32_t active_delta = delta - (int32_t)deadband;
    value = (active_delta * 100 + usable / 2) / usable;
  } else {
    const int32_t usable = (int32_t)centre - (int32_t)minimum - (int32_t)deadband;
    if (usable <= 0) return 0;
    if (((int32_t)raw - (int32_t)minimum) <= (int32_t)RDC_JOY_ENDPOINT_SNAP_COUNTS)
      return -100;
    const int32_t active_delta = -delta - (int32_t)deadband;
    value = -((active_delta * 100 + usable / 2) / usable);
  }

  if (value > 100) value = 100;
  if (value < -100) value = -100;
  return (int16_t)value;
}

/* LED1 warns on left-stick translation; LED2 warns on right-stick rotation.
 * A warning is shown only when a normalized joystick ratio exceeds 60%. */
static void update_joystick_warning_leds(void)
{
  if ((ratio_x > RDC_JOY_WARNING_THRESHOLD_PCT) ||
      (ratio_x < -RDC_JOY_WARNING_THRESHOLD_PCT) ||
      (ratio_y > RDC_JOY_WARNING_THRESHOLD_PCT) ||
      (ratio_y < -RDC_JOY_WARNING_THRESHOLD_PCT)) {
    led_on(LED1);
  } else {
    led_off(LED1);
  }

  if ((ratio_z > RDC_JOY_WARNING_THRESHOLD_PCT) ||
      (ratio_z < -RDC_JOY_WARNING_THRESHOLD_PCT)) {
    led_on(LED2);
  } else {
    led_off(LED2);
  }
}

static void uart_send_line(const char *line, size_t length)
{
  if (length == 0U || length > RDC_PROTOCOL_MAX_FRAME) return;
  if (HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)length,
                        UART_TX_TIMEOUT_MS) != HAL_OK) return;
  g_tx_count++;
  size_t visible = (length > 0U && line[length - 1U] == '\n') ? length - 1U : length;
  if (visible >= sizeof(g_last_sent)) visible = sizeof(g_last_sent) - 1U;
  memcpy(g_last_sent, line, visible);
  g_last_sent[visible] = '\0';
}

static void send_motion_lines(void)
{
  char line[RDC_PROTOCOL_MAX_FRAME];
  size_t length = rdc_encode_move(line, sizeof(line), ratio_x, ratio_y);
  uart_send_line(line, length);
  length = rdc_encode_rotation(line, sizeof(line), ratio_z);
  uart_send_line(line, length);
}

static void update_debounced_button(DebouncedButton *button, uint8_t raw_pressed,
                                    uint32_t now)
{
  if (raw_pressed != button->candidate_pressed) {
    button->candidate_pressed = raw_pressed;
    button->candidate_since = now;
  }
  if ((button->candidate_pressed != button->stable_pressed) &&
      ((uint32_t)(now - button->candidate_since) >= MECH_DEBOUNCE_MS)) {
    button->stable_pressed = button->candidate_pressed;
  }
}

static void update_mechanism_buttons(uint32_t now)
{
  update_debounced_button(&mechanism_button7, btn_read(Button7) ? 1U : 0U, now);
  update_debounced_button(&mechanism_button8, btn_read(Button8) ? 1U : 0U, now);

  /* Button 7 closes, button 8 opens; neither or both pressed means STOP. */
  RdcMessageType desired = mechanism_command_for_buttons(
      mechanism_button7.stable_pressed, mechanism_button8.stable_pressed);
  mechanism_command = desired;

  /* Repeat the active command as a hold/deadman signal; send STOP on release
   * or conflict, and periodically so the receiver refreshes its watchdog. */
  if (!mechanism_sent_known || (desired != last_mechanism_sent) ||
      ((uint32_t)(now - last_mechanism_tx_tick) >= RDC_TX_PERIOD_MS)) {
    char line[RDC_PROTOCOL_MAX_FRAME];
    const size_t length = rdc_encode_mechanism(line, sizeof(line), desired);
    uart_send_line(line, length);
    last_mechanism_sent = desired;
    last_mechanism_tx_tick = now;
    mechanism_sent_known = 1U;
  }
}

static void update_heading_buttons(uint32_t now)
{
  const uint8_t was_pressed = heading_toggle_button.stable_pressed;
  update_debounced_button(&heading_toggle_button, btn_read(Button1) ? 1U : 0U, now);
  update_debounced_button(&heading_zero_button, btn_read(Button4) ? 1U : 0U, now);
  if (heading_toggle_button.stable_pressed && !was_pressed) heading_mode ^= 1U;
}

static void send_heading_lines(void)
{
  char line[RDC_PROTOCOL_MAX_FRAME];
  RdcMessageType mode = heading_mode ? RDC_MESSAGE_HEADING_ON : RDC_MESSAGE_HEADING_OFF;
  size_t length = rdc_encode_heading(line, sizeof(line), mode);
  uart_send_line(line, length);
  if (heading_zero_button.stable_pressed) {
    length = rdc_encode_heading(line, sizeof(line), RDC_MESSAGE_HEADING_ZERO);
    uart_send_line(line, length);
  }
}

static void update_turn_buttons(uint32_t now)
{
  update_debounced_button(&turn_left_button, btn_read(Button2) ? 1U : 0U, now);
  update_debounced_button(&turn_right_button, btn_read(Button3) ? 1U : 0U, now);
  turn_buttons_update(&turn_buttons, turn_left_button.stable_pressed,
                      turn_right_button.stable_pressed, now);
}

static void send_turn_line(void)
{
  char line[RDC_PROTOCOL_MAX_FRAME];
  /* ADC failure must cancel automatic movement too, not merely zero sticks. */
  RdcMessageType command = joystick_dma.state == JOYSTICK_DMA_OK ?
                           turn_buttons.output : RDC_MESSAGE_TURN_CANCEL;
  size_t length = rdc_encode_turn(line, sizeof(line), command);
  uart_send_line(line, length);
}

static const char *mechanism_text(RdcMessageType command)
{
  switch (command) {
    case RDC_MESSAGE_MECHANISM_OPEN: return "OPEN";
    case RDC_MESSAGE_MECHANISM_CLOSE: return "CLOSE";
    default: return "STOP";
  }
}

static void ack_rx_start(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &ack_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart1) return;
  const char c = (char)ack_rx_byte;
  if (c == '\n') {
    if (!ack_rx_discard && ack_rx_length > 0U) {
      const uint8_t next = (uint8_t)((ack_queue_head + 1U) % RDC_ACK_QUEUE_LEN);
      if (next != ack_queue_tail) {
        memcpy(ack_queue[ack_queue_head], ack_rx_line, ack_rx_length);
        ack_queue_length[ack_queue_head] = ack_rx_length;
        ack_queue_head = next;
      } else {
        ++g_ack_overflow;
      }
    }
    ack_rx_length = 0U;
    ack_rx_discard = 0U;
  } else if (c == '\r') {
    ack_rx_discard = 1U; /* LF is mandatory; reject CRLF. */
  } else if (!ack_rx_discard) {
    if (ack_rx_length < sizeof(ack_rx_line) - 1U) {
      ack_rx_line[ack_rx_length++] = c;
    } else {
      ack_rx_length = 0U;
      ack_rx_discard = 1U;
      ++g_ack_bad;
    }
  }
  ack_rx_start();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) {
    ++g_uart_errors;
    ack_rx_length = 0U;
    ack_rx_discard = 1U;
    ack_rx_start();
  }
}

static void process_ack_lines(void)
{
  while (ack_queue_tail != ack_queue_head) {
    char line[RDC_PROTOCOL_MAX_FRAME];
    const uint8_t length = ack_queue_length[ack_queue_tail];
    memcpy(line, ack_queue[ack_queue_tail], length);
    ack_queue_tail = (uint8_t)((ack_queue_tail + 1U) % RDC_ACK_QUEUE_LEN);
    line[length] = '\n';
    RdcMessage ack;
    if (rdc_parse_line(line, (size_t)length + 1U, &ack) && ack.type == RDC_MESSAGE_ACK) {
      ++g_ack_count;
      g_last_ack_sequence = ack.sequence;
      g_last_ack_status = ack.status;
      g_last_ack_tick = HAL_GetTick();
    } else {
      ++g_ack_bad;
    }
  }
}

static void ratio_text(char *out, size_t capacity, int16_t ratio)
{
  const unsigned magnitude = (unsigned)((ratio < 0) ? -ratio : ratio);
  (void)snprintf(out, capacity, "%c%u.%02u", (ratio < 0) ? '-' : '+',
                 magnitude / 100U, magnitude % 100U);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  tft_init(PIN_ON_TOP, BLACK, WHITE, CYAN, DARK_GREY);
  led_off(LED1);
  led_off(LED2);
  const HAL_StatusTypeDef adc_start = HAL_ADC_Start_DMA(
      &hadc1, (uint32_t *)(uintptr_t)joystick_adc, 4U);
  if (adc_start == HAL_OK) {
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);
  }
  joystick_dma_begin(&joystick_dma, HAL_GetTick(), adc_start == HAL_OK);
  ack_rx_start();
  next_tx_tick = HAL_GetTick();
  next_display_tick = next_tx_tick;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const uint32_t now = HAL_GetTick();
    process_ack_lines();
    update_joystick_dma(HAL_GetTick());
    update_mechanism_buttons(now);
    update_heading_buttons(now);
    update_turn_buttons(now);

    if ((int32_t)(now - next_tx_tick) >= 0) {
      if (joystick_dma.state == JOYSTICK_DMA_OK) {
        ratio_x = normalize_axis(joystick_adc[0], RDC_JOY_LX_MIN,
                                 RDC_JOY_LX_CENTRE, RDC_JOY_LX_MAX,
                                 RDC_JOY_DEADBAND);
        ratio_y = (int16_t)-normalize_axis(joystick_adc[1], RDC_JOY_LY_MIN,
                                           RDC_JOY_LY_CENTRE, RDC_JOY_LY_MAX,
                                           RDC_JOY_DEADBAND);
        ratio_z = normalize_axis(joystick_adc[2], RDC_JOY_RX_MIN,
                                 RDC_JOY_RX_CENTRE, RDC_JOY_RX_MAX,
                                 RDC_JOY_DEADBAND); /* right-stick X: + clockwise */
      } else {
        ratio_x = ratio_y = ratio_z = 0; /* inhibit driving until a valid scan */
      }
      update_joystick_warning_leds();
      send_motion_lines();
      send_heading_lines();
      send_turn_line();

      next_tx_tick = now + RDC_TX_PERIOD_MS;
    }

    if ((int32_t)(now - next_display_tick) >= 0) {
      char x_text[8], y_text[8], z_text[8];
      ratio_text(x_text, sizeof(x_text), ratio_x);
      ratio_text(y_text, sizeof(y_text), ratio_y);
      ratio_text(z_text, sizeof(z_text), ratio_z);
      const char *ack_state = (g_last_ack_tick != 0U &&
          (now - g_last_ack_tick) <= 350U) ?
          (g_last_ack_status == 0U ? "UP" : "ERR") : "WAIT";
      tft_prints(1, 0, "RDC CONTROLLER");
      tft_prints(1, 1, "ACK:%s #%u", ack_state, (unsigned)g_last_ack_sequence);
      tft_prints(1, 2, "LX:%s", x_text);
      tft_prints(1, 3, "LY:%s", y_text);
      tft_prints(1, 4, "RZ:%s", z_text);
      tft_prints(1, 5, "MECH:%s", mechanism_text(mechanism_command));
      tft_prints(1, 6, "TX:%u RX:%u", (unsigned)g_tx_count, (unsigned)g_ack_count);
      tft_prints(1, 7, "BAD:%u OVF:%u", (unsigned)g_ack_bad, (unsigned)g_ack_overflow);
      tft_prints(1, 8, "UART ERR:%u", (unsigned)g_uart_errors);
      tft_prints(1, 9, "HEAD:%s B4:ZERO", heading_mode ? "ON " : "OFF");
      tft_prints(1, 10, "B7:CLOSE B8:OPEN");
      tft_prints(1, 11, "ADC:%-5s", joystick_dma.state == JOYSTICK_DMA_OK ? "OK" :
                 (joystick_dma.state == JOYSTICK_DMA_WAIT ? "WAIT" : "FAULT"));
      tft_update(50U);
      next_display_tick = now + RDC_DISPLAY_PERIOD_MS;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
