/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MainBoard communication progress display.
  ******************************************************************************
  * Application code is kept inside CubeMX USER CODE sections.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd/lcd.h"
#include "lcd/lcd_graphics.h"
#include "rdc_protocol.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  CMD_NONE = 0,
  CMD_MOVE,
  CMD_ROTATE,
  CMD_MECHANISM_OPEN,
  CMD_MECHANISM_CLOSE,
  CMD_MECHANISM_STOP
} cmd_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RDC_LINK_TIMEOUT_MS 350U
#define RDC_DISPLAY_PERIOD_MS 100U
#define RDC_MOTION_AVG_SAMPLES 3U
#define BT_LINE_MAX 16U
#define BT_QUEUE_LEN 8U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Motion setpoints are moving-averaged over the latest received commands.
 * Trigger events remain discrete and are never averaged. */
static int16_t move_x_centi, move_y_centi, rotation_z_centi;
static RdcMessageType mechanism_command = RDC_MESSAGE_MECHANISM_STOP;
static int16_t move_history[RDC_MOTION_AVG_SAMPLES][2];
static int16_t rotation_history[RDC_MOTION_AVG_SAMPLES];
static uint8_t move_history_count, move_history_next;
static uint8_t rotation_history_count, rotation_history_next;
static uint8_t link_timed_out;
static uint8_t have_valid_command;
static uint16_t ack_sequence;
static uint32_t last_valid_command_tick, next_display_tick;
static uint32_t command_ok, command_bad, ack_errors;

/* USART1 byte-to-line receiver. RX is handled in the ISR; command parsing is foreground-only. */
static uint8_t bt_rx_byte, bt_rx_length, bt_rx_discard;
static char bt_rx_line[BT_LINE_MAX];
static char bt_line_queue[BT_QUEUE_LEN][BT_LINE_MAX];
static volatile uint8_t bt_queue_head, bt_queue_tail;
static volatile uint32_t bt_uart_errors, bt_overflows, bt_dropped;

static cmd_t last_command = CMD_NONE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void mainboard_app_init(void);
static void mainboard_app_step(void);
static void bt_rx_start(void);
static bool bt_rx_get_line(char *out);
static cmd_t bt_parse(const char *line, RdcMessage *message);
static const char *cmd_name(cmd_t command);
static void send_ack(void);
static void draw_progress_screen(uint32_t now);
static void ratio_text(char *out, size_t capacity, int16_t ratio);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void bt_rx_start(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1U);
}

static bool bt_rx_get_line(char *out)
{
  if (bt_queue_head == bt_queue_tail) return false;
  memcpy(out, bt_line_queue[bt_queue_tail], BT_LINE_MAX);
  bt_queue_tail = (uint8_t)((bt_queue_tail + 1U) % BT_QUEUE_LEN);
  return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart1) return;

  const char byte = (char)bt_rx_byte;
  if (byte == '\n') {
    if (!bt_rx_discard && bt_rx_length > 0U) {
      const uint8_t next = (uint8_t)((bt_queue_head + 1U) % BT_QUEUE_LEN);
      bt_rx_line[bt_rx_length] = '\0';
      if (next != bt_queue_tail) {
        memcpy(bt_line_queue[bt_queue_head], bt_rx_line, BT_LINE_MAX);
        bt_queue_head = next;
      } else {
        ++bt_dropped;
      }
    }
    bt_rx_length = 0U;
    bt_rx_discard = 0U;
  } else if (byte == '\r') {
    /* The wire protocol is LF-only; discard CRLF lines through their LF. */
    bt_rx_length = 0U;
    bt_rx_discard = 1U;
  } else if (!bt_rx_discard) {
    if (bt_rx_length < BT_LINE_MAX - 1U) {
      bt_rx_line[bt_rx_length++] = byte;
    } else {
      bt_rx_length = 0U;
      bt_rx_discard = 1U;
      ++bt_overflows;
    }
  }
  bt_rx_start();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart1) return;
  ++bt_uart_errors;
  bt_rx_length = 0U;
  bt_rx_discard = 1U;
  bt_rx_start();
}

static cmd_t bt_parse(const char *line, RdcMessage *message)
{
  char frame[BT_LINE_MAX + 1U];
  const size_t length = strlen(line);
  memset(message, 0, sizeof(*message));
  if ((length + 1U) > sizeof(frame)) return CMD_NONE;
  memcpy(frame, line, length);
  frame[length] = '\n';
  if (!rdc_parse_line(frame, length + 1U, message)) return CMD_NONE;

  switch (message->type) {
    case RDC_MESSAGE_MOVE: return CMD_MOVE;
    case RDC_MESSAGE_ROTATE: return CMD_ROTATE;
    case RDC_MESSAGE_MECHANISM_OPEN: return CMD_MECHANISM_OPEN;
    case RDC_MESSAGE_MECHANISM_CLOSE: return CMD_MECHANISM_CLOSE;
    case RDC_MESSAGE_MECHANISM_STOP: return CMD_MECHANISM_STOP;
    default: return CMD_NONE;
  }
}

static const char *cmd_name(cmd_t command)
{
  switch (command) {
    case CMD_MOVE: return "M";
    case CMD_ROTATE: return "R";
    case CMD_MECHANISM_OPEN: return "GO";
    case CMD_MECHANISM_CLOSE: return "GC";
    case CMD_MECHANISM_STOP: return "GS";
    default: return "WAIT";
  }
}

static void send_ack(void)
{
  char frame[RDC_PROTOCOL_MAX_FRAME];
  const size_t length = rdc_encode_ack(frame, sizeof(frame), ack_sequence++, 0U);
  if ((length == 0U) || (HAL_UART_Transmit(&huart1, (uint8_t *)frame,
      (uint16_t)length, 20U) != HAL_OK)) {
    ++ack_errors;
  }
}

static void ratio_text(char *out, size_t capacity, int16_t ratio)
{
  const unsigned magnitude = (unsigned)((ratio < 0) ? -ratio : ratio);
  (void)snprintf(out, capacity, "%c%u.%02u", (ratio < 0) ? '-' : '+',
                 magnitude / 100U, magnitude % 100U);
}

static void reset_motion_average(void)
{
  memset(move_history, 0, sizeof(move_history));
  memset(rotation_history, 0, sizeof(rotation_history));
  move_history_count = 0U;
  move_history_next = 0U;
  rotation_history_count = 0U;
  rotation_history_next = 0U;
  move_x_centi = 0;
  move_y_centi = 0;
  rotation_z_centi = 0;
}

static void add_move_sample(int16_t x, int16_t y)
{
  move_history[move_history_next][0] = x;
  move_history[move_history_next][1] = y;
  move_history_next = (uint8_t)((move_history_next + 1U) % RDC_MOTION_AVG_SAMPLES);
  if (move_history_count < RDC_MOTION_AVG_SAMPLES) ++move_history_count;

  int32_t sum_x = 0, sum_y = 0;
  for (uint8_t i = 0U; i < move_history_count; ++i) {
    sum_x += move_history[i][0];
    sum_y += move_history[i][1];
  }
  move_x_centi = (int16_t)(sum_x / (int32_t)move_history_count);
  move_y_centi = (int16_t)(sum_y / (int32_t)move_history_count);
}

static void add_rotation_sample(int16_t z)
{
  rotation_history[rotation_history_next] = z;
  rotation_history_next = (uint8_t)((rotation_history_next + 1U) % RDC_MOTION_AVG_SAMPLES);
  if (rotation_history_count < RDC_MOTION_AVG_SAMPLES) ++rotation_history_count;

  int32_t sum = 0;
  for (uint8_t i = 0U; i < rotation_history_count; ++i) sum += rotation_history[i];
  rotation_z_centi = (int16_t)(sum / (int32_t)rotation_history_count);
}

static void draw_progress_screen(uint32_t now)
{
  const uint32_t age = have_valid_command ? now - last_valid_command_tick : 0U;
  const char *link_state = !have_valid_command ? "WAIT" :
                           (age <= RDC_LINK_TIMEOUT_MS ? "UP" : "STALE");
  char x_text[8], y_text[8], z_text[8];
  ratio_text(x_text, sizeof(x_text), move_x_centi);
  ratio_text(y_text, sizeof(y_text), move_y_centi);
  ratio_text(z_text, sizeof(z_text), rotation_z_centi);

  tft_prints(1, 0, "RDC MAINBOARD");
  tft_prints(1, 1, "LINK:%s", link_state);
  if (have_valid_command) tft_prints(1, 2, "AGE:%4ums", (unsigned)age);
  else                    tft_prints(1, 2, "AGE:----");
  tft_prints(1, 3, "MOVE X:%s", x_text);
  tft_prints(1, 4, "MOVE Y:%s", y_text);
  tft_prints(1, 5, "ROT Z:%s", z_text);
  tft_prints(1, 6, "MECH:%s", mechanism_command == RDC_MESSAGE_MECHANISM_OPEN ? "OPEN" :
             mechanism_command == RDC_MESSAGE_MECHANISM_CLOSE ? "CLOSE" : "STOP");
  if (have_valid_command) {
    tft_prints(1, 7, "CMD:%s A:%u", cmd_name(last_command),
               (unsigned)(uint16_t)(ack_sequence - 1U));
  } else {
    tft_prints(1, 7, "CMD:WAIT ACK:--");
  }
  tft_prints(1, 8, "OK:%u BAD:%u", (unsigned)command_ok, (unsigned)command_bad);
  tft_prints(1, 9, "U:%u O:%u D:%u A:%u", (unsigned)bt_uart_errors,
             (unsigned)bt_overflows, (unsigned)bt_dropped, (unsigned)ack_errors);
  tft_prints(1, 10, "MOTION AVG:%u", (unsigned)RDC_MOTION_AVG_SAMPLES);
  tft_update(0U);
}

static void mainboard_app_init(void)
{
  tft_init(PIN_ON_TOP, BLACK, WHITE, CYAN, DARK_GREY);
  tft_force_clear();
  tft_set_text_color(WHITE);
  tft_set_bg_color(BLACK);
  bt_rx_start();
  next_display_tick = HAL_GetTick();
}

static void mainboard_app_step(void)
{
  const uint32_t now = HAL_GetTick();
  char line[BT_LINE_MAX];
  while (bt_rx_get_line(line)) {
    RdcMessage message;
    const cmd_t command = bt_parse(line, &message);
    last_command = command;
    if (command == CMD_NONE) {
      ++command_bad;
      continue;
    }

    ++command_ok;
    have_valid_command = 1U;
    link_timed_out = 0U;
    last_valid_command_tick = now;
    switch (command) {
      case CMD_MOVE:
        add_move_sample(message.ratio_x_centi, message.ratio_y_centi);
        break;
      case CMD_ROTATE:
        add_rotation_sample(message.ratio_z_centi);
        break;
      case CMD_MECHANISM_OPEN:
        mechanism_command = RDC_MESSAGE_MECHANISM_OPEN;
        break;
      case CMD_MECHANISM_CLOSE:
        mechanism_command = RDC_MESSAGE_MECHANISM_CLOSE;
        break;
      case CMD_MECHANISM_STOP:
        mechanism_command = RDC_MESSAGE_MECHANISM_STOP;
        break;
      default:
        break;
    }
    send_ack();
  }

  if (have_valid_command && ((now - last_valid_command_tick) > RDC_LINK_TIMEOUT_MS)) {
    mechanism_command = RDC_MESSAGE_MECHANISM_STOP;
    if (!link_timed_out) {
      reset_motion_average();
      link_timed_out = 1U;
    }
  }
  if ((int32_t)(now - next_display_tick) >= 0) {
    draw_progress_screen(now);
    next_display_tick = now + RDC_DISPLAY_PERIOD_MS;
  }
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  mainboard_app_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    mainboard_app_step();
    HAL_Delay(1U);
  /* USER CODE END 3 */
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  __disable_irq();
  while (1) {
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
