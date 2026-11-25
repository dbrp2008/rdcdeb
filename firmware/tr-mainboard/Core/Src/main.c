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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd/lcd.h"
#include "lcd/lcd_graphics.h"
#include "robotics_logo.h"
#include "tr_drive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Q status is the HAL enqueue result only; live ESC replies are shown in C1/C2. */
static char can_queue_code(uint8_t init_ok, uint8_t tx_ok, uint32_t tx_failures)
{
  if (init_ok == 0U) return 'I';
  if (tx_failures != 0U) return '!';
  return (tx_ok != 0U) ? 'O' : 'E';
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Starts CAN (RM motors, targets 0) and UART receive for Controller commands
  TRDrive_Init();

  tft_init(PIN_ON_TOP, BLACK, WHITE, CYAN, DARK_GREY);
  tft_prints(0, 0, "TR DRIVE");
  tft_update(0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    TRDrive_Update();

    TRDrive_Status st;
    TRDrive_GetStatus(&st);

    const char can1_code = can_queue_code(st.can1_init_ok, st.can1_tx_ok, st.can1_tx_failures);
#if TR_BACK_ON_CAN2
    const char can2_code = can_queue_code(st.can2_init_ok, st.can2_tx_ok, st.can2_tx_failures);
#else
    const char can2_code = '-';
#endif
    const int fl_current_deci_a = (int)(st.current_a[TR_WHEEL_FL] * 10.0f);
    const int fr_current_deci_a = (int)(st.current_a[TR_WHEEL_FR] * 10.0f);
    const int b_current_deci_a = (int)(st.current_a[TR_WHEEL_B] * 10.0f);

    tft_prints(0, 0, "TR DRIVE");
    tft_prints(0, 1, "L:%s Q1:%c Q2:%c", st.link_ok ? "OK" : "--", can1_code, can2_code);
    tft_prints(0, 2, "X%-4dY%-4dR%-4d", (int)st.move_x_pct, (int)st.move_y_pct, (int)st.rot_pct);
    tft_prints(0, 3, "RX:%-5u ERR:%-3u", (unsigned)st.rx_bytes, (unsigned)st.uart_err);
    tft_prints(0, 4, "A/T RPM I0.1A");
    tft_prints(0, 5, "FL%4d/%4d I%+03d", (int)st.actual_rpm[TR_WHEEL_FL], (int)st.target_rpm[TR_WHEEL_FL], fl_current_deci_a);
    tft_prints(0, 6, "FR%4d/%4d I%+03d", (int)st.actual_rpm[TR_WHEEL_FR], (int)st.target_rpm[TR_WHEEL_FR], fr_current_deci_a);
    tft_prints(0, 7, "B %4d/%4d I%+03d", (int)st.actual_rpm[TR_WHEEL_B], (int)st.target_rpm[TR_WHEEL_B], b_current_deci_a);
    tft_prints(0, 8, "C1:%02X C2:%02X", (unsigned)st.can1_ids, (unsigned)st.can2_ids);
    tft_prints(0, 9, "ARM:%-2d %-4dus", (int)st.a, (int)st.arm_us);
    tft_prints(0, 10, "H:%s %4d", st.heading_requested ? (st.heading_valid ? "ON  " : "WAIT") : "OFF ", (int)st.heading_deg);
    const char *turn_label = st.turn_state == TR_TURN_RUN ? "RUN" :
                             st.turn_state == TR_TURN_DONE ? "DONE" :
                             st.turn_state == TR_TURN_ABORT ? "ABORT" :
                             st.turn_state == TR_TURN_NOFB ? "NOFB" :
                             st.turn_state == TR_TURN_REJECT ? "REJECT" :
                             st.turn_state == TR_TURN_WRONG_WAY ? "WRONG" : "IDLE";
    tft_prints(0, 11, "TURN:%-6s", turn_label);
    tft_update(50);
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
