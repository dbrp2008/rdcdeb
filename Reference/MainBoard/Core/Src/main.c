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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tr_auto_app.h"
#include <stdlib.h>
#include "lcd/lcd.h"
#include "lcd/lcd_graphics.h"
#include "robotics_logo.h"
#include "elcp_motor/elcp_motor.h"
#include "tr_drive.h"
#include "tr_sensors.h"
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
static void tft_signed(uint8_t x, uint8_t y, int v) { /* this tft printf has no %+d: draw the sign separately */
  tft_printc(x, y, v < 0 ? "-" : "+");
  tft_prints((uint8_t)(x + 1u), y, "%3d", v < 0 ? -v : v);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  // Starts CAN (RM motors, targets 0) and UART receive for Controller commands
  TRDrive_Init();
  TRSensors_Init(); /* left IR PB10, right IR PB11, ultrasonic PA0 / PA1 */

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
    TRSensors_Update();

    if (TRSensors_PageActive()) { /* BTN1: sensors page instead of the drive page */
      TRSensors_DrawPage();
      tft_update(TRAutoApp_Running() ? 300 : 50); /* fewer screen updates while the routine drives: they block the loop */
      continue;
    }

    TRDrive_Status st;
    TRDrive_GetStatus(&st);

    tft_prints(0, 0, "TR DRIVE");
#if TR_BENCH_TEST == 1
    tft_prints(0, 1, "BENCH:DRIVE 1/2");
#elif TR_BENCH_TEST == 2
    tft_prints(0, 1, "BENCH:ARM 1/2");
#elif TR_BENCH_TEST == 3
    tft_prints(0, 1, "BENCH DRV+ARM");
#else
    if (st.link_ok)
      tft_prints(0, 1, "LINK: OK  ");
    else
      tft_prints(0, 1, "LINK: LOST");
#endif
    if (st.side_state != 0U) { /* sideways test (Controller buttons 2 / 3): encoder sideways mm, forward drift, heading */
      static const char *const side_text[] = {"-", "R", "D", "A", "N"};
      const int sd = st.side_mm, fd = st.side_fwd_mm, hd = st.side_hdg_deg;
      tft_prints(0, 2, "SD%c%3d F%c%2d H%c%d%s", sd < 0 ? '-' : '+', abs(sd) % 1000, fd < 0 ? '-' : '+', abs(fd) % 100,
                 hd < 0 ? '-' : '+', abs(hd) % 10, side_text[st.side_state <= 4 ? st.side_state : 0]);
    } else {
      tft_prints(0, 2, "X%-4dY%-4dR%-4d", (int)st.move_x_pct, (int)st.move_y_pct, (int)st.rot_pct);
    }
    {
      static const char *const turn_text[] = {"-", "R", "D", "A", "N"}; /* idle, running, done, abort, no motion */
      tft_printc(0, 3, "HDG");
      tft_signed(3, 3, (int)st.heading_deg);
      tft_printc(8, 3, st.heading_mode ? "HEAD" : "NORM");
      tft_printc(13, 3, turn_text[st.turn_state <= 4 ? st.turn_state : 0]);
    }
    tft_prints(0, 8, "C1:%02X C2:%02X", (unsigned)st.can1_ids, (unsigned)st.can2_ids);
    tft_prints(0, 4, "ACT/TGT  TEMP  ");
    tft_prints(0, 5, "FL %-4d/%-4d T%02d", (int)st.actual_rpm[TR_WHEEL_FL], (int)st.target_rpm[TR_WHEEL_FL], (int)st.temp_c[TR_WHEEL_FL]);
    tft_prints(0, 6, "FR %-4d/%-4d T%02d", (int)st.actual_rpm[TR_WHEEL_FR], (int)st.target_rpm[TR_WHEEL_FR], (int)st.temp_c[TR_WHEEL_FR]);
    tft_prints(0, 7, "B  %-4d/%-4d T%02d", (int)st.actual_rpm[TR_WHEEL_B], (int)st.target_rpm[TR_WHEEL_B], (int)st.temp_c[TR_WHEEL_B]);
    tft_prints(0, 9, "SPD%d ARM%-2d %4d", (int)st.speed_level, (int)st.a, (int)st.arm_us); /* arm_us = servo pulse in microseconds */
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
