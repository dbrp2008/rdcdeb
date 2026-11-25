#include "joystick_calibration.h"

#include "adc.h"
#include "lcd/lcd.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define CENTER_SAMPLE_COUNT       128U
#define SAMPLE_INTERVAL_MS         10U
#define SWEEP_DURATION_MS       15000U
#define EARLY_FINISH_AFTER_MS    4000U
#define MIN_USABLE_SIDE_COUNTS    100U

/* The ADC/DMA engine updates these four halfwords continuously in rank order:
 * J1 X, J1 Y, J2 X, J2 Y. Static lifetime and alignment are required by DMA. */
static volatile uint16_t joystick_dma_samples[4] __attribute__((aligned(4)));

static const char *const axis_names[4] = { "J1X", "J1Y", "J2X", "J2Y" };

typedef struct
{
    uint16_t minimum;
    uint16_t centre;
    uint16_t maximum;
} JoystickAxisCalibration;

static JoystickAxisCalibration calibration[4];

static void clear_and_update(void)
{
    tft_force_clear();
    tft_update(0U);
}

static void wait_until_released(uint8_t button)
{
    while (button == 1U ? btn_read(Button1) : btn_read(Button2))
    {
        HAL_Delay(10U);
    }
}

static void wait_for_exit_button(void)
{
    while (!btn_read(Button1))
    {
        HAL_Delay(10U);
    }
    HAL_Delay(30U);
    wait_until_released(1U);
}

static bool result_is_usable(void)
{
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        const JoystickAxisCalibration *axis = &calibration[i];
        if (!(axis->minimum < axis->centre && axis->centre < axis->maximum))
        {
            return false;
        }
        if (((uint32_t)axis->centre - axis->minimum) < MIN_USABLE_SIDE_COUNTS ||
            ((uint32_t)axis->maximum - axis->centre) < MIN_USABLE_SIDE_COUNTS)
        {
            return false;
        }
    }
    return true;
}

static bool show_results_and_wait(void)
{
    clear_and_update();
    tft_prints(0U, 0U, result_is_usable() ? "CAL OK: NO SAVE" : "CHECK: NO SAVE");

    for (uint32_t i = 0U; i < 4U; ++i)
    {
        const uint8_t row = (uint8_t)(1U + 2U * i);
        tft_prints(0U, row, "%s C:%4u", axis_names[i], (unsigned)calibration[i].centre);
        tft_prints(0U, (uint8_t)(row + 1U), "L:%4u H:%4u",
                   (unsigned)calibration[i].minimum,
                   (unsigned)calibration[i].maximum);
    }
    tft_prints(0U, 9U, "B1 exit B2 retry");
    tft_update(0U);

    for (;;)
    {
        if (btn_read(Button1))
        {
            HAL_Delay(30U);
            wait_until_released(1U);
            return false;
        }
        if (btn_read(Button2))
        {
            HAL_Delay(30U);
            wait_until_released(2U);
            return true;
        }
        HAL_Delay(10U);
    }
}

static bool run_one_calibration(void)
{
    HAL_StatusTypeDef status = HAL_ADCEx_Calibration_Start(&hadc1);
    if (status != HAL_OK)
    {
        goto adc_error;
    }

    status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(uintptr_t)joystick_dma_samples, 4U);
    if (status != HAL_OK)
    {
        goto adc_error;
    }

    /* DMA keeps filling the live array; no completion callback is needed. */
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);

    clear_and_update();
    tft_prints(0U, 0U, "CAL CENTER");
    tft_prints(0U, 2U, "Release sticks");
    tft_prints(0U, 4U, "Sampling 128x");
    tft_update(0U);
    HAL_Delay(500U);

    uint32_t sum[4] = { 0U, 0U, 0U, 0U };
    for (uint32_t sample = 0U; sample < CENTER_SAMPLE_COUNT; ++sample)
    {
        for (uint32_t axis = 0U; axis < 4U; ++axis)
        {
            sum[axis] += joystick_dma_samples[axis];
        }
        if ((sample % 16U) == 0U)
        {
            tft_prints(0U, 6U, "Sample %3u/%3u",
                       (unsigned)(sample + 1U), (unsigned)CENTER_SAMPLE_COUNT);
            tft_update(20U);
        }
        HAL_Delay(SAMPLE_INTERVAL_MS);
    }

    for (uint32_t axis = 0U; axis < 4U; ++axis)
    {
        calibration[axis].centre = (uint16_t)(sum[axis] / CENTER_SAMPLE_COUNT);
        calibration[axis].minimum = calibration[axis].centre;
        calibration[axis].maximum = calibration[axis].centre;
    }

    clear_and_update();
    tft_prints(0U, 0U, "FULL RANGE SWEEP");
    tft_prints(0U, 2U, "Move all sticks");
    tft_prints(0U, 3U, "Hold at each end");
    tft_prints(0U, 5U, "15 sec total");
    tft_prints(0U, 7U, "B1=end after 4s");
    tft_update(0U);

    const uint32_t sweep_start = HAL_GetTick();
    uint32_t last_screen_update = sweep_start;
    while ((uint32_t)(HAL_GetTick() - sweep_start) < SWEEP_DURATION_MS)
    {
        for (uint32_t axis = 0U; axis < 4U; ++axis)
        {
            const uint16_t value = joystick_dma_samples[axis];
            if (value < calibration[axis].minimum) calibration[axis].minimum = value;
            if (value > calibration[axis].maximum) calibration[axis].maximum = value;
        }

        const uint32_t elapsed = (uint32_t)(HAL_GetTick() - sweep_start);
        if (elapsed >= EARLY_FINISH_AFTER_MS && btn_read(Button1))
        {
            HAL_Delay(30U);
            wait_until_released(1U);
            break;
        }
        if ((uint32_t)(HAL_GetTick() - last_screen_update) >= 500U)
        {
            const uint32_t seconds_left = (SWEEP_DURATION_MS - elapsed + 999U) / 1000U;
            tft_prints(0U, 8U, "Remaining:%2u s", (unsigned)seconds_left);
            tft_update(20U);
            last_screen_update = HAL_GetTick();
        }
        HAL_Delay(SAMPLE_INTERVAL_MS);
    }

    (void)HAL_ADC_Stop_DMA(&hadc1);
    return true;

adc_error:
    (void)HAL_ADC_Stop_DMA(&hadc1);
    clear_and_update();
    tft_prints(0U, 0U, "ADC CAL ERROR");
    tft_prints(0U, 2U, "Check ADC/DMA");
    tft_prints(0U, 4U, "B1 to exit");
    tft_update(0U);
    wait_for_exit_button();
    return false;
}

void joystick_calibration_run(void)
{
    clear_and_update();
    tft_prints(0U, 0U, "JOYSTICK CAL");
    tft_prints(0U, 2U, "Center all sticks");
    tft_prints(0U, 4U, "Starting shortly");
    tft_update(0U);
    HAL_Delay(1000U);

    for (;;)
    {
        if (!run_one_calibration())
        {
            clear_and_update();
            return;
        }
        if (!show_results_and_wait())
        {
            /* Remove calibration rows so the normal T3 screen starts clean. */
            clear_and_update();
            return;
        }
    }
}
