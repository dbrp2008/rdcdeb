#!/usr/bin/env python3
"""Check tracked pin configuration, not a substitute for CubeMX regeneration."""
from pathlib import Path
import re

root = Path(__file__).resolve().parent.parent
properties = {}
for line in (root / "MainBoard.ioc").read_text().splitlines():
    if line and not line.startswith("#") and "=" in line:
        key, value = line.split("=", 1)
        assert key not in properties, f"duplicate IOC key {key}"
        properties[key] = value
expected = {
    "PB8.GPIO_Label": "US3_ECHO", "PB8.Signal": "GPIO_Input", "PB8.GPIO_PuPd": "GPIO_PULLDOWN",
    "PB9.GPIO_Label": "US3_TRIG", "PB9.Signal": "GPIO_Output",
    "PA1.GPIO_Label": "US1_TRIG", "PA1.Signal": "GPIO_Output", "PA0-WKUP.GPIO_Label": "US1_ECHO",
    "PA0-WKUP.Signal": "GPIO_Input", "PB10.Signal": "GPIO_Input",
    "PB10.GPIO_Label": "US2_ECHO", "PB11.Signal": "GPIO_Output", "PB11.GPIO_Label": "US2_TRIG",
    "TIM2.Prescaler": "83", "TIM2.Period": "4294967295",
    "VP_TIM2_VS_ClockSourceINT.Signal": "TIM2_VS_ClockSourceINT",
}
for key, value in expected.items():
    assert properties.get(key) == value, (key, properties.get(key))
pins = [properties[f"Mcu.Pin{i}"] for i in range(int(properties["Mcu.PinsNb"]))]
assert len(pins) == len(set(pins)) and "PB8" in pins and "PB9" in pins
main = (root / "Core/Inc/main.h").read_text()
for symbol, value in {"US1_TRIG_Pin": "GPIO_PIN_1", "US1_TRIG_GPIO_Port": "GPIOA",
                      "US1_ECHO_Pin": "GPIO_PIN_0", "US1_ECHO_GPIO_Port": "GPIOA",
                      "US2_TRIG_Pin": "GPIO_PIN_11", "US2_TRIG_GPIO_Port": "GPIOB",
                      "US2_ECHO_Pin": "GPIO_PIN_10", "US2_ECHO_GPIO_Port": "GPIOB",
                      "US3_ECHO_Pin": "GPIO_PIN_8", "US3_ECHO_GPIO_Port": "GPIOB",
                      "US3_TRIG_Pin": "GPIO_PIN_9", "US3_TRIG_GPIO_Port": "GPIOB"}.items():
    assert re.search(rf"^#define {symbol} {value}$", main, re.MULTILINE), symbol
assert properties["RCC.APB1TimFreq_Value"] == "84000000"
assert "TIM2.Channel-Input_Capture1_from_TI1" not in properties
assert "TIM2.Channel-Input_Capture3_from_TI3" not in properties
assert "NVIC.TIM2_IRQn" not in properties
source = (root / "Core/Src/tim.c").read_text()
assert "htim2.Init.Prescaler = 83;" in source and "htim2.Init.Period = 4294967295;" in source
assert "HAL_TIM_IC_Init(&htim2)" not in source and "__HAL_RCC_TIM2_CLK_ENABLE();" in source
print("US1/2/3 pins and shared 1 MHz TIM2 IOC/source checks PASS (not CubeMX validation)")
