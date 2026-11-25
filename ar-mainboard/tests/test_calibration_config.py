#!/usr/bin/env python3
"""Check drive/bench configuration and nominal PWM timing, not physical hardware."""
from pathlib import Path
import re
import hashlib
import json
root = Path(__file__).resolve().parent.parent
props = {}
for line in (root / "MainBoard.ioc").read_text().splitlines():
    if line and not line.startswith("#") and "=" in line:
        k, v = line.split("=", 1)
        assert k not in props, k
        props[k] = v
expected = {
    "PB14.Signal": "S_TIM12_CH1", "PC7.Signal": "S_TIM8_CH2",
    "PC6.Signal": "GPIO_Output", "PC9.Signal": "GPIO_Output",
    "PC6.GPIO_ModeDefaultOutputPP": "GPIO_MODE_OUTPUT_OD",
    "PC9.GPIO_ModeDefaultOutputPP": "GPIO_MODE_OUTPUT_OD",
    "PB15.Signal": "S_TIM12_CH2", "PC8.Signal": "S_TIM8_CH3", "PB15.GPIO_Label": "ELCP_J1_FB", "PC8.GPIO_Label": "ELCP_J3_FB",
    "SH.S_TIM12_CH2.0": "TIM12_CH2,Input_Capture2_from_TI2", "SH.S_TIM8_CH3.0": "TIM8_CH3,Input_Capture3_from_TI3",
    "TIM12.ICFilter_CH2": "15", "TIM8.ICFilter_CH3": "15",
    "TIM12.ClockDivision": "TIM_CLOCKDIVISION_DIV4", "TIM8.ClockDivision": "TIM_CLOCKDIVISION_DIV4",
    "PB4.GPIO_Label": "IR_L2", "PB4.Signal": "GPXTI4",
    "PB6.GPIO_Label": "IR_R2", "PB6.Signal": "GPXTI6",
    "PB15.GPIO_PuPd": "GPIO_PULLUP", "PC8.GPIO_PuPd": "GPIO_PULLUP",
    "TIM12.AutoReloadPreload": "TIM_AUTORELOAD_PRELOAD_ENABLE",
    "TIM8.AutoReloadPreload": "TIM_AUTORELOAD_PRELOAD_ENABLE",
    "TIM8.OCPolarity_2": "TIM_OCPOLARITY_HIGH",
    "TIM12.Prescaler": "7", "TIM12.Period": "1049",
    "TIM8.Prescaler": "15", "TIM8.Period": "1049",
}
for k, v in expected.items(): assert props.get(k) == v, (k, props.get(k))
ips = [props[f"Mcu.IP{i}"] for i in range(int(props["Mcu.IPNb"]))]
assert "TIM8" in ips and "TIM12" in ips
# digital line IRs (Core/Inc/ir_mode.h IR_DIGITAL 1): plain inputs, no pull, and no ADC1 left in the project
for pin, label in (("PA2", "IR_LEFT"), ("PC2", "IR_MID"), ("PC3", "IR_RIGHT")):
    assert props.get(f"{pin}.Signal") == "GPIO_Input" and props.get(f"{pin}.GPIO_Label") == label, pin
    assert f"{pin}.GPIO_PuPd" not in props, pin
assert "ADC1" not in ips and not any(k.startswith("ADC1.") for k in props)
assert "#define IR_DIGITAL 1" in (root / "Core/Inc/ir_mode.h").read_text()
assert not (root / "Core/Src/adc.c").exists() and "adc.h" not in (root / "Core/Src/main.c").read_text()
pins = [props[f"Mcu.Pin{i}"] for i in range(int(props["Mcu.PinsNb"]))]
assert len(pins) == len(set(pins))
header = (root / "Core/Inc/main.h").read_text()
for name, line, vector in (("IR_L2", 4, "EXTI4"), ("IR_R2", 6, "EXTI9_5")):
    assert f"PB{line}" in pins and props[f"SH.GPXTI{line}.0"] == f"GPIO_EXTI{line}"
    assert props["NVIC." + vector + "_IRQn"].split("\\:")[0] == "true"
    for suffix, value in (("Pin", f"GPIO_PIN_{line}"), ("GPIO_Port", "GPIOB"), ("EXTI_IRQn", vector + "_IRQn")):
        assert re.search(rf"^#define {name}_{suffix} {value}$", header, re.M)
for p in list((root / "Core").rglob("*.c")) + list((root / "Core").rglob("*.h")) + [root / "CMakeLists.txt"]:
    text = p.read_text()
    assert not re.search(r"\b(?:AR_Nav_\w*|motor_sequence)\b", text), p
    if p.name not in ("elcp_motor.c", "elcp_motor.h", "j3_rotation_run.c", "calibration_app.c"):
        assert "ELCP_Motor_SetSpeed(" not in text and "ELCP_Motor_Stop(" not in text, p
    if p.name != "elcp_motor.c":
        assert "HAL_TIM_PWM_Start(" not in text, p
# Exact bytes from main/reference commit a6104b7, not just matching API names.
reference_hashes = {
    "Core/Src/elcp_motor/elcp_motor.c": "7c06cd4289eaa487ecbe89ee5fcf6afeb581c540755f4ead2d1f083e26a6c3b0",
    "Core/Inc/elcp_motor/elcp_motor.h": "3966c238c377919dbf71ca92effc67368317b52148ab154e89d62665647e02e7",
    "Core/Inc/elcp_motor/README.md": "6f1114a1310cb94314299f361a248398fd7480cde2db964829e71f9e9ed46dbc",
}
for path, digest in reference_hashes.items():
    assert hashlib.sha256((root / path).read_bytes()).hexdigest() == digest, path
it = (root / "Core/Src/stm32f4xx_it.c").read_text()
# encoder FB: timer input capture (filter 15), owned by enc_capture.c - no EXTI, and CubeMX must not generate the vectors
for pin in ("ELCP_J1_FB_Pin", "ELCP_J3_FB_Pin"):
    assert f"HAL_GPIO_EXTI_IRQHandler({pin});" not in it
assert "EXTI15_10" not in it and "TIM8_CC_IRQHandler" not in it and "TIM8_BRK_TIM12_IRQHandler" not in it
assert not any(k.startswith(("NVIC.TIM8_CC", "NVIC.TIM8_BRK_TIM12", "NVIC.EXTI15_10")) for k in props)
cap = (root / "Core/Src/enc_capture.c").read_text()
for need in ("void TIM8_BRK_TIM12_IRQHandler(void)", "void TIM8_CC_IRQHandler(void)", "HAL_GPIO_EXTI_Callback(ELCP_J1_FB_Pin)",
             "HAL_GPIO_EXTI_Callback(ELCP_J3_FB_Pin)", "TIM_CCMR1_IC2F", "TIM_CCMR2_IC3F", "TIM_CR1_CKD_1"):
    assert need in cap, need
assert "EncCapture_Start()" in (root / "Core/Src/elcp_zero_pwm.c").read_text()
assert "EncCapture_Start()" in (root / "Core/Src/encoder_bench.c").read_text()
tim = (root / "Core/Src/tim.c").read_text()
assert tim.count("sConfigIC.ICFilter = 15;") == 2 and tim.count("TIM_CLOCKDIVISION_DIV4") == 2
assert "RotationEstimate_RecordIRQ" not in it
cmake = (root / "CMakeLists.txt").read_text()
assert "Core/Src/rotation_estimate.c" not in cmake
assert "Core/Src/j3_rotation_run.c" not in cmake

driver = (root / "Core/Src/elcp_motor/elcp_motor.c").read_text()
for motor, timer, channel in (("J1", "htim12", "TIM_CHANNEL_1"), ("J3", "htim8", "TIM_CHANNEL_2")):
    match = re.search(rf"static ELCP_Motor_t motor_{motor} = \{{(.*?)\}};", driver, re.S)
    assert match, motor
    block = match.group(1)
    for required in (f".htim = &{timer}", f".tim_channel = {channel}", f".dir_pin = ELCP_{motor}_DIR_Pin", f".fb_pin = ELCP_{motor}_FB_Pin", ".invert_dir = false", ".invert_pwm = false"):
        assert required in block, (motor, required)
for symbol in ("ELCP_Motor_Init", "ELCP_Motor_SetSpeed", "ELCP_Motor_Stop", "ELCP_Motor_GetState", "ELCP_Motor_SetPulsesPerRev", "ELCP_Motor_FB_IRQHandler", "ELCP_Motor_UpdatePPS"):
    assert re.search(rf"\b{symbol}\s*\(", driver), symbol
main = (root / "Core/Src/main.c").read_text()
tim = (root / "Core/Src/tim.c").read_text()

# The IOC is editable; the generated C is what gets linked. Verify the nominal
# clock from the board's HSE through each APB timer, not just equal register values.
def assigned(source, field):
    values = re.findall(rf"^[ \t]*{re.escape(field)}\s*=\s*(\w+)\s*;", source, re.M)
    assert len(values) == 1, (field, values)
    return values[0]

pll, rcc = "RCC_OscInitStruct.PLL.", "RCC_ClkInitStruct."
assert assigned(main, pll + "PLLSource") == props["RCC.PLLSourceVirtual"] == "RCC_PLLSOURCE_HSE"
assert assigned(main, rcc + "SYSCLKSource") == props["RCC.SYSCLKSource"] == "RCC_SYSCLKSOURCE_PLLCLK"
m, n = (int(assigned(main, pll + k)) for k in ("PLLM", "PLLN"))
assert (m, n) == (int(props["RCC.PLLM"]), int(props["RCC.PLLN"]))
pll_p = int(assigned(main, pll + "PLLP").split("_DIV")[1])
ahb_div = int(assigned(main, rcc + "AHBCLKDivider").split("_DIV")[1])
hclk = int(props["RCC.HCLKFreq_Value"])
assert int(props["RCC.HSE_VALUE"]) * n == hclk * m * pll_p * ahb_div, "HSE/PLL/AHB do not give the IOC HCLK"
for timer, bus in ((8, "APB2"), (12, "APB1")):
    bus_div = assigned(main, rcc + bus + "CLKDivider")
    assert bus_div == props[f"RCC.{bus}CLKDivider"], (bus, bus_div)
    div = int(bus_div.split("_DIV")[1])
    pclk = int(props[f"RCC.{bus}Freq_Value"])
    assert pclk * div == hclk, (bus, pclk, div, hclk)
    timer_hz = pclk if div == 1 else 2 * pclk  # STM32F405 timer clock on a divided APB
    assert timer_hz == int(props[f"RCC.{bus}TimFreq_Value"]), (bus, timer_hz)
    init = f"htim{timer}.Init."
    ioc_mode = props.get(f"TIM{timer}.CounterMode", "TIM_COUNTERMODE_UP")
    assert assigned(tim, init + "CounterMode") == ioc_mode == "TIM_COUNTERMODE_UP", f"TIM{timer} must count up"
    fields = ("Prescaler", "Period")
    psc, arr = (int(assigned(tim, init + k)) for k in fields)
    ioc_settings = tuple(int(props[f"TIM{timer}.{k}"]) for k in fields)
    assert (psc, arr) == ioc_settings, f"TIM{timer}: C has {(psc, arr)}, IOC has {ioc_settings}"
    assert timer_hz == 10_000 * (psc + 1) * (arr + 1), f"TIM{timer} is not 10 kHz"
print("Motor PWM: J3 TIM8 and J1 TIM12 each give a 100 us period in IOC and generated C PASS")

# PWM timer channels exist before the app starts; the app orders zero before the first TFT delay.
for init in ("MX_GPIO_Init();", "MX_TIM8_Init();", "MX_TIM12_Init();"):
    assert main.index(init) < main.index("CalibrationApp_Init();")
assert "tft_init(" not in main
assert "Core/Src/elcp_zero_pwm.c" in (root / "CMakeLists.txt").read_text()
assert "Core/Src/elcp_motor/elcp_motor.c" in (root / "CMakeLists.txt").read_text()
app = (root / "Core/Src/calibration_app.c").read_text()
for old in ("US2_Measure(", "US3_Measure(", "J3Run_", "RotationEstimate_"):
    assert old not in app, old
for required in ("IR_Sensor_Init()", "IR_ReadAll(", "ELCP_ZeroPWM_Init()", "US1_Measure()", "US2_Init()", "btn_read(BTN1)", "btn_read(BTN2)", "ELCP_Motor_Stop(MOTOR_J1)", "ELCP_Motor_Stop(MOTOR_J3)"):
    assert required in app, required
# Setup and drive stay together in the normal image. Retired switches may
# reject a stale cache but may not select another application.
assert not re.search(r"(?:option|if)\(\s*AR_(?:IR_BENCH|ENABLE_F[123]|F1_DRY_RUN)\b", cmake)
assert "AR_IR_BENCH" in cmake and 'message(FATAL_ERROR "${retired} is retired.' in cmake
assert not re.search(r"target_compile_definitions\([^)]*AR_(?:ENABLE_F[123]|IR_BENCH)", cmake, re.S)
for name in ("side_ir", "ir_setup", "calibration_app", "drive_plan", "function1", "function2", "function3", "us2_sensor"):
    assert cmake.count(f"Core/Src/{name}.c") == 1, name
assert "calibration_app_ir.c" not in cmake
# Set the languages on project() itself: enabling C/ASM later does not undo implicit CXX.
project = re.search(r"^project\(([^)]*)\)", cmake, re.M)
assert project and project[1].split() == ["${CMAKE_PROJECT_NAME}", "LANGUAGES", "C", "ASM"]
assert cmake.index("set(CMAKE_TOOLCHAIN_FILE") < project.start()
assert not re.search(r"^\s*enable_language\([^)]*\bCXX\b", cmake, re.M | re.I)
assert "drive_probe" not in cmake
assert re.search(r"set_property\(SOURCE Core/Src/elcp_motor/elcp_motor.c APPEND PROPERTY\s+"
                 r"COMPILE_DEFINITIONS HAL_GPIO_EXTI_Callback=ELCP_EncoderEXTI_Callback\)", cmake)
for path in ("Core/Src/calibration_app.c", "Core/Inc/calibration_app.h", "Core/Src/drive_plan.c", "Core/Inc/drive_plan.h"):
    assert not re.search(r"AR_ENABLE_F[123]", (root / path).read_text()), path
presets = json.loads((root / "CMakePresets.json").read_text())
configs = {p["name"]: p for p in presets["configurePresets"]}
public = {"Debug", "Release", "EncoderBench"}
assert {name for name, p in configs.items() if not p.get("hidden")} == public
assert {p["name"]: p["configurePreset"] for p in presets["buildPresets"]} == {n:n for n in public}
assert configs["Debug"]["inherits"] == configs["Release"]["inherits"] == "default"
assert configs["default"]["cacheVariables"]["AR_IMAGE"] == "Drive"
assert configs["default"]["binaryDir"] == "${sourceDir}/build/${presetName}"
assert configs["EncoderBench"]["inherits"] == "Release"
assert configs["EncoderBench"]["cacheVariables"]["AR_IMAGE"] == "EncoderBench"
assert not re.search(r"AR_(?:ENABLE_F[123]|IR_BENCH|ENCODER_BENCH)", json.dumps(presets))
tasks = json.loads((root / ".vscode/tasks.json").read_text())["tasks"]
by_label = {t["label"]: t for t in tasks}
assert len(by_label) == len(tasks)
flash, both = (f"{action} AR MainBoard (Release)" for action in ("Flash Existing", "Build + Flash"))
encoder_flash = "Build + Flash AR Encoder Bench"
assert {t["label"] for t in tasks if not t.get("hide")} == {flash, both, encoder_flash}
# Neither a hidden helper nor a flash action should take over Ctrl+Shift+B.
for t in tasks:
    group = t.get("group")
    assert (group.get("kind") if isinstance(group, dict) else group) != "build", t["label"]
assert by_label[both]["dependsOrder"] == "sequence"
build, program = (by_label[name] for name in by_label[both]["dependsOn"])
assert not any("preset" in t or "--preset" in t.get("args", []) for t in (build, program, by_label[both]))
assert program == by_label[flash] and not program.get("dependsOn")
assert program["args"] == ["-c", "port=SWD", "-w", "build/Release/MainBoard.elf", "-v", "-rst"]
assert build["hide"] and build["type"] == "process" and build["command"] == "cmake" and not build.get("dependsOn")
assert build["args"] == ["-P", "tools/build_release.cmake"] and (root / build["args"][1]).is_file()
# The visible bench programmer depends on a successful hidden build. Neither
# its dependency nor its ELF path goes through a Release program/flash task.
program_bench = by_label[encoder_flash]
build_bench = by_label[program_bench["dependsOn"]]
assert program_bench["type"] == "process" and program_bench["command"] == program["command"]
assert program_bench["args"] == ["-c", "port=SWD", "-w", "build/EncoderBench/EncoderBench.elf", "-v", "-rst"]
assert build_bench["hide"] and build_bench["type"] == "process" and build_bench["command"] == "cmake"
assert build_bench["args"] == ["-DAR_BUILD_PRESET=EncoderBench", "-P", "tools/build_release.cmake"]
assert not build_bench.get("dependsOn")
limits = (root / "Core/Inc/line_drive.h").read_text()
assert re.search(r"^#define DRIVE_US1_STOP_MM\s+60\b", limits, re.M)
assert re.search(r"^#define DRIVE_US1_FINISH_MM\s+55\b", limits, re.M)
f1 = (root / "Core/Src/function1.c").read_text()
assert "US_STOP_HOLD_MS 1000u" in f1
assert "US_REVERSE_MS 500u" in f1
print("Calibration-first drive, isolated EncoderBench task/preset and reference motor checks PASS (not hardware validation)")
