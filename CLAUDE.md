# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for a PID line-follower robot, plus a wireless tuning bridge:

- `LineFollowerRobot/` — STM32CubeIDE project for an **STM32F401CCU6** ("Black Pill"), C + ST HAL. The robot firmware.
- `ESP32C3_Tuner/ESP32C3_Tuner.ino` — Arduino sketch for an ESP32-C3. It creates a Wi-Fi AP (`LineFollower` / `seguidor123`, page at `http://192.168.4.1`) with a web page that sends ASCII commands to the STM32 over UART. It also relays USB Serial to the STM32 and saves PID/speed in `Preferences`, re-sending them when the STM32 boots (`HELLO LINEFOLLOWER`).
- `PINOUT.md` / `PINOUT.csv` — pin map, wiring and the full UART protocol reference. **Read it before touching pins, timers or the protocol.**
- `docs/*.pdf` — team documentation (technical doc, code development, team organization), in Portuguese.
- `.metadata/` — Eclipse/CubeIDE workspace state. It's noise: don't edit it and keep it out of commits unless asked.

Code comments, identifiers (`vel_base`, `Motor_Freio`, `STATE_SEGUINDO_LINHA`) and commit messages are in **Portuguese without accents**. Match that.

## Build / flash

- STM32: open the workspace in STM32CubeIDE (installed at `C:\ST\STM32CubeIDE_1.19.0`), build the `Debug` configuration, and flash/debug over ST-Link with `LineFollowerRobot/LineFollowerRobot Debug.launch`. The IDE generates `Debug/makefile`, so `make -j -C LineFollowerRobot/Debug all` works from a shell only if CubeIDE's `arm-none-eabi-` toolchain and `make` are on PATH. `Debug/` and `Release/` are gitignored.
- ESP32: Arduino IDE, board "ESP32C3 Dev Module" (or the matching Super Mini). Set Tools > USB CDC On Boot > Enabled to use the Serial Monitor (115200, Newline). It needs no extra libraries. CLI: `arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc ESP32C3_Tuner` (upload with `-u -p COM3`). The sketch sends an `ESP OK ...` heartbeat every 2 s (USB Serial, LED, top of the page).
- There are no automated tests. You verify on hardware through the UART protocol (`SENS`, `STATUS`, `MOTOR l r ms`, `CAL START`/`CAL STOP`), sent from the ESP web page or its Serial Monitor.

## CubeMX-generated code

`LineFollowerRobot.ioc` generates `main.c`, `adc.c`, `tim.c`, `gpio.c`, `dma.c`, `usart.c`, `stm32f4xx_it.c` and the `*_msp.c` files. In those files, put hand-written code **only between `/* USER CODE BEGIN x */` and `/* USER CODE END x */`**, or regeneration will delete it. `comm.c`, `control.c`, `motor.c` and `robot_config.h` are fully hand-written.

USART1 is driven at register level by `comm.c`, which defines its own `USART1_IRQHandler` with RX/TX ring buffers. If USART1 is ever enabled in CubeMX in a way that generates a handler, remove the one in `comm.c` (see `comm.h`).

## Runtime architecture

- **Sensors**: ADC1 scans IN1–IN8 (PA1–PA7, PB0 = QTR-8A OUT1–OUT8) continuously and DMA2_Stream0 copies the readings in circular mode into `line_sensor_raw[8]`. The DMA HT/TC interrupts are deliberately disabled (they starved the loop and UART). `Sensors_CheckADC()` restarts the ADC on overrun and sends `EVT ADC RESTART`.
- **Tick**: the TIM2 interrupt sets `tick_flag` at 500 Hz (2 ms) in `HAL_TIM_PeriodElapsedCallback` (`main.c`). All control work runs in the `while(1)` loop in `main.c`, not in the interrupt: `Comm_Process()` and the ADC check run every pass, and the state machine runs once per tick.
- **State machine** (`RobotState_t` in `control.h`, shared with `comm.c`): INIT → PARADO (it boots stopped and waits for `RUN` unless `AUTO_START_DELAY_MS` > 0), SEGUINDO_LINHA / LINHA_PERDIDA, TESTE_MOTOR, CALIBRANDO, PARADO_SEGURANCA. `comm.c` changes `robot_state` in response to commands. The link watchdog (`WD`) stops the robot whenever it's moving and no line has arrived in time.
- **Control** (`control.c`): sensors are normalized to 0..1000 using min/max calibration. Position is a weighted average from 0 to 7000 (center 3500), and `error = 3500 − pos`. Losing the line **never stops the robot**: `Linha_Erro` returns either the last valid error or ±`ERRO_MAX` (decided by `MARGEM_LADO`). LINHA_PERDIDA is only reported in `STATUS`. Wheel mix: the outer wheel gets `vel_base`, and the inner wheel gets `vel_base − 2·|pid|`, clamped at 0 so it never reverses. The result then goes through the slew limit (`SLEW_SUBIDA`/`SLEW_DESCIDA`).
- **Motors** (`motor.c`): a DRV8833 with both inputs of each bridge on PWM (left = TIM3 CH1/CH2 on PB4/PB5, right = TIM4 CH1/CH2 on PB6/PB7, 20 kHz). `MOTOR_DECAY_LENTO` picks slow or fast decay. Commands use the signed "U" scale −1000..1000. The CCR value is computed from the timer's ARR at runtime. Dead-zone compensation maps |u| onto `DZ..U_MAX`. `Motor_ComandoBruto` skips compensation, `Motor_Freio` brakes and `Motor_Solto` lets the wheels coast.
- **Tunables**: `Core/Inc/robot_config.h` holds the U scale, pin mapping, inversion flags, dead zone, slew, line threshold, lost-line margin, `SENSOR_IGNORAR_MASK` (dead sensors, currently S0/PA1: normalized forced to 0, center stays 3500), and `MODO_MEDIR_DZ` (when set, `RUN` runs a dead-zone measurement ramp instead of line following). The default PID gains are in the `Control_Init(&pid, kp, ki, kd, 1000.0f)` call in `main.c`. The default calibration is `SENSOR_MIN_DEFAULT`/`SENSOR_MAX_DEFAULT`.

Gains and calibration set over UART live only in STM32 RAM. Values that should persist go into `main.c` / the defaults above.

## Protocol changes

The ASCII protocol is implemented in `comm.c` (its header comment lists the commands), consumed by the web page in `ESP32C3_Tuner.ino`, and documented in `PINOUT.md`. A protocol change must update all three. Known drift: `EVT LOST STOP` is still listed in the docs, but the current firmware never stops when it loses the line.
