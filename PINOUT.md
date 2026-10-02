# LineFollowerRobot – Pinout & Calibration Link

Taken from `LineFollowerRobot.ioc`, `Core/Src/*.c`, and the new `Core/Src/comm.c`.

MCU: **STM32F401CCU6** (UFQFPN48, "Black Pill"), SYSCLK 84 MHz (HSI + PLL), APB1 42 MHz, APB2 84 MHz, timer clocks 84 MHz.

## STM32 pin map

| Pin | Package pin | Function | Peripheral / config | Connects to | Notes |
|---|---|---|---|---|---|
| PA0  | 10 | Line sensor S0 | ADC1_IN0, rank 1 | Sensor bar ch 0 (far edge) | weight 0 |
| PA1  | 11 | Line sensor S1 | ADC1_IN1, rank 2 | Sensor bar ch 1 | weight 1000 |
| PA2  | 12 | Line sensor S2 | ADC1_IN2, rank 3 | Sensor bar ch 2 | weight 2000 |
| PA3  | 13 | Line sensor S3 | ADC1_IN3, rank 4 | Sensor bar ch 3 | weight 3000 |
| PA4  | 14 | Line sensor S4 | ADC1_IN4, rank 5 | Sensor bar ch 4 | weight 4000 |
| PA5  | 15 | Line sensor S5 | ADC1_IN5, rank 6 | Sensor bar ch 5 | weight 5000 |
| PA6  | 16 | Line sensor S6 | ADC1_IN6, rank 7 | Sensor bar ch 6 | weight 6000 |
| PA7  | 17 | Line sensor S7 | ADC1_IN7, rank 8 | Sensor bar ch 7 (far edge) | weight 7000 |
| PB5  | 41 | Motor RIGHT PWM | TIM3_CH2, AF2, PWM1 | DRV8833 **IN1** | 20 kHz |
| PB4  | 40 | Motor RIGHT DIR | GPIO out push-pull (`MOTOR2_DIR`) | DRV8833 **IN2** | LOW = forward (fast decay), HIGH = reverse (slow decay, PWM inverted) |
| PB3  | 39 | Motor LEFT PWM | TIM2_CH2, AF1, PWM1 | DRV8833 **IN3** | 20 kHz |
| PB6  | 42 | Motor LEFT DIR | GPIO out push-pull (`MOTOR_DIR_R`) | DRV8833 **IN4** | same as PB4 |
| PB2  | 20 | *unused* | still GPIO out in CubeMX (`MOTOR1_DIR`) | – | not wired; the left motor's direction is on PB6 |
| **PA9**  | 30 | **UART TX → ESP32** | USART1_TX, AF7 (register-level, `comm.c`) | ESP32-C3 **GPIO6** (RX) | 115200 8N1 · **new** |
| **PA10** | 31 | **UART RX ← ESP32** | USART1_RX, AF7, pull-up | ESP32-C3 **GPIO7** (TX) | 115200 8N1 · **new** |
| PA13 / PA14 | 34 / 37 | SWDIO / SWCLK | Debug | ST-Link | leave free |
| GND | – | Ground | – | ESP32 GND, driver GND, sensors GND | **must be shared** |

Internal (no pin): **TIM4** runs the control loop tick at 84 MHz / 84 / 2000 = **500 Hz (2 ms)**. ADC1 is 12-bit, scans 8 channels continuously, and DMA2_Stream0 copies them in circular mode into `line_sensor_raw[8]`.

Position = weighted average of the normalized sensors, 0..7000, **center = 3500**. `error = 3500 − position`. Motor commands are in the U scale, −1000..1000 (see `Core/Inc/robot_config.h`): outer wheel = base speed, inner wheel = base − 2·|PID|, clamped at 0 (never reverses), then slew-limited (`SLEW_SUBIDA`/`SLEW_DESCIDA`).

## ESP32-C3 ↔ STM32 wiring

| ESP32-C3 | STM32 | Signal |
|---|---|---|
| GPIO7 (TX, `STM_TX_PIN`) | PA10 (USART1_RX) | commands |
| GPIO6 (RX, `STM_RX_PIN`) | PA9 (USART1_TX) | replies |
| GND | GND | common ground |
| 3V3 / 5V | – | power the ESP separately or from the robot's 5 V → ESP 5V pin |

Both sides use 3.3 V logic, so no level shifter is needed. The pins can be changed at the top of `ESP32C3_Tuner.ino`. Avoid GPIO 2, 8 and 9, which are strapping pins.

## Protocol (ASCII, 115200 8N1, one command per line ending in `\n`)

| Command | Reply | What it does |
|---|---|---|
| `PING` | `PONG` | Checks the link |
| `STATUS` | `STATUS <state> KP <kp> KI <ki> KD <kd> SPEED <u>` | state = RUN / LOST / STOP / SAFE_STOP / MOTOR_TEST / CALIB |
| `STOP` | `OK STOP` | Motors off at once and stays stopped |
| `RUN` | `OK RUN` | Resets the PID and the last-seen-side memory and starts line following. **The robot boots stopped and waits for `RUN`** (`AUTO_START_DELAY_MS` in `main.c` changes this) |
| `PID <kp> <ki> <kd>` | `OK PID <kp> <ki> <kd>` | Sets all three gains (resets integrator). Each gain −10000..10000; `nan`/`inf` rejected |
| `KP <v>` / `KI <v>` / `KD <v>` | `OK PID …` | Changes one gain |
| `PID?` | `PID <kp> <ki> <kd>` | Reads the gains |
| `SPEED <0..1000>` | `OK SPEED <u>` | Base speed in U (1000 = 100%) |
| `MOTOR <left> <right> <ms>` | `OK MOTOR …` then later `EVT MOTOR DONE` | Motor test. −1000..1000 (negative = reverse, 0 = brake), with dead-zone compensation, for 1..10000 ms, then brakes |
| `SENS` | `SENS R r0…r7 N n0…n7 POS <0..7000> LINE <1\|0>` | Raw ADC (R), calibrated 0..1000 (N), position used by the PID. `LINE 0` = line lost, POS = edge of the side where it was last seen |
| `CAL START` | `OK CAL START` | Motors off. Records the min/max of each sensor while you slide the robot over the line |
| `CAL STOP` | `CAL MIN m0…m7 MAX M0…M7 [REJ i j …]` | Ends calibration. Sensors whose range is below 200 are **rejected** and keep their previous calibration (listed after `REJ`) |
| `CAL?` | `CAL MIN … MAX … [WARN]` | Reads the current calibration |
| `WD <0..60000>` | `OK WD <ms>` | Link watchdog: if no line arrives for `<ms>` while the robot is moving, it stops and sends `EVT WD STOP`. `0` = off (default) |
| `HB` | *(no reply)* | Heartbeat that only feeds the watchdog |
| anything else | `ERR …` | |

Spontaneous events: `EVT MOTOR DONE`, `EVT LOST STOP` (spun 2 s without finding the line → `SAFE_STOP`), `EVT WD STOP`, `EVT ADC RESTART` (ADC overrun recovered).

At boot the STM32 sends `HELLO LINEFOLLOWER`. The ESP uses it to re-send the PID and speed saved with **"Salvar no ESP"**.

Gains and calibration live in STM32 RAM, so they reset on power-cycle. When you're happy with the values, copy them into `Control_Init(&pid, kp, ki, kd, 1000.0f)` in `main.c` and into `SENSOR_MIN_DEFAULT` / `SENSOR_MAX_DEFAULT`, or into per-sensor arrays.
