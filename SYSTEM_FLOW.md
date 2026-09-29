# Self-Balancing Car — System Flow and Firmware Architecture

## 1. Project scope

This project targets a two-wheel self-balancing platform using an STM32F446RET6 and a GY-ICM20948V2 inertial sensor.

The current firmware baseline provides:

- I2C1 communication with the ICM-20948 on PB6/PB7.
- Accelerometer and gyroscope initialization and burst reading.
- Roll/pitch calculation with gyro bias correction and a complementary filter.
- A fixed 250 Hz IMU sampling schedule.
- Differential motor PWM control through TIM2.
- UART1 diagnostic logging at 115200 baud.
- A `BalanceManager` interface with PID, PD and state-feedback algorithm modes.
- Safety handling for invalid/stale IMU data and excessive tilt.

The `BalanceManager` implementation and the 250 Hz scheduler hook are present in the current source tree. The current application retains its existing automatic balance-enable behavior after a valid IMU sample. FreeRTOS middleware is not installed yet; the current scheduler hook is the HAL timebase loop.

## 2. Hardware and software data flow

```text
GY-ICM20948V2
      │ I2C1: PB6/PB7, 400 kHz
      ▼
ICM20948 driver
      │ calibrated accel + bias-corrected gyro
      ▼
Angle conversion
      │ filtered roll/pitch, gyro rate
      ├──────────────► UART1 diagnostic log
      │
      ▼
      Balance controller (`BalanceManager`)
      │ left/right correction
      ▼
Motor controller
      │ TIM2 CH2/CH3 + direction GPIO
      ▼
Left and right motor driver
```

## 3. MCU peripheral allocation

| Peripheral | Configuration | Responsibility |
|---|---|---|
| RCC | 72 MHz system clock | CPU and peripheral clock source |
| I2C1 | PB6/PB7, 400 kHz Fast mode | ICM-20948 communication |
| TIM2 CH2 | PA1, PWM | Left motor speed |
| TIM2 CH3 | PA2, PWM | Right motor speed |
| GPIO | PA0, PC1, PC2, PC3 | Motor direction/enable |
| USART1 | PA9/PA10, 115200 | Debug and telemetry |
| GPIO | PB2 | Status LED |
| GPIO | PB0, PC13 | User buttons |
| SysTick | HAL time base | Millisecond timing |

## 4. Startup sequence

```text
Reset
  │
  ▼
HAL_Init()
  │
  ▼
SystemClock_Config() — 72 MHz
  │
  ▼
MX_GPIO_Init()
MX_I2C1_Init() — 400 kHz
MX_TIM2_Init() — motor PWM
MX_USART1_UART_Init() — 115200 baud
  │
  ▼
AppMain_Init()
  ├─ Initialize UART logger
  ├─ Probe ICM-20948 at 0x68
  ├─ Fallback probe at 0x69
  ├─ Read WHO_AM_I and expect 0xEA
  ├─ Reset and wake the IMU
  ├─ Configure accel ±2 g and gyro ±250 dps
  ├─ Initialize motor controller
  └─ Force motor stop
  │
  ▼
Main loop
```

If the ICM-20948 is not detected at either address, the application keeps UART diagnostics alive and rejects IMU reads. Balance output is not valid until a sensor sample is available.

## 5. Current sampling design

### 5.1 Sensor internal configuration

The driver configures the accelerometer and gyroscope at their highest raw output rate using the corresponding sample divider values. The application does not read the sensor continuously; it performs one read every 4 ms.

### 5.2 Application sampling rate

```text
Target sample rate = 250 Hz
Sample period       = 4 ms
I2C bus             = 400 kHz
Read transaction    = 14 data bytes, accel + gyro + temperature block
```

250 Hz is a practical initial rate for a two-wheel balance controller. It provides a 4 ms control interval while leaving CPU time for motor safety, filtering, logging and future communications. The balance loop should use the same timestamp as the IMU sample; it must not run on an uncontrolled tight loop.

### 5.3 Timing rules

- Do not call blocking sensor reads faster than the configured sample period.
- Do not print every IMU sample over UART.
- Keep diagnostic logging rate-limited to approximately 5–10 Hz.
- The PWM carrier frequency is independent of the balance control rate.
- The PID must use the measured `dt`, not a guessed constant, once the control loop is closed.

## 6. Angle calculation and filtering

The driver calculates static roll and pitch from the accelerometer, calibrates gyro bias while stationary at startup, and combines gyro integration with accelerometer correction using a complementary filter. The balance axis is currently the physical roll/Gyro-X axis selected for the installed GY-ICM20948V2 orientation.

```text
roll  = atan2(ay, sqrt(ax² + az²))
pitch = atan2(ax, sqrt(ay² + az²))
```

These angles are reliable while the platform is stationary or moving slowly, but accelerometer readings are disturbed by motor acceleration and vibration. The next control-stage implementation should use a complementary filter:

```text
angle = α × (previous_angle + gyro_rate × dt)
      + (1 - α) × accelerometer_angle
```

Recommended starting point:

- Control rate: 250 Hz
- `dt`: approximately 0.004 s, measured at runtime
- `α`: 0.96–0.99, tune experimentally
- Low-pass filtering: start conservatively; avoid adding excessive phase delay

The correct pitch axis depends on the physical orientation of the breakout board. Confirm the axis signs by logging the values while manually tilting the chassis before enabling PID.

## 7. Recommended FreeRTOS architecture

FreeRTOS is not currently present in this repository. After adding the official FreeRTOS middleware through STM32CubeMX, use the following task separation:

The current non-RTOS implementation already keeps the IMU update at 250 Hz. The `BalanceManager` can be moved into the future Balance task without changing its input/output contract.

```text
┌──────────────────────────────┐
│ IMU task — 250 Hz            │ High priority
│ I2C read, calibration,       │
│ filter, publish latest sample│
└──────────────┬───────────────┘
               │ queue / mutex / double buffer
               ▼
┌──────────────────────────────┐
│ Balance task — 250 Hz        │ Highest control priority
│ angle error → PID → motors   │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ Motor safety task — 100 Hz   │
│ timeout, saturation, stop    │
└──────────────────────────────┘

┌──────────────────────────────┐
│ Telemetry task — 10 Hz       │ Low priority
│ UART logs and diagnostics    │
└──────────────────────────────┘

┌──────────────────────────────┐
│ LED/button task — 20–50 Hz   │ Low priority
└──────────────────────────────┘
```

Use `vTaskDelayUntil()` for periodic tasks so the release time remains stable. With a 1 kHz FreeRTOS tick, the 4 ms IMU/control period is representable. A hardware timer or the sensor data-ready interrupt should be used later if sub-millisecond timing or lower jitter is required.

### Suggested task priorities

| Task | Rate | Priority | Notes |
|---|---:|---:|---|
| Balance/PID | 250 Hz | Highest | Must not block on UART |
| IMU acquisition | 250 Hz | High | Protect I2C with a mutex |
| Motor safety | 100 Hz | High | Stop on stale sensor/command |
| Telemetry | 10 Hz | Low | Never control balancing |
| LED/buttons | 20–50 Hz | Low | Human interface only |

For the first FreeRTOS version, an IMU task can read and publish a single latest-sample structure protected by a mutex. A queue is preferable when every sample must be processed; a double buffer is preferable when only the newest sample matters and latency must stay low.

## 8. FreeRTOS synchronization rules

- Use one I2C mutex for all future I2C devices.
- Never hold the I2C mutex while logging or running the PID.
- Copy the IMU sample into a local structure before releasing the mutex.
- Do not call blocking UART logging from the balance task.
- Use a watchdog or timestamp check: if the latest IMU sample is older than approximately 20 ms, command motor stop.
- Keep motor output bounded and apply a safe zero-output state on startup and fault.

## 9. Balance-control implementation order

1. Confirm ICM-20948 detection and address.
2. Log raw accelerometer and gyro values while the chassis is still.
3. Confirm roll/pitch axis direction and zero offset.
4. Validate the startup gyro bias calibration while the robot is stationary.
5. Validate the complementary filter and the roll/Gyro-X sign with the wheels lifted.
6. Add PID with motors lifted off the ground.
7. Tune saturation and motor deadband at low output.
8. Connect the encoder timer/callback and configure counts per revolution.
9. Tune PID on the floor at low battery and nominal battery voltage.

## 10. BalanceManager interface

The manager is implemented in:

- `Core/Inc/modules/control/balance_manager.hpp`
- `Core/Src/modules/control/balance_manager.cpp`

### Input contract

Each update provides:

| Input | Unit | Current source |
|---|---|---|
| `timestamp_ms` | ms | `HAL_GetTick()` |
| `dt_s` | s | Difference between IMU samples |
| `imu_valid` | boolean | ICM-20948 read result |
| `angle_deg` | degree | Complementary-filtered ICM-20948 roll |
| `angular_rate_dps` | degree/s | Bias-corrected ICM-20948 gyro X |
| `left/right_speed` | m/s | Wheel encoder module |
| `left/right_position` | m | Wheel encoder module |
| `battery_voltage` | V | Reserved for ADC/battery monitor |

### Output contract

| Output | Meaning |
|---|---|
| `left_motor_percent` | Left motor command from -100 to +100 |
| `right_motor_percent` | Right motor command from -100 to +100 |
| `angle_error_deg` | Target angle minus measured angle |
| `correction_percent` | Controller correction before motor application |
| `integral` | PID integral state for diagnostics |
| `algorithm` | Active controller mode |
| `faults` | Invalid IMU, stale IMU, tilt limit or disabled state |
| `saturated` | Output reached configured maximum |
| `active` | Output is safe to apply to the motors |

Supported modes are `Disabled`, `PID`, `PD` and `StateFeedback`. The state-feedback mode accepts angle, angular-rate, wheel-speed and wheel-position gains. The encoder module is ready for timer/callback counts; its runtime values remain zero until the board-specific encoder pins are configured.

The public app controls are:

```cpp
AppMain_SetBalanceEnabled(0);          // safe default: motor balance disabled
AppMain_SetBalanceAlgorithm(1);        // PID
AppMain_SetBalanceAlgorithm(2);        // PD
AppMain_SetBalanceAlgorithm(3);        // state feedback
```

Before floor testing, verify the roll/Gyro-X axis, sign convention, zero angle and gyro bias with the wheels lifted. The current firmware keeps the existing automatic balance-enable behavior requested for this project.

## 11. Current implementation status

| Component | Status |
|---|---|
| I2C1 PB6/PB7 | Implemented |
| ICM-20948 initialization | Implemented |
| Address fallback 0x68/0x69 | Implemented |
| Accel/gyro raw reading | Implemented |
| 250 Hz application sampling | Implemented |
| Angle calculation | Implemented, unfiltered |
| BalanceManager PID/PD/state feedback | Implemented |
| IMU and balance diagnostic logging | Implemented, rate-limited |
| Complementary filter | Implemented |
| Balance loop enable | Available through `AppMain_SetBalanceEnabled()` |
| FreeRTOS middleware | Not installed |
| FreeRTOS task split | Architecture defined |
| Encoder feedback | Software module ready; timer/pins pending board wiring |

## 12. References

- TDK InvenSense ICM-20948 datasheet: register map, sensor configuration and I2C limits.
- FreeRTOS API documentation: `vTaskDelayUntil()`, task priorities and tick configuration.
