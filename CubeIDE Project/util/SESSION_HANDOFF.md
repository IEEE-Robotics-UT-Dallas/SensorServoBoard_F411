# SensorServoBoard F411 — Session Handoff

## Project Overview
Custom sensor/servo board running STM32F411CEU6 with FreeRTOS. Connected via ST-Link.
Firmware in `CubeIDE Project/SensorServorBoard/` (note typo in folder name).

### Hardware Connected
- **MLX90393** magnetometer on I2C3 (addr 0x0C, 8-bit 0x18)
- **VEML7700** light sensor on I2C3 (addr 0x10, 8-bit 0x20)
- **2× ToF sensors** on I2C1 (default addr 0x29 / 8-bit 0x52) — **type unknown, likely VL53L1X not VL53L0X**
- **5 servos** on TIM2/TIM3 PWM channels
- **USART6** (PA11/PA12 via USB-C) — **micro-ROS XRCE-DDS transport** @ 500000 baud

### Pin Assignments
| Function | Pin |
|----------|-----|
| I2C1 SCL | PB8 |
| I2C1 SDA | PB9 |
| I2C3 SCL | PA8 |
| I2C3 SDA | PB4 |
| ToF 1 XSHUT | PB0 |
| ToF 2 XSHUT | PB1 |
| ToF 3 XSHUT | PB12 |
| ToF 4 XSHUT | PB13 |
| USART6 TX | PA11 |
| USART6 RX | PA12 |

### Build & Flash
```bash
cd ~/SensorServoBoard_F411 && bash build.sh    # Headless build
# Flash via TUI debugger F8 key, or:
probe-rs download --chip STM32F411CEUx "CubeIDE Project/SensorServorBoard/Debug/SensorServorBoard.elf"
```

---

## Architecture

### Firmware Tasks (FreeRTOS)
- **StartI2C3Task** — Reads mag (100Hz DMA) + light (5Hz), writes to double buffer
- **StartI2C1Task** — XSHUT sequence + ToF reads (20Hz), writes to double buffer *(guarded by `TOF_SENSORS_CONNECTED`)*
- **uros_task** — micro-ROS publishers, servo command subscriber, and Parameter Server (dynamic calibration)

### Key Subsystems
1. **Flash Config** (`flash_config.h/c`) — XOR-CRC validated persistence in Sector 7. Loads/saves `BoardConfig_t`.
2. **Parameter Manager** (`uros_task.c`) — Bridges micro-ROS `on_parameter_changed` to the RAM config struct.
3. **Double Buffer** (`double_buffer.h/c`) — Lock-free sensor snapshot sharing between writer tasks and reader (uros).
4. **I2C3 DMA** (`i2c_common.h/c`) — Semaphore-synchronized DMA transfers.
5. **MLX90393** (`mlx90393.h/c`) — Magnetometer driver (working, DMA).
6. **VEML7700** (`veml7700.h/c`) — Light sensor driver (working, DMA).
7. **micro-ROS Transport** (`uros_transport.h/c`) — UART6 DMA circular buffer with HAL error auto-recovery.
8. **Servo Control** (`servo_control.h/c`) — PWM + offset calibration layer.

### Important Files
```
Core/Src/uros_task.c          — micro-ROS node: publishers, subscriber, params
Core/Src/uros_transport.c     — UART6 DMA circular-buffer transport
Core/Src/sensor_tasks.c       — All FreeRTOS sensor tasks
Core/Src/mlx90393.c           — Magnetometer driver (working, DMA)
Core/Src/veml7700.c           — Light sensor driver (working, DMA)
Core/Src/vl53l0x.c            — ToF driver (BROKEN — wrong sensor type)
Core/Src/servo_control.c      — ServoControl API (angle + offset + pulse)
Core/Src/i2c_common.c         — I2C recovery + DMA wrappers
Core/Src/double_buffer.c      — Lock-free double buffer
Core/Src/flash_config.c       — Sector 7 flash persistence
Core/Src/microros_allocators.c — FreeRTOS-backed allocators for micro-ROS
Core/Src/microros_time.c      — clock_gettime for micro-ROS (FreeRTOS tick)
Core/Src/custom_memory_manager.c — heap_4 realloc/calloc wrappers
Core/Src/main.c               — CubeMX-generated init + DMA/NVIC config + task creation
Core/Inc/*.h                  — Corresponding headers
```

---

## Current State (March 19, 2026)

### Build Configuration
- **`MICRO_ROS_ENABLED`** — Active (production micro-ROS mode)
- **`HW_TEST`** — Removed (test suite disabled)
- **`TOF_SENSORS_CONNECTED`** — Not defined (I2C1/I2C2 tasks skipped — ToF hardware issue)

### Resource Usage
| Resource | Used | Available | % |
|----------|------|-----------|---|
| Flash (text+data) | 227 KB | 512 KB | 44% |
| RAM (data+bss) | 114 KB | 128 KB | 89% |

### What's Working ✓
- System clock (96MHz), FreeRTOS, heap
- I2C3 bus: MLX90393 mag reads (DMA), VEML7700 light
- Double buffer: Lock-free sensor data sharing
- **Flash Persistence**: Parameters survive power-cycles (Sector 7)
- **Servo PWM**: Corrected logic (0°=500, 180°=2500) + centered at 90° on boot
- **micro-ROS node** (`sensor_servo_board`): Compiled, linked, flashed
  - Transport: USART6 DMA circular buffer (500 kbaud)
  - Publishers: `/imu/mag`, `/telemetry`, `/servo_positions` @ 20 Hz
  - Subscriber: `/servo_cmd` (Float32MultiArray × 5 angles)
  - Parameter Server: `servo_offset_1..5` with 2s flash-write settle

### HARDWARE BLOCKER: ToF Sensors (I2C1)
The ToF diagnostic scan found **0 devices on I2C1**.
- Software toggles `XSHUT PB0` successfully, but sensors do not ACK at default `0x29`.
- **Verdict**: Physical layer failure (power, wiring, or pull-ups).
- Task creation is guarded by `TOF_SENSORS_CONNECTED` (not defined).

---

## micro-ROS ↔ Jetson Integration

### Physical Connection
```
STM32 USART6 (USB-C) → UART-over-Arduino → Jetson USB
```

### Running the Agent (on Jetson)
```bash
# Option A: Quick docker run
docker run --rm -it --device=/dev/ttyUSB0 --net=host \
  microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 500000 -v6

# Option B: Docker compose (persistent)
cd ~/SensorServoBoard_F411
SERIAL_PORT=/dev/ttyUSB0 BAUD=500000 docker compose -f docker/docker-compose.yml up -d

# Option C: Helper script
./scripts/uros-agent.sh /dev/ttyUSB0 500000
```

### Verifying Topics
```bash
# Inside the ros2-cli container, or any ROS 2 Humble shell:
ros2 topic list
ros2 topic echo /imu/mag
ros2 topic echo /telemetry
ros2 topic echo /servo_positions

# Command servos:
ros2 topic pub /servo_cmd std_msgs/msg/Float32MultiArray '{data: [90.0, 90.0, 90.0, 90.0, 90.0]}'

# Tune offsets (persisted to flash after 2s):
ros2 param set /sensor_servo_board servo_offset_1 -5.5
```

### Full Integration Test
```bash
./scripts/test-uros.sh /dev/ttyUSB0
```

### ROS 2 Topic Map
| Topic | Type | Dir | Rate |
|-------|------|-----|------|
| `/imu/mag` | `sensor_msgs/MagneticField` | F411→Jetson | 20 Hz |
| `/telemetry` | `std_msgs/Float32MultiArray` | F411→Jetson | 20 Hz |
| `/servo_positions` | `std_msgs/Float32MultiArray` | F411→Jetson | 20 Hz |
| `/servo_cmd` | `std_msgs/Float32MultiArray` | Jetson→F411 | On demand |
| `/sensor_servo_board/*` | Parameter services | Bidir | On demand |

### Telemetry Array Layout
| Index | Field | Unit | Source |
|-------|-------|------|--------|
| 0–4 | ToF distance 0–4 | meters | VL53L0X/L1X (0 when disconnected) |
| 5–7 | Magnetometer x/y/z | raw | MLX90393 |
| 8 | Ambient light | lux | VEML7700 |

---

## MLX90393 Magnetometer — Key Protocol Notes

- **Sensor persists state across MCU resets** — probe-rs resets MCU but NOT the sensor
- **Exit-first init required**: EX (0x80) → RT (0xF0) → 200ms delay → bus recovery → WR config → SB
- Status byte format: `[BURST|WOC|SM|ERROR|SED|RS|D1|D0]`
- CS pin pulled HIGH = I2C mode (correct for this board)

## I2C3 DMA Architecture
- DMA1 Stream2/Ch3 (RX) and Stream4/Ch3 (TX)
- Binary semaphore (`i2c3_dma_sem`) — starts taken, released by HAL callbacks
- HAL DMA functions internally enable/disable per-transfer interrupts

## USART6 Transport Notes
- **EIE (Error Interrupt Enable) is disabled** after DMA RX start
  - STM32F4 HAL treats ORE/FE/NE as "blocking errors" in DMA mode → aborts circular RX
  - With EIE off, byte errors are silently dropped; XRCE-DDS CRC catches corruption
- `ensure_dma_rx_running()` auto-recovers if HAL error handler killed the DMA RX
- TX uses `HAL_UART_Transmit_DMA` with busy-wait completion (1s timeout)

---

## Git State
- **SensorServoBoard_F411**: Branch `main`
- Key changes since last commit:
  - `.cproject` — `MICRO_ROS_ENABLED` active, `HW_TEST` removed
  - `uros_task.c` — Complete micro-ROS node (publishers, subscriber, params, timestamp)
  - `uros_transport.c` — Removed unused variable
- **TUI debugger** (`~/stm32-tui-debugger-outline/`): Branch `master`, at `f20b375`.
