# Sensor & Servo Controller Board (STM32F411CEU6)

This repository contains the firmware and terminal-based debugging toolchain for the Sensor and Servo control board.

## Hardware Configuration (STM32CubeMX / CubeIDE)

When setting up the `.ioc` file for this project, apply the following pinout and peripheral configuration:

### 1. Servos (PWM Output)
All servo pins must be configured as **Alternate Function Push-Pull** (PWM Generation).
*   **PA1**: `TIM2_CH2` (Servo 1)
*   **PA2**: `TIM2_CH3` (Servo 2)
*   **PA3**: `TIM2_CH4` (Servo 3)
*   **PA5**: `TIM2_CH1` (Servo 4)
*   **PA6**: `TIM3_CH1` (Servo 5)

*Configuration:*
*   Enable `TIM2` Channels 1, 2, 3, and 4 for PWM Generation.
*   Enable `TIM3` Channel 1 for PWM Generation.
*   Set the Prescaler and Counter Period to achieve a 50Hz (20ms) PWM signal suitable for standard servos.

### 2. Time-of-Flight (ToF) Sensors & I2C Bus
The sensors are distributed across 3 I2C buses to manage address conflicts.
*   **I2C1** (ToF 1 & 2): 
    *   SCL: **PB8**
    *   SDA: **PB9**
*   **I2C2** (ToF 3 & 4):
    *   SCL: **PB10**
    *   SDA: **PB3**
*   **I2C3** (ToF 5, Magnetometer, Light Sensor):
    *   SCL: **PA8**
    *   SDA: **PB4**

*Configuration:* Set all I2C interfaces to Fast Mode (400 kHz) if supported by all sensors.

### 3. ToF XSHUT (Enable/Reset) Pins
These pins are used to sequence the boot-up of the ToF sensors to assign them unique I2C addresses dynamically.
Configure as **GPIO_Output**:
*   **PB0**: XSHUT ToF 1
*   **PB1**: XSHUT ToF 2
*   **PB12**: XSHUT ToF 3
*   **PB13**: XSHUT ToF 4
*(Note: ToF 5 XSHUT pin is unassigned. It can either be hardwired to 3.3V if it's the only ToF on I2C3, or you may need to assign a 5th GPIO).*

### 4. FreeRTOS & System Configuration
When utilizing FreeRTOS for managing the multiple sensor streams and servo updates, apply these critical settings:
*   **Timebase Source**: Navigate to **System Core** -> **SYS** and change the **Timebase Source** from SysTick to a dedicated hardware timer (e.g., **TIM4** or **TIM5**). This is strongly recommended by ST to avoid conflicts with FreeRTOS's internal tick.
*   **Newlib Reentrant**: Navigate to **Middleware** -> **FREERTOS** -> **Advanced Settings**. Enable `USE_NEWLIB_REENTRANT` to ensure that standard C library calls (like `printf`, `malloc`, etc.) are thread-safe. Note that this will increase RAM usage slightly.

## Terminal Toolchain

This project includes the same headless build and TUI debugging toolchain as the main motor driver board.

### Build the Project
```bash
./build.sh
```

### Run the Debugger
```bash
./stm32-debug.sh
```

## ROS 2 Interface

The firmware exposes a micro-ROS node (`sensor_servo_board`) with the following topics.

### Published Topics (20 Hz)

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `telemetry` | `std_msgs/Float32MultiArray` | Aggregated sensor data (9 floats): [tof_0..tof_4 (meters), mag_x, mag_y, mag_z, light_lux] |
| `tof_0`..`tof_4` | `sensor_msgs/Range` | Individual ToF range (meters), INFRARED type, FOV 27°, 0.03–2.0 m |
| `imu/mag` | `sensor_msgs/MagneticField` | Magnetometer (QMC5883L) x/y/z raw values |
| `light` | `std_msgs/Float32` | Ambient light level (BH1750) in lux |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `servo_cmd` | `std_msgs/Float32MultiArray` | 5-element array of servo angles (0–180°) |

### Telemetry Array Layout

Matches the H5 Motor Driver Board aggregated-telemetry pattern:

| Index | Field | Unit | Source |
|-------|-------|------|--------|
| 0–4 | ToF distance 0–4 | meters | VL53L0X (converted from mm) |
| 5–7 | Magnetometer x/y/z | raw | QMC5883L |
| 8 | Ambient light | lux | BH1750 |

> The micro-ROS transport uses a board-agnostic API (`uros_transport.h`) over USART1 with DMA circular buffering (512 bytes). The ROS 2 node name is `sensor_servo_board`.

## Clock & Timer Configuration

### 1. Clock Configuration (100 MHz)
For optimal performance and stable PWM/I2C signals, configure the clock for **100 MHz** (maximum speed for the F411):
1. In **System Core** -> **RCC**:
   * Set **High Speed Clock (HSE)** to **Crystal/Ceramic Resonator** (assuming a standard STM32F411 board with a 25MHz crystal).
2. Go to the **Clock Configuration** tab:
   * Input frequency: **25** (or match your crystal frequency).
   * Select **HSE** as the PLL Source.
   * Type **100** into the **HCLK (MHz)** box and press **Enter**.
   * Let CubeMX automatically resolve the multipliers and dividers (it will set APB1 to 50MHz and APB1 Timer clocks to 100MHz).

### 2. Servo PWM Math (Based on 100MHz Timer Clock)
With APB1 Timer clocks running at 100 MHz, configuring TIM2 and TIM3 for a standard 50Hz (20ms) servo signal requires:
*   **Prescaler (PSC)**: `100 - 1` (This scales the timer to tick at 1 MHz, or 1µs per tick).
*   **Counter Period (ARR)**: `20000 - 1` (20,000 ticks = 20ms = 50Hz).
*   **Pulse (CCR)**: Set between `1000` and `2000`. (1500 is typically the center position, representing a 1.5ms pulse).
