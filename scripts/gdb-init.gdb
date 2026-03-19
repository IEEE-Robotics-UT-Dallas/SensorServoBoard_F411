# GDB init for SensorServoBoard STM32F411 debugging
target extended-remote :3333

# Load symbols
file CubeIDE Project/SensorServorBoard/Debug/SensorServorBoard.elf

# Flash and reset
monitor reset halt
load

# Safety breakpoints — catch hard faults and stack overflows
break HardFault_Handler
break BusFault_Handler
break UsageFault_Handler

# FreeRTOS thread awareness (shows tasks as GDB threads)
monitor rtos auto

# Load debug dashboard (ssb commands)
source scripts/gdb-dashboard.py

# Start execution
tbreak main
continue
