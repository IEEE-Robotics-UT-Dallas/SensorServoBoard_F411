# GDB init for SensorServoBoard STM32F411 debugging
# ELF is passed as argument to GDB by the launch script

target extended-remote :3333

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
