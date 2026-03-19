# GDB init for SensorServoBoard STM32F411 debugging
# ELF is passed as argument to GDB by the launch script

target extended-remote :3333

# Halt via SWD (no SRST wire needed)
monitor halt
load

# Safety breakpoints — catch hard faults and stack overflows
break HardFault_Handler
break BusFault_Handler
break UsageFault_Handler

# FreeRTOS thread awareness (may not work with HLA/ST-Link V2)
python
try:
    gdb.execute('monitor rtos auto', to_string=True)
except:
    pass
end

# Load debug dashboard (ssb commands)
source scripts/gdb-dashboard.py

# Reset to start of firmware and run to main
monitor reset init
tbreak main
continue
