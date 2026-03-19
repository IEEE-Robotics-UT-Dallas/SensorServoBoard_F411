# GDB init for SensorServoBoard STM32F411 debugging
# ELF is passed as argument to GDB by the launch script

target extended-remote :3333
set confirm off

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

# Reset, boot firmware, wait for FreeRTOS to start, then halt and show dashboard
python
import time
gdb.execute('monitor reset init', to_string=True)
gdb.execute('monitor resume', to_string=True)
print("Booting firmware (2s)...", flush=True)
time.sleep(2)
gdb.execute('monitor halt', to_string=True)
time.sleep(0.05)
gdb.execute('ssb health', to_string=False)
print("\nType 'c' to resume, Ctrl+C to halt, 'ssb' to inspect, 'quit' to exit.")
end
