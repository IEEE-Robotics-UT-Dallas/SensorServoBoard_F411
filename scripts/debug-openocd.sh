#!/bin/bash
# Start OpenOCD server for STM32F411 SensorServoBoard
# Run this in one terminal, then connect with debug-gdb.sh in another

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting OpenOCD for SensorServoBoard (STM32F411)..."
echo "GDB server on :3333 | Telnet on :4444"
echo "Press Ctrl+C to stop"
echo ""

openocd -f "$SCRIPT_DIR/openocd.cfg"
