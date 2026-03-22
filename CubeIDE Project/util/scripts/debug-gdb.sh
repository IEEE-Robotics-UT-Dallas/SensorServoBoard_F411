#!/bin/bash
# Connect GDB to OpenOCD for live debugging
# Requires debug-openocd.sh running in another terminal

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ELF="$PROJECT_ROOT/CubeIDE Project/SensorServorBoard/Debug/SensorServorBoard.elf"

if [ ! -f "$ELF" ]; then
    echo "Error: Debug ELF not found. Run build.sh first."
    exit 1
fi

echo "Connecting to SensorServoBoard via OpenOCD on :3333..."
echo "  Type 'ssb' for debug dashboard, 'ssb help' for all commands"
echo ""

cd "$PROJECT_ROOT"
arm-none-eabi-gdb -x scripts/gdb-init.gdb "$ELF"
