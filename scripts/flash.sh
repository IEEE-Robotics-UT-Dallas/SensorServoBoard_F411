#!/bin/bash
# Flash SensorServoBoard via ST-LINK (no IDE needed)

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="${1:-Debug}"
ELF="$PROJECT_ROOT/CubeIDE Project/SensorServorBoard/$CONFIG/SensorServorBoard.elf"

if [ ! -f "$ELF" ]; then
    echo "Error: $CONFIG ELF not found at:"
    echo "  $ELF"
    echo "Usage: $0 [Debug|Release]"
    exit 1
fi

echo "Flashing $CONFIG build..."
STM32_Programmer_CLI -c port=SWD -d "$ELF" -v -rst

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Flash complete — board reset and running"
else
    echo ""
    echo "✗ Flash failed"
    exit 1
fi
