#!/bin/bash
# Build & flash the hardware test firmware (no micro-ROS, UART console)
# Connect USB-UART to PB6(TX)/PB7(RX), open picocom at 500000 baud

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_PATH="$PROJECT_ROOT/CubeIDE Project/SensorServorBoard"
CPROJECT="$PROJECT_PATH/.cproject"

# Swap defines: remove MICRO_ROS_ENABLED, add HW_TEST
echo "Configuring for HW_TEST build..."
sed -i 's/value="MICRO_ROS_ENABLED"/value="__DISABLED_MICRO_ROS_ENABLED__"/g' "$CPROJECT"
sed -i 's/value="RMW_UXRCE_TRANSPORT_CUSTOM"/value="HW_TEST"/g' "$CPROJECT"

# Build
rm -rf /tmp/sensorservo_headless_ws
cd "$PROJECT_ROOT" && bash build.sh

BUILD_OK=$?

# Restore defines
sed -i 's/value="__DISABLED_MICRO_ROS_ENABLED__"/value="MICRO_ROS_ENABLED"/g' "$CPROJECT"
sed -i 's/value="HW_TEST"/value="RMW_UXRCE_TRANSPORT_CUSTOM"/g' "$CPROJECT"

if [ $BUILD_OK -ne 0 ]; then
    # Check if ELF was actually produced (CubeIDE exits 1 on warnings)
    ELF="$PROJECT_PATH/Debug/SensorServorBoard.elf"
    if [ ! -f "$ELF" ] || [ ! "$(find "$ELF" -newer "$CPROJECT" 2>/dev/null)" ]; then
        echo "Build failed!"
        exit 1
    fi
fi

ELF="$PROJECT_PATH/Debug/SensorServorBoard.elf"
echo ""
arm-none-eabi-size "$ELF"

# Flash if requested
if [ "${1}" = "--flash" ]; then
    echo ""
    echo "Flashing HW test firmware..."
    STM32_Programmer_CLI -c port=SWD -d "$ELF" -v -rst
    echo ""
    echo "Done! Open serial console:"
    echo "  picocom -b 500000 /dev/ttyUSB0"
else
    echo ""
    echo "To flash: $0 --flash"
    echo "To monitor: picocom -b 500000 /dev/ttyUSB0"
fi
