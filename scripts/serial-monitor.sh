#!/bin/bash
# Serial monitor for SensorServoBoard UART output
# USART1: micro-ROS transport at 460800 baud
# Use this to sniff raw XRCE-DDS traffic or debug UART issues

BAUD=500000
PORT=""

# Auto-detect serial port
for p in /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyUSB1 /dev/ttyACM1; do
    if [ -e "$p" ]; then
        PORT="$p"
        break
    fi
done

if [ -z "$PORT" ]; then
    echo "No serial device found. Available ports:"
    ls /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null || echo "  (none)"
    echo ""
    echo "Usage: $0 [port] [baud]"
    echo "  e.g. $0 /dev/ttyUSB0 460800"
    exit 1
fi

# Override from args
PORT="${1:-$PORT}"
BAUD="${2:-$BAUD}"

echo "Connecting to $PORT @ ${BAUD} baud..."
echo "Exit: Ctrl+A then Ctrl+X"
echo ""

picocom -b "$BAUD" "$PORT"
