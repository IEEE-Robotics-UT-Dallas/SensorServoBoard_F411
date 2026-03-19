#!/bin/bash
# Launch a full debug session: OpenOCD + GDB + micro-ROS agent
# Opens each in a new Ghostty window via Hyprland

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Launching SensorServoBoard debug session..."

# 1. OpenOCD in a new terminal
hyprctl dispatch exec "ghostty -e bash -c 'cd \"$PROJECT_ROOT\" && bash scripts/debug-openocd.sh; read -p \"Press enter to close...\"'" 2>/dev/null
sleep 1

# 2. GDB in a new terminal
hyprctl dispatch exec "ghostty -e bash -c 'cd \"$PROJECT_ROOT\" && sleep 2 && bash scripts/debug-gdb.sh'" 2>/dev/null

# 3. micro-ROS agent (if serial port available)
for p in /dev/ttyUSB0 /dev/ttyACM0; do
    if [ -e "$p" ]; then
        hyprctl dispatch exec "ghostty -e bash -c 'cd \"$PROJECT_ROOT\" && bash scripts/uros-agent.sh; read -p \"Press enter to close...\"'" 2>/dev/null
        break
    fi
done

echo ""
echo "Debug windows launched:"
echo "  • OpenOCD server (GDB :3333, Telnet :4444)"
echo "  • GDB connected to target"
if ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1 > /dev/null 2>&1; then
    echo "  • micro-ROS agent on serial"
else
    echo "  • micro-ROS agent skipped (no serial port)"
fi
echo ""
echo "Quick commands:"
echo "  Flash:   scripts/flash.sh [Debug|Release]"
echo "  Serial:  scripts/serial-monitor.sh"
echo "  Agent:   scripts/uros-agent.sh"
