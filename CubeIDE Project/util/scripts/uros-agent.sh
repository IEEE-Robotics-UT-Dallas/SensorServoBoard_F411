#!/bin/bash
# Start micro-ROS agent via Docker to bridge UART ↔ ROS 2
# Connects to the SensorServoBoard over serial

PORT=""
BAUD=500000

# Auto-detect serial port
for p in /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyUSB1 /dev/ttyACM1; do
    if [ -e "$p" ]; then
        PORT="$p"
        break
    fi
done

PORT="${1:-$PORT}"
BAUD="${2:-$BAUD}"

if [ -z "$PORT" ]; then
    echo "No serial device found."
    echo "Usage: $0 [port] [baud]"
    echo "  e.g. $0 /dev/ttyUSB0 460800"
    exit 1
fi

echo "Starting micro-ROS agent on $PORT @ ${BAUD} baud..."
echo "Expected topics:"
echo "  /tof_0..tof_4  (sensor_msgs/Range)"
echo "  /imu/mag       (sensor_msgs/MagneticField)"
echo "  /light         (std_msgs/Float32)"
echo "  /servo_cmd     (std_msgs/Float32MultiArray) [subscriber]"
echo ""
echo "Press Ctrl+C to stop"
echo ""

docker run --rm -it \
    --device="$PORT" \
    --net=host \
    microros/micro-ros-agent:humble \
    serial --dev "$PORT" -b "$BAUD" -v6
