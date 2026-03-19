#!/bin/bash
# Verify STM32 ↔ ROS 2 micro-ROS communication
# Usage: scripts/test-uros.sh [serial_port]
set -euo pipefail

PORT="${1:-/dev/ttyACM0}"
BAUD=500000
TIMEOUT=30

echo "SensorServoBoard — micro-ROS Integration Test"
echo "────────────────────────────────────────"

# Check serial port
if [ ! -e "$PORT" ]; then
    echo "  ✗ Serial port $PORT not found"
    echo "  Available ports:"
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | sed 's/^/    /' || echo "    (none)"
    exit 1
fi
echo "  ✓ Serial port: $PORT"

# Check Docker
if ! docker info >/dev/null 2>&1; then
    echo "  ✗ Docker not running"
    exit 1
fi
echo "  ✓ Docker running"

# Pull images if needed
echo ""
echo "  Pulling Docker images (if needed)..."
docker pull -q microros/micro-ros-agent:jazzy 2>/dev/null || true
docker pull -q ros:jazzy 2>/dev/null || true

# Stop any existing containers
docker compose -f docker/docker-compose.yml down 2>/dev/null || true

# Start the agent
echo ""
echo "  Starting micro-ROS agent on $PORT @ ${BAUD} baud..."
SERIAL_PORT="$PORT" BAUD="$BAUD" docker compose -f docker/docker-compose.yml up -d uros-agent

echo "  Waiting ${TIMEOUT}s for agent + STM32 handshake..."
sleep 5

# Check if agent container is running
if ! docker ps --format '{{.Names}}' | grep -q uros-agent; then
    echo "  ✗ Agent container failed to start"
    docker logs uros-agent 2>&1 | tail -10
    exit 1
fi
echo "  ✓ Agent container running"

# Start ros2-cli container and list topics
echo ""
echo "  Checking for ROS 2 topics..."
TOPICS=""
for i in $(seq 1 $TIMEOUT); do
    TOPICS=$(docker compose -f docker/docker-compose.yml run --rm ros2-cli \
        bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null" 2>/dev/null || true)
    if echo "$TOPICS" | grep -q "tof_\|imu/mag\|light\|telemetry"; then
        break
    fi
    sleep 1
done

echo ""
if echo "$TOPICS" | grep -q "tof_\|imu/mag\|light\|telemetry"; then
    echo "  ✓ ROS 2 topics discovered:"
    echo "$TOPICS" | grep -E "tof_|imu/mag|light|telemetry|servo_cmd" | sed 's/^/    /'
    echo ""

    # Echo one message from each key topic
    echo "  Sampling sensor data (5s)..."
    echo ""

    echo "  ── /imu/mag ──"
    docker compose -f docker/docker-compose.yml run --rm ros2-cli \
        bash -c "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo /imu/mag --once 2>/dev/null" 2>/dev/null \
        | head -10 | sed 's/^/    /' || echo "    (no data yet)"

    echo ""
    echo "  ── /light ──"
    docker compose -f docker/docker-compose.yml run --rm ros2-cli \
        bash -c "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo /light --once 2>/dev/null" 2>/dev/null \
        | head -5 | sed 's/^/    /' || echo "    (no data yet)"

    echo ""
    echo "  ── /telemetry ──"
    docker compose -f docker/docker-compose.yml run --rm ros2-cli \
        bash -c "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo /telemetry --once 2>/dev/null" 2>/dev/null \
        | head -15 | sed 's/^/    /' || echo "    (no data yet)"

    echo ""
    echo "────────────────────────────────────────"
    echo "  ✓ micro-ROS bridge working!"
    echo ""
    echo "  To interact manually:"
    echo "    docker exec -it ros2-cli bash"
    echo "    ros2 topic list"
    echo "    ros2 topic echo /imu/mag"
    echo "    ros2 topic pub /servo_cmd std_msgs/msg/Float32MultiArray \\"
    echo "      '{data: [90.0, 90.0, 90.0, 90.0, 90.0]}'"
else
    echo "  ✗ No sensor topics found after ${TIMEOUT}s"
    echo "  Agent logs:"
    docker logs uros-agent 2>&1 | tail -20 | sed 's/^/    /'
    echo ""
    echo "  Possible causes:"
    echo "    - STM32 firmware not running (flash normal firmware, not HW_TEST)"
    echo "    - Wrong serial port or baud rate"
    echo "    - USART6 DMA not initialized"
    echo ""
    echo "────────────────────────────────────────"
    echo "  ✗ Test FAILED"
    exit 1
fi
