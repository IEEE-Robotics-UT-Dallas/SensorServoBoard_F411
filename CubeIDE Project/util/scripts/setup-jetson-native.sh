#!/bin/bash
# Installs micro-ROS agent natively on the Jetson
# Usage: copy to jetson and run, or run via SSH: ssh user@jetson "bash -s" < setup-jetson-native.sh

set -e

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    else
        echo "Error: ROS 2 setup.bash not found. Is ROS 2 installed on this Jetson?"
        exit 1
    fi
fi

WORKSPACE_DIR="$HOME/micro_ros_ws"

echo "Setting up micro-ROS agent workspace natively in $WORKSPACE_DIR..."
mkdir -p "$WORKSPACE_DIR/src"
cd "$WORKSPACE_DIR"

if [ ! -d "src/micro-ROS-Agent" ]; then
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent
else
    echo "micro-ROS-Agent repository already exists, pulling latest..."
    cd src/micro-ROS-Agent && git pull && cd ../..
fi

echo "Installing dependencies using rosdep..."
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

echo "Building micro-ROS agent..."
colcon build

echo ""
echo "micro-ROS agent built successfully!"
echo "To run the agent natively on this Jetson, use:"
echo "  source $WORKSPACE_DIR/install/local_setup.bash"
echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 500000"
