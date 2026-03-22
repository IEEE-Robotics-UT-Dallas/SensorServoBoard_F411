#!/bin/bash
# Deploy Jetson workspace over SSH and build it
# Usage: ./deploy_to_jetson.sh <user@jetson_ip>

if [ -z "$1" ]; then
    echo "Usage: $0 <user@jetson_ip>"
    exit 1
fi

DEST="$1"
WORKSPACE_DIR="~/jetson_ws"

# Get absolute path of script directory to find the workspace correctly
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Deploying jetson_ws to $DEST..."
ssh "$DEST" "mkdir -p $WORKSPACE_DIR/src"
rsync -avz --exclude 'build' --exclude 'install' --exclude 'log' "$REPO_ROOT/jetson_ws/src/" "$DEST:$WORKSPACE_DIR/src/"

echo "Building workspace on Jetson..."
ssh "$DEST" "cd $WORKSPACE_DIR && source /opt/ros/humble/setup.bash && colcon build --symlink-install"

echo ""
echo "Deployment complete! On the Jetson, you can now run:"
echo "  source ~/jetson_ws/install/setup.bash"
echo "  ros2 launch sensor_servo_agent agent_launch.py"
