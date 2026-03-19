#!/bin/bash
# STM32CubeIDE Headless Build Script for SensorServoBoard
# This script builds the project from the terminal.

# --- CONFIGURATION ---
CUBE_IDE_PATH="/opt/stm32cubeide/stm32cubeide"
PROJECT_PATH="$(pwd)/CubeIDE Project/SensorServorBoard"
PROJECT_NAME="SensorServorBoard"
BUILD_CONFIG="Debug" # or Release
WORKSPACE_PATH="/tmp/sensorservo_headless_ws"

# Check if CUBE_IDE_PATH exists
if [ ! -f "$CUBE_IDE_PATH" ]; then
    echo "Error: STM32CubeIDE not found at $CUBE_IDE_PATH"
    echo "Please update the CUBE_IDE_PATH variable in this script if necessary."
    exit 1
fi

echo "Building $PROJECT_NAME ($BUILD_CONFIG)..."

# Create a temporary workspace if it doesn't exist
mkdir -p "$WORKSPACE_PATH"

# Run headless build
"$CUBE_IDE_PATH" -nosplash \
    -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
    -data "$WORKSPACE_PATH" \
    -import "$PROJECT_PATH" \
    -build "$PROJECT_NAME/$BUILD_CONFIG"

# Check build status
if [ $? -eq 0 ]; then
    echo "Build Successful!"
    ELF_FILE="$PROJECT_PATH/$BUILD_CONFIG/$PROJECT_NAME.elf"
    if [ -f "$ELF_FILE" ]; then
        echo "ELF Location: $ELF_FILE"
    fi
else
    echo "Build Failed!"
    exit 1
fi
