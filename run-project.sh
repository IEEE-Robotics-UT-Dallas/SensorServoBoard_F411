#!/bin/bash
# Master Build & Debug Script for SensorServoBoard
# This script builds the project and starts the TUI debugger.

# 1. SVD Check
echo "--- STEP 1: Checking SVD Files ---"
# We share the stm32-tui-debugger folder from the parent directory
SVD_PATH="../stm32-tui-debugger/STM32F411.svd"
if [ ! -f "$SVD_PATH" ]; then
    echo "Downloading SVD for F411..."
    curl -L https://github.com/modm-io/cmsis-svd-stm32/raw/master/stm32f4/STM32F411.svd -o "$SVD_PATH"
fi

# 2. Build
echo "--- STEP 2: Building Project ---"
./build.sh
if [ $? -ne 0 ]; then
    echo "Build failed. Aborting."
    exit 1
fi

# 3. Debug
echo "--- STEP 3: Launching TUI Debugger ---"
./stm32-debug.sh
