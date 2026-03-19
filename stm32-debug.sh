#!/bin/bash
# STM32 Terminal Debugger Launcher for SensorServoBoard
# Usage: ./stm32-debug.sh

PROJECT_NAME="SensorServorBoard"
PROJECT_PATH="$(pwd)/CubeIDE Project"
ELF_PATH="$PROJECT_PATH/Debug/$PROJECT_NAME.elf"
CFG="target/stm32f4x.cfg"

# Check if ELF exists
if [ ! -f "$ELF_PATH" ]; then
    echo "Error: ELF file not found at $ELF_PATH"
    echo "Please run ./build.sh first."
    exit 1
fi

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "Error: tmux is required for the TUI debugger."
    exit 1
fi

echo "Starting STM32 Debugger for STM32F411..."
echo "Project: $PROJECT_NAME"
echo "ELF: $ELF_PATH"

# Create a new tmux session
tmux new-session -d -s stm32-debug-f4 -n "OpenOCD" "openocd -f interface/stlink.cfg -f $CFG"
# We reference the universal .gdbinit from the parent directory's tools setup
tmux split-window -h -t stm32-debug-f4 "arm-none-eabi-gdb -ex 'set var \$SVD_FILE=\"STM32F411.svd\"' -x ../stm32-tui-debugger/.gdbinit \"$ELF_PATH\""
tmux select-pane -t 0
tmux split-window -v -t stm32-debug-f4 "echo 'SWO/ITM Trace Console'; itmdump -f /tmp/swo.log"
tmux select-pane -t 1

tmux attach-session -t stm32-debug-f4
