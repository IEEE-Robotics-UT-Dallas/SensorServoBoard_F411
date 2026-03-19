#!/usr/bin/env bash
# Launch TUI debugger for SensorServoBoard
# Usage: bash scripts/tui-debug.sh [--poll-rate HZ]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
DEBUGGER="$HOME/stm32-tui-debugger-outline/target/release/stm32-tui-debugger"

if [[ ! -f "$DEBUGGER" ]]; then
    echo "Error: TUI debugger not found at $DEBUGGER"
    echo "Build it: cd ~/stm32-tui-debugger-outline && cargo build --release"
    exit 1
fi

cd "$PROJECT_DIR"
exec "$DEBUGGER" -C debugger.toml "$@"
