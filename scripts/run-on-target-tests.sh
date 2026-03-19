#!/bin/bash
# Build & flash the on-target automated test suite
# Swaps hw_test.c for automated tests, builds with -DHW_TEST, restores after
#
# Usage:
#   ./scripts/run-on-target-tests.sh              Build only
#   ./scripts/run-on-target-tests.sh --flash       Build + flash + monitor
#   ./scripts/run-on-target-tests.sh --interactive  Build interactive hw_test instead

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_PATH="$PROJECT_ROOT/CubeIDE Project/SensorServorBoard"
CPROJECT="$PROJECT_PATH/.cproject"
HW_TEST_SRC="$PROJECT_PATH/Core/Src/hw_test.c"
ON_TARGET_SRC="$PROJECT_ROOT/tests/on_target/on_target_tests.c"

BOLD='\033[1m'
GREEN='\033[32m'
RED='\033[31m'
RESET='\033[0m'

DO_FLASH=false
INTERACTIVE=false

BAUD=500000
SERIAL_PORT=""

for arg in "$@"; do
    case "$arg" in
        --flash) DO_FLASH=true ;;
        --interactive) INTERACTIVE=true ;;
        --baud=*) BAUD="${arg#--baud=}" ;;
        --port=*) SERIAL_PORT="${arg#--port=}" ;;
        --help|-h)
            echo "On-target test runner for SensorServoBoard F411"
            echo ""
            echo "  $0              Build automated tests"
            echo "  $0 --flash      Build + flash + open serial monitor"
            echo "  $0 --interactive Build interactive hw_test instead"
            echo "  $0 --port=/dev/ttyACM0  Specify serial port"
            echo "  $0 --baud=115200        Override baud rate"
            exit 0
            ;;
    esac
done

echo -e "${BOLD}SensorServoBoard F411 — On-Target Tests${RESET}"
echo "────────────────────────────────────────"

# ── Backup original hw_test.c ────────────────────────────────────
if [ ! -f "$HW_TEST_SRC.orig" ]; then
    cp "$HW_TEST_SRC" "$HW_TEST_SRC.orig"
fi

# ── Swap in test source ─────────────────────────────────────────
cleanup() {
    echo -e "\n  Restoring original hw_test.c..."
    cp "$HW_TEST_SRC.orig" "$HW_TEST_SRC"

    echo "  Restoring .cproject defines..."
    sed -i 's/value="__DISABLED_MICRO_ROS_ENABLED__"/value="MICRO_ROS_ENABLED"/g' "$CPROJECT"
    sed -i 's/value="HW_TEST"/value="RMW_UXRCE_TRANSPORT_CUSTOM"/g' "$CPROJECT"
    echo -e "  ${GREEN}✓${RESET} Restored"
}
trap cleanup EXIT

if ! $INTERACTIVE; then
    echo "  Swapping in automated tests..."
    cp "$ON_TARGET_SRC" "$HW_TEST_SRC"
else
    echo "  Using interactive hw_test.c..."
fi

# ── Swap defines: MICRO_ROS_ENABLED → disabled, add HW_TEST ────
echo "  Configuring for HW_TEST build..."
sed -i 's/value="MICRO_ROS_ENABLED"/value="__DISABLED_MICRO_ROS_ENABLED__"/g' "$CPROJECT"
sed -i 's/value="RMW_UXRCE_TRANSPORT_CUSTOM"/value="HW_TEST"/g' "$CPROJECT"

# ── Build ────────────────────────────────────────────────────────
echo "  Building..."
rm -rf /tmp/sensorservo_headless_ws
cd "$PROJECT_ROOT"
if bash build.sh > /tmp/ssb-test-build.log 2>&1; then
    echo -e "  ${GREEN}✓${RESET} Build succeeded"
else
    ELF="$PROJECT_PATH/Debug/SensorServorBoard.elf"
    if [ -f "$ELF" ]; then
        echo -e "  ${GREEN}✓${RESET} Build completed (with warnings)"
    else
        echo -e "  ${RED}✗${RESET} Build failed"
        tail -20 /tmp/ssb-test-build.log
        exit 1
    fi
fi

ELF="$PROJECT_PATH/Debug/SensorServorBoard.elf"
arm-none-eabi-size "$ELF"

# ── Flash & monitor ──────────────────────────────────────────────
if $DO_FLASH; then
    echo ""

    # Use specified port or auto-detect
    SERIAL="$SERIAL_PORT"
    if [ -z "$SERIAL" ]; then
        for p in /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyUSB1 /dev/ttyACM1; do
            if [ -e "$p" ]; then
                SERIAL="$p"
                break
            fi
        done
    fi

    if [ -z "$SERIAL" ]; then
        echo "  No serial port found (/dev/ttyUSB* or /dev/ttyACM*)"
        echo "  Connect USB-UART adapter and run:"
        echo "    picocom -b $BAUD /dev/ttyUSB0"
        exit 0
    fi

    # Flash without reset first — we'll reset after serial is open
    echo "  Flashing test firmware (no reset)..."
    openocd -f "$SCRIPT_DIR/openocd.cfg" \
        -c "program \"$ELF\" verify exit" > /tmp/ssb-test-flash.log 2>&1
    echo -e "  ${GREEN}✓${RESET} Flashed"

    echo ""
    echo -e "${BOLD}Test output ($BAUD baud, port: $SERIAL):${RESET}"
    echo "────────────────────────────────────────"

    # Configure serial port
    if ! timeout 3 stty -F "$SERIAL" "$BAUD" raw -echo -hupcl 2>/dev/null; then
        echo "  Warning: could not configure $SERIAL at $BAUD baud"
    fi

    # Start background serial capture
    LOGFILE="/tmp/ssb-test-output.log"
    > "$LOGFILE"
    timeout 35 cat "$SERIAL" >> "$LOGFILE" 2>/dev/null &
    CAT_PID=$!
    sleep 0.5

    # Reset the board via OpenOCD to start tests
    echo "  Resetting board..."
    openocd -f "$SCRIPT_DIR/openocd.cfg" \
        -c "init; reset run; shutdown" > /dev/null 2>&1 || true
    echo "  Waiting for test output..."

    # Tail the log so user sees output in real time
    tail -f "$LOGFILE" 2>/dev/null &
    TAIL_PID=$!

    # Wait for test completion or timeout
    WAITED=0
    while [ $WAITED -lt 30 ]; do
        sleep 1
        WAITED=$((WAITED + 1))
        if grep -q "tests complete" "$LOGFILE" 2>/dev/null; then
            sleep 1
            break
        fi
        # Also stop if cat already exited
        if ! kill -0 $CAT_PID 2>/dev/null; then
            break
        fi
    done

    kill $TAIL_PID 2>/dev/null || true
    kill $CAT_PID 2>/dev/null || true
    wait $TAIL_PID 2>/dev/null || true
    wait $CAT_PID 2>/dev/null || true

    echo ""
    echo "────────────────────────────────────────"
    if grep -q "FAIL" "$LOGFILE" 2>/dev/null; then
        FAILS=$(grep -c "FAIL" "$LOGFILE")
        echo -e "  ${RED}✗${RESET} $FAILS test failure(s) — see output above"
    elif grep -q "PASS" "$LOGFILE" 2>/dev/null; then
        PASSES=$(grep -c "PASS" "$LOGFILE")
        echo -e "  ${GREEN}✓${RESET} All $PASSES tests passed"
    else
        echo "  No test output received — check wiring and baud rate ($BAUD)"
        echo "  Try: picocom -b $BAUD $SERIAL"
    fi
    echo ""
else
    echo ""
    echo "To flash and run:"
    echo "  $0 --flash"
    echo ""
    echo "Or manually:"
    echo "  STM32_Programmer_CLI -c port=SWD -d \"$ELF\" -v -rst"
    echo "  picocom -b 500000 /dev/ttyUSB0"
fi
