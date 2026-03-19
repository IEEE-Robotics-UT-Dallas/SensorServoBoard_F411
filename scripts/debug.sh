#!/bin/bash
# ── SensorServoBoard F411 — Single-command debug deployment ──────
#
# Starts OpenOCD, optionally builds/flashes, launches GDB with the
# debug dashboard. Everything in one terminal, clean teardown on exit.
#
# Usage:
#   ./scripts/debug.sh              Connect and debug (assumes already flashed)
#   ./scripts/debug.sh --flash      Flash latest ELF before debugging
#   ./scripts/debug.sh --build      Build, flash, and debug
#   ./scripts/debug.sh --health     Flash, connect, run health check, exit
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ELF="$PROJECT_ROOT/CubeIDE Project/SensorServorBoard/Debug/SensorServorBoard.elf"
OPENOCD_CFG="$SCRIPT_DIR/openocd.cfg"
OPENOCD_LOG="/tmp/ssb-openocd.log"
OPENOCD_PID=""

# ── Colors ───────────────────────────────────────────────────────
BOLD='\033[1m'
DIM='\033[2m'
GREEN='\033[32m'
RED='\033[31m'
YELLOW='\033[33m'
CYAN='\033[36m'
RESET='\033[0m'

msg()  { echo -e "  ${CYAN}▸${RESET} $*"; }
ok()   { echo -e "  ${GREEN}✓${RESET} $*"; }
fail() { echo -e "  ${RED}✗${RESET} $*"; }
warn() { echo -e "  ${YELLOW}!${RESET} $*"; }

# ── Cleanup on exit ──────────────────────────────────────────────
cleanup() {
    if [ -n "$OPENOCD_PID" ] && kill -0 "$OPENOCD_PID" 2>/dev/null; then
        msg "Stopping OpenOCD (PID $OPENOCD_PID)..."
        kill "$OPENOCD_PID" 2>/dev/null
        wait "$OPENOCD_PID" 2>/dev/null || true
        ok "OpenOCD stopped"
    fi
}
trap cleanup EXIT INT TERM

# ── Parse args ───────────────────────────────────────────────────
DO_BUILD=false
DO_FLASH=false
HEALTH_ONLY=false

for arg in "$@"; do
    case "$arg" in
        --build)  DO_BUILD=true; DO_FLASH=true ;;
        --flash)  DO_FLASH=true ;;
        --health) DO_FLASH=true; HEALTH_ONLY=true ;;
        --help|-h)
            echo -e "${BOLD}SensorServoBoard Debug${RESET}"
            echo ""
            echo "  $0              Connect GDB with debug dashboard"
            echo "  $0 --flash      Flash before debugging"
            echo "  $0 --build      Build + flash + debug"
            echo "  $0 --health     Flash + run health check + exit"
            echo ""
            echo "Requires: ST-Link connected, openocd, arm-none-eabi-gdb"
            exit 0
            ;;
        *)
            echo "Unknown option: $arg (try --help)"
            exit 1
            ;;
    esac
done

echo ""
echo -e "${BOLD}SensorServoBoard F411 — Debug Session${RESET}"
echo -e "${DIM}────────────────────────────────────────${RESET}"

# ── 1. Check prerequisites ──────────────────────────────────────
msg "Checking prerequisites..."

if ! command -v openocd &>/dev/null; then
    fail "openocd not found"; exit 1
fi
if ! command -v arm-none-eabi-gdb &>/dev/null; then
    fail "arm-none-eabi-gdb not found"; exit 1
fi

# Check ST-Link
PROBE=$(st-info --probe 2>&1)
if echo "$PROBE" | grep -q "Found 0"; then
    fail "No ST-Link detected — is it plugged in?"
    exit 1
fi
STLINK_VER=$(echo "$PROBE" | grep "version:" | head -1 | awk '{print $2}')
ok "ST-Link ${STLINK_VER:-detected}"

# ── 2. Build (optional) ─────────────────────────────────────────
if $DO_BUILD; then
    msg "Building firmware..."
    cd "$PROJECT_ROOT"
    if bash build.sh 2>&1 | tail -5; then
        ok "Build complete"
    else
        # CubeIDE exits 1 on warnings — check if ELF was produced
        if [ -f "$ELF" ]; then
            warn "Build exited with warnings (ELF produced)"
        else
            fail "Build failed — no ELF produced"
            exit 1
        fi
    fi
fi

# ── 3. Verify ELF ───────────────────────────────────────────────
if [ ! -f "$ELF" ]; then
    fail "Debug ELF not found at:"
    echo "      $ELF"
    echo ""
    echo "  Run: ./build.sh   or   $0 --build"
    exit 1
fi

ELF_SIZE=$(arm-none-eabi-size "$ELF" 2>/dev/null | tail -1 | awk '{print $4}')
ELF_DATE=$(date -r "$ELF" '+%Y-%m-%d %H:%M')
ok "ELF: ${ELF_SIZE:-?} bytes (${ELF_DATE})"

# ── 4. Kill any existing OpenOCD ─────────────────────────────────
if ss -tlnp 2>/dev/null | grep -q ':3333'; then
    EXISTING_PID=$(ss -tlnp 2>/dev/null | grep ':3333' | grep -oP 'pid=\K[0-9]+' | head -1)
    if [ -n "$EXISTING_PID" ]; then
        warn "OpenOCD already running (PID $EXISTING_PID) — reusing"
        OPENOCD_PID=""
    fi
else
    # ── 5. Start OpenOCD ─────────────────────────────────────────
    msg "Starting OpenOCD..."
    openocd -f "$OPENOCD_CFG" > "$OPENOCD_LOG" 2>&1 &
    OPENOCD_PID=$!

    # Wait for GDB port to be ready
    for i in $(seq 1 30); do
        if ss -tlnp 2>/dev/null | grep -q ':3333'; then
            break
        fi
        if ! kill -0 "$OPENOCD_PID" 2>/dev/null; then
            fail "OpenOCD exited unexpectedly:"
            tail -5 "$OPENOCD_LOG" | sed 's/^/      /'
            exit 1
        fi
        sleep 0.2
    done

    if ! ss -tlnp 2>/dev/null | grep -q ':3333'; then
        fail "OpenOCD failed to open port 3333 (timeout)"
        tail -10 "$OPENOCD_LOG" | sed 's/^/      /'
        exit 1
    fi
    ok "OpenOCD ready (PID $OPENOCD_PID, log: $OPENOCD_LOG)"
fi

# ── 6. Flash (optional) ─────────────────────────────────────────
if $DO_FLASH; then
    msg "Flashing firmware..."
    if STM32_Programmer_CLI -c port=SWD -d "$ELF" -v -rst > /tmp/ssb-flash.log 2>&1; then
        ok "Flash complete"
    else
        # Try OpenOCD flash as fallback
        warn "STM32_Programmer_CLI failed, trying OpenOCD flash..."
        GDB_FLASH=$(arm-none-eabi-gdb --batch \
            -ex "target extended-remote :3333" \
            -ex "monitor reset halt" \
            -ex "load" \
            -ex "monitor reset run" \
            -ex "quit" \
            "$ELF" 2>&1)
        if echo "$GDB_FLASH" | grep -q "Transfer rate"; then
            ok "Flashed via GDB/OpenOCD"
        else
            fail "Flash failed"
            echo "$GDB_FLASH" | tail -5 | sed 's/^/      /'
            exit 1
        fi
    fi
fi

# ── 7. Launch GDB ───────────────────────────────────────────────
echo ""

if $HEALTH_ONLY; then
    msg "Running health check..."
    arm-none-eabi-gdb --batch \
        -ex "target extended-remote :3333" \
        -ex "file \"$ELF\"" \
        -ex "monitor reset halt" \
        -ex "load" \
        -ex "monitor reset run" \
        -ex "shell sleep 2" \
        -ex "monitor halt" \
        -ex "source $SCRIPT_DIR/gdb-dashboard.py" \
        -ex "ssb health" \
        -ex "quit" 2>&1 | grep -v "^Reading\|^Loading\|^Transfer\|^Start address\|^Remote"
    echo ""
else
    echo -e "${BOLD}Launching GDB with debug dashboard...${RESET}"
    echo -e "${DIM}  Type 'ssb' for overview, 'ssb help' for all commands${RESET}"
    echo -e "${DIM}  Type 'ssb watch' for live monitoring${RESET}"
    echo -e "${DIM}  Type 'quit' to exit (OpenOCD stops automatically)${RESET}"
    echo ""

    cd "$PROJECT_ROOT"
    arm-none-eabi-gdb -x scripts/gdb-init.gdb "$ELF"
fi
