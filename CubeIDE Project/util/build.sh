#!/bin/bash
# ── SensorServoBoard F411 — Build & Flash ────────────────────────
#
# Usage:
#   ./build.sh              Build (incremental)
#   ./build.sh clean        Clean + rebuild from scratch
#   ./build.sh flash        Build + flash via probe-rs
#   ./build.sh flash clean  Clean + rebuild + flash
#   ./build.sh debug        Build + launch TUI debugger
#   ./build.sh hwtest       Build with HW_TEST (no micro-ROS)
#
set -uo pipefail  # no -e: CubeIDE returns non-zero for warnings-as-errors

# ── Configuration ────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_PATH="$SCRIPT_DIR/CubeIDE Project/SensorServorBoard"
PROJECT_NAME="SensorServorBoard"
BUILD_CONFIG="Debug"
WORKSPACE_PATH="/tmp/sensorservo_headless_ws"
CUBE_IDE_PATH="/usr/bin/stm32cubeide"
ELF_FILE="$PROJECT_PATH/$BUILD_CONFIG/$PROJECT_NAME.elf"
CHIP="STM32F411CEUx"
TUI_DEBUGGER="$HOME/stm32-tui-debugger-outline/target/release/stm32-tui-debugger"

# STM32F411 specs
FLASH_TOTAL=524288
RAM_TOTAL=131072

# ── Colors ───────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m'

ok()   { echo -e "  ${GREEN}✓${NC} $*"; }
fail() { echo -e "  ${RED}✗${NC} $*"; }
info() { echo -e "  ${CYAN}▸${NC} $*"; }

# ── Parse args ───────────────────────────────────────────────────
DO_CLEAN=false
DO_FLASH=false
DO_DEBUG=false
DO_HWTEST=false

for arg in "$@"; do
    case "$arg" in
        clean)   DO_CLEAN=true ;;
        flash)   DO_FLASH=true ;;
        debug)   DO_DEBUG=true ;;
        hwtest)  DO_HWTEST=true ;;
        -h|--help)
            echo -e "${BOLD}SensorServoBoard F411 — Build & Flash${NC}"
            echo ""
            echo "  $0              Build (incremental)"
            echo "  $0 clean        Clean + full rebuild"
            echo "  $0 flash        Build + flash via probe-rs"
            echo "  $0 flash clean  Clean rebuild + flash"
            echo "  $0 debug        Build + launch TUI debugger"
            echo "  $0 hwtest       Build with HW_TEST (no micro-ROS)"
            echo ""
            exit 0
            ;;
        *)
            echo "Unknown option: $arg (try --help)"
            exit 1
            ;;
    esac
done

# ── Preflight ────────────────────────────────────────────────────
if [ ! -f "$CUBE_IDE_PATH" ]; then
    fail "STM32CubeIDE not found at $CUBE_IDE_PATH"
    exit 1
fi

# ── HW_TEST mode: swap defines ──────────────────────────────────
CPROJECT="$PROJECT_PATH/.cproject"
RESTORE_DEFINES=false

if $DO_HWTEST; then
    info "Configuring for HW_TEST build (micro-ROS disabled)..."
    sed -i 's/value="MICRO_ROS_ENABLED"/value="__DISABLED_MICRO_ROS_ENABLED__"/g' "$CPROJECT"
    sed -i 's/value="RMW_UXRCE_TRANSPORT_CUSTOM"/value="HW_TEST"/g' "$CPROJECT"
    RESTORE_DEFINES=true
    DO_CLEAN=true  # Force clean when switching modes
fi

# ── Clean ────────────────────────────────────────────────────────
if $DO_CLEAN; then
    info "Cleaning build artifacts..."
    rm -rf "$PROJECT_PATH/$BUILD_CONFIG"
    rm -rf "$WORKSPACE_PATH"
    ok "Clean"
fi

# ── Build ────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}Building $PROJECT_NAME ($BUILD_CONFIG)...${NC}"
mkdir -p "$WORKSPACE_PATH"

"$CUBE_IDE_PATH" -nosplash \
    -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
    -data "$WORKSPACE_PATH" \
    -import "$PROJECT_PATH" \
    -build "$PROJECT_NAME/$BUILD_CONFIG"

BUILD_EXIT=$?

# ── Restore defines if HW_TEST ──────────────────────────────────
if $RESTORE_DEFINES; then
    sed -i 's/value="__DISABLED_MICRO_ROS_ENABLED__"/value="MICRO_ROS_ENABLED"/g' "$CPROJECT"
    sed -i 's/value="HW_TEST"/value="RMW_UXRCE_TRANSPORT_CUSTOM"/g' "$CPROJECT"
fi

# ── Check result ─────────────────────────────────────────────────
if [ ! -f "$ELF_FILE" ]; then
    fail "Build failed — no ELF produced"
    exit 1
fi

# ── Size report ──────────────────────────────────────────────────
echo ""
ok "Build successful"
echo -e "  ${DIM}ELF: $ELF_FILE${NC}"
echo ""

SIZE_OUT=$(arm-none-eabi-size "$ELF_FILE" 2>/dev/null | tail -1)
TEXT=$(echo "$SIZE_OUT" | awk '{print $1}')
DATA=$(echo "$SIZE_OUT" | awk '{print $2}')
BSS=$(echo "$SIZE_OUT" | awk '{print $3}')
RAM_USED=$((DATA + BSS))
FLASH_USED=$((TEXT + DATA))
RAM_PCT=$((RAM_USED * 100 / RAM_TOTAL))
FLASH_PCT=$((FLASH_USED * 100 / FLASH_TOTAL))

printf "  %-8s %6d / %6d bytes  (%2d%%)\n" "Flash:" "$FLASH_USED" "$FLASH_TOTAL" "$FLASH_PCT"
printf "  %-8s %6d / %6d bytes  (%2d%%)\n" "RAM:" "$RAM_USED" "$RAM_TOTAL" "$RAM_PCT"

# Warn if RAM is tight
if [ "$RAM_PCT" -gt 85 ]; then
    echo -e "  ${YELLOW}⚠ RAM usage above 85% — watch for heap exhaustion${NC}"
fi

# ── Flash ────────────────────────────────────────────────────────
if $DO_FLASH; then
    echo ""
    if ! command -v probe-rs &>/dev/null; then
        fail "probe-rs not found — install it: curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh"
        exit 1
    fi
    info "Flashing via probe-rs..."
    probe-rs download --chip "$CHIP" "$ELF_FILE"
    if [ $? -eq 0 ]; then
        ok "Flash complete"
        info "Resetting target..."
        probe-rs reset --chip "$CHIP" 2>/dev/null || true
        ok "Board running"
    else
        fail "Flash failed"
        exit 1
    fi
fi

# ── Debug ────────────────────────────────────────────────────────
if $DO_DEBUG; then
    echo ""
    if [ ! -f "$TUI_DEBUGGER" ]; then
        fail "TUI debugger not found at $TUI_DEBUGGER"
        echo "  Build it: cd ~/stm32-tui-debugger-outline && cargo build --release"
        exit 1
    fi
    info "Launching TUI debugger..."
    exec "$TUI_DEBUGGER" -C "$SCRIPT_DIR/debugger.toml"
fi
