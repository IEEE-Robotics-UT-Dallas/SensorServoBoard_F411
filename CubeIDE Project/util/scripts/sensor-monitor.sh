#!/usr/bin/env bash
# sensor-monitor.sh — Continuous sensor validation for SensorServoBoard
# Usage: bash scripts/sensor-monitor.sh [--duration SECS] [--hz]
set -euo pipefail

DURATION=0  # 0 = infinite
SHOW_HZ=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --hz)       SHOW_HZ=true; shift ;;
    -h|--help)
      echo "Usage: $0 [--duration SECS] [--hz]"
      echo "  --duration N   Run for N seconds (default: infinite)"
      echo "  --hz           Show publish rates"
      exit 0 ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

ROS_IMG="ros:humble"
NET="--net=host"
SH="source /opt/ros/humble/setup.bash"

header() {
  printf '\n\033[1;36m%s\033[0m\n' "$1"
}

# --- Topic availability check ---
header "Checking topics..."
TOPICS=$(docker run --rm $NET $ROS_IMG bash -c "$SH && ros2 topic list 2>/dev/null")
EXPECTED=("/tof" "/imu/mag" "/light" "/telemetry" "/servo_pos" "/servo_cmd")
ALL_OK=true
for t in "${EXPECTED[@]}"; do
  if echo "$TOPICS" | grep -qx "$t"; then
    printf "  ✅ %s\n" "$t"
  else
    printf "  ❌ %s MISSING\n" "$t"
    ALL_OK=false
  fi
done
$ALL_OK || { echo "Some topics missing — is the agent running?"; exit 1; }

# --- Publish rate check ---
if $SHOW_HZ; then
  header "Publish rates (5s sample)..."
  for t in /imu/mag /light /telemetry /tof /servo_pos; do
    HZ=$(docker run --rm $NET $ROS_IMG bash -c "$SH && timeout 8 ros2 topic hz $t --window 5 2>&1 | grep 'average rate' | tail -1" 2>/dev/null || echo "  (no data)")
    printf "  %-14s %s\n" "$t" "$HZ"
  done
fi

# --- Sensor data validation ---
header "Sensor data snapshot..."

echo "  ── Magnetometer ──"
MAG=$(docker run --rm $NET $ROS_IMG bash -c "$SH && timeout 5 ros2 topic echo /imu/mag --once 2>/dev/null")
MAG_X=$(echo "$MAG" | grep "x:" | head -1 | awk '{print $2}')
MAG_Y=$(echo "$MAG" | grep "y:" | head -1 | awk '{print $2}')
MAG_Z=$(echo "$MAG" | grep "z:" | head -1 | awk '{print $2}')
MAG_SEC=$(echo "$MAG" | grep "sec:" | head -1 | awk '{print $2}')
printf "    x=%.0f  y=%.0f  z=%.0f\n" "$MAG_X" "$MAG_Y" "$MAG_Z"

# Validate mag timestamp
if [[ "$MAG_SEC" != "0" ]]; then
  printf "    ✅ timestamp: sec=%s (synced)\n" "$MAG_SEC"
else
  printf "    ⚠️  timestamp: sec=0 (not synced — using boot time)\n"
fi

# Validate mag range (Earth's field: ~250-650 µT total, raw counts typically 100-5000)
MAG_OK=true
for v in "$MAG_X" "$MAG_Y" "$MAG_Z"; do
  ABS=$(echo "$v" | tr -d '-' | cut -d. -f1)
  if [[ "$ABS" -gt 10000 ]]; then
    echo "    ❌ Mag value $v out of range (>10000)"
    MAG_OK=false
  fi
done
$MAG_OK && echo "    ✅ Mag values in expected range"

echo "  ── Light Sensor ──"
LIGHT=$(docker run --rm $NET $ROS_IMG bash -c "$SH && timeout 5 ros2 topic echo /light --once 2>/dev/null" | grep "data:" | awk '{print $2}')
printf "    lux=%.1f\n" "$LIGHT"
LIGHT_INT=${LIGHT%.*}
if [[ "$LIGHT_INT" -lt 65535 ]]; then
  echo "    ✅ Light not saturated"
else
  echo "    ❌ Light saturated (65535)"
fi

echo "  ── ToF Distances ──"
TOF=$(docker run --rm $NET $ROS_IMG bash -c "$SH && timeout 5 ros2 topic echo /tof --once 2>/dev/null")
echo "$TOF" | grep "^- " | while read -r line; do
  printf "    %s m\n" "$line"
done

echo "  ── Servo Positions ──"
SERVO=$(docker run --rm $NET $ROS_IMG bash -c "$SH && timeout 5 ros2 topic echo /servo_pos --once 2>/dev/null")
echo "$SERVO" | grep "^- " | while read -r line; do
  printf "    %s°\n" "$line"
done

echo "  ── Telemetry ──"
TELEM=$(docker run --rm $NET $ROS_IMG bash -c "$SH && timeout 5 ros2 topic echo /telemetry --once 2>/dev/null")
echo "$TELEM" | grep "^- " | head -9 | cat -n | while read -r n line; do
  case $n in
    1|2|3|4|5) label="tof_$((n-1))" ;;
    6) label="mag_x " ;;
    7) label="mag_y " ;;
    8) label="mag_z " ;;
    9) label="lux   " ;;
  esac
  printf "    [%d] %s = %s\n" "$((n-1))" "$label" "$line"
done

# --- Stability check (2 samples, compare) ---
header "Stability check (2 readings, 1s apart)..."
R1=$(docker run --rm $NET $ROS_IMG bash -c "$SH && timeout 5 ros2 topic echo /imu/mag --once 2>/dev/null")
sleep 1
R2=$(docker run --rm $NET $ROS_IMG bash -c "$SH && timeout 5 ros2 topic echo /imu/mag --once 2>/dev/null")

get_field() { echo "$1" | grep "$2:" | head -1 | awk '{print $2}' | cut -d. -f1; }
DX=$(( $(get_field "$R2" "x") - $(get_field "$R1" "x") ))
DY=$(( $(get_field "$R2" "y") - $(get_field "$R1" "y") ))
DZ=$(( $(get_field "$R2" "z") - $(get_field "$R1" "z") ))
# Use absolute values
DX=${DX#-}; DY=${DY#-}; DZ=${DZ#-}

printf "  Mag delta: dx=%d dy=%d dz=%d\n" "$DX" "$DY" "$DZ"
if [[ $DX -lt 200 && $DY -lt 200 && $DZ -lt 200 ]]; then
  echo "  ✅ Readings stable"
else
  echo "  ⚠️  Readings drifting (delta > 200)"
fi

header "Done ✓"
