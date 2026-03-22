#!/usr/bin/env bash
# run_tests.sh — Convenience wrapper for host-side unit tests
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
make -C "$SCRIPT_DIR" test_all
