#!/usr/bin/env bash
set -euo pipefail

# Make sure Homebrew installs are visible even if VS Code launched without a login shell.
export PATH="/opt/homebrew/bin:/usr/local/bin:${PATH}"

ELF_PATH="${1:-target/thumbv6m-none-eabi/debug/template}"

if ! command -v probe-rs >/dev/null 2>&1; then
  echo "probe-rs is not on PATH; install it (cargo install probe-rs) before running this script." >&2
  exit 1
fi

if [ ! -f "${ELF_PATH}" ]; then
  echo "Firmware ELF not found at ${ELF_PATH}. Did the build succeed?" >&2
  exit 1
fi

echo "Flashing ${ELF_PATH} via probe-rs..."
probe-rs flash --chip RP2040 --speed 4000 --protocol swd "${ELF_PATH}"
