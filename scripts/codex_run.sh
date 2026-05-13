#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT/build"

PICO_HOME="${PICO_HOME:-$HOME/.pico-sdk}"
CMAKE="${CMAKE:-$PICO_HOME/cmake/v3.31.5/bin/cmake}"
NINJA="${NINJA:-$PICO_HOME/ninja/v1.12.1/ninja}"
PICOTOOL="${PICOTOOL:-$PICO_HOME/picotool/2.2.0-a4/picotool/picotool}"

if [[ ! -x "$CMAKE" ]]; then
  CMAKE="$(command -v cmake)"
fi
if [[ ! -x "$NINJA" ]]; then
  NINJA="$(command -v ninja)"
fi
if [[ ! -x "$PICOTOOL" ]]; then
  PICOTOOL="$(command -v picotool)"
fi

export PICO_SDK_PATH="${PICO_SDK_PATH:-$PICO_HOME/sdk/2.2.0}"
export PICO_TOOLCHAIN_PATH="${PICO_TOOLCHAIN_PATH:-$PICO_HOME/toolchain/14_2_Rel1}"
export PATH="$PICO_TOOLCHAIN_PATH/bin:$PICO_HOME/picotool/2.2.0-a4/picotool:$PICO_HOME/cmake/v3.31.5/bin:$PICO_HOME/ninja/v1.12.1:$PATH"

echo "==> Configuring Pico firmware"
"$CMAKE" -S "$ROOT" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release

echo "==> Building Pico firmware"
"$NINJA" -C "$BUILD_DIR"

echo "==> Detecting connected target and flashing matching firmware"
python3 "$ROOT/scripts/flash_auto.py" --build-dir "$BUILD_DIR" --picotool "$PICOTOOL"
