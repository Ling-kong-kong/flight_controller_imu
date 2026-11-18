#!/usr/bin/env bash
# Usage: ./scripts/build.sh [build-directory] [-DNAME=VALUE ...]
# Default output: build/imu_firmware.{elf,bin,hex,map}, starting at 0x08000000.
set -euo pipefail

task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
task_build_dir="$task_root/build"
if [[ $# -gt 0 && $1 != -* ]]; then
    task_build_dir=$1
    shift
fi
task_generator="Unix Makefiles"
if command -v ninja >/dev/null 2>&1; then
    task_generator=Ninja
fi

python3 "$task_root/scripts/bootstrap_deps.py"
cmake -S "$task_root" -B "$task_build_dir" -G "$task_generator" \
    -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build "$task_build_dir" --parallel "${JOBS:-8}"
