#!/usr/bin/env bash
set -Ee -o pipefail

if [[ $# -ne 1 || "$1" != /* ]]; then
    echo "usage: $0 /absolute/fast-output-dir" >&2
    exit 2
fi

TARGET_DIRS=(
    "$1/Log/Colmap/images"
    "$1/Log/Colmap/sparse/0"
)

for dir in "${TARGET_DIRS[@]}"; do
    mkdir -p "$dir"
    echo "Ready: $dir"
done
