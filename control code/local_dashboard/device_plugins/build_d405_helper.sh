#!/bin/sh
set -eu

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd -P)
env="$root/.venv"
helper="$env/bin/ophis-d405-capture"
output=$(mktemp "$env/bin/.ophis-d405-capture.XXXXXX")
trap 'rm -f "$output"' EXIT

/usr/bin/clang++ -std=c++17 -O2 -I"$env/include" "$root/device_plugins/d405_capture.cpp" \
    -L"$env/lib" -lrealsense2 -Wl,-rpath,@executable_path/../lib -o "$output"
chmod 755 "$output"
mv -f "$output" "$helper"
printf '%s\n' "$helper"
