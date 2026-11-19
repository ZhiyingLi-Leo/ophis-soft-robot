#!/bin/zsh
set -eu
cd "${0:A:h}"
envdir="$PWD/.venv"
helper="$envdir/bin/ophis-d405-capture"
source_file="$PWD/device_plugins/d405_capture.cpp"
/usr/bin/clang++ -std=c++17 -O2 -I"$envdir/include" "$source_file" -L"$envdir/lib" -lrealsense2 -Wl,-rpath,"$envdir/lib" -o "$helper"
exec /usr/bin/sudo -- "$helper" "$@"
