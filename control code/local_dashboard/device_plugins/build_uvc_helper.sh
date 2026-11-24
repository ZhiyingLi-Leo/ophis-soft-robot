#!/bin/sh
set -eu
source_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
install_dir="$source_dir/../.venv/OPHIS Side View.app"
scratch=$(mktemp -d "${TMPDIR:-/tmp}/ophis-uvc.XXXXXXXX")
trap 'rm -rf "$scratch"' EXIT
bundle="$scratch/OPHIS Side View.app"
contents="$bundle/Contents"
mkdir -p "$contents/MacOS"
cp "$source_dir/uvc_Info.plist" "$contents/Info.plist"
/usr/bin/xcrun clang -fobjc-arc -O2 -Wall -Wextra -Werror \
    -framework Foundation -framework AVFoundation -framework CoreMedia -framework CoreVideo \
    "$source_dir/uvc_capture.m" -o "$contents/MacOS/ophis-uvc-capture"
/usr/bin/codesign --force --sign - "$bundle"
/usr/bin/ditto --norsrc --noextattr "$bundle" "$install_dir"
/usr/bin/codesign --verify "$install_dir"
printf '%s\n' "$install_dir"
