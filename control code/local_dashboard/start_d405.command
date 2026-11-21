#!/bin/zsh
set -eu
cd "${0:A:h}"
exec "$PWD/.venv/bin/python" "$PWD/launch_d405.py" "$@"
