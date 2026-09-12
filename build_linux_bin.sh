#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

# PyInstaller is a project dependency, not a system tool: it lives in ./.venv
# and is invisible to a plain `make` from an unactivated shell. Resolve an
# interpreter that actually has it rather than trusting PATH.
if [ -n "${PYTHON:-}" ]; then
  py="$PYTHON"
elif [ -n "${VIRTUAL_ENV:-}" ] && [ -x "$VIRTUAL_ENV/bin/python" ]; then
  py="$VIRTUAL_ENV/bin/python"
elif [ -x .venv/bin/python ]; then
  py=".venv/bin/python"
else
  py="python3"
fi

if ! "$py" -c 'import PyInstaller' >/dev/null 2>&1; then
  echo "ERROR: PyInstaller is not installed for $py"
  echo "Set one up with:"
  echo "  python3 -m venv .venv && .venv/bin/pip install -r requirements.txt"
  echo "or point PYTHON= at an interpreter that has it."
  exit 1
fi

echo "building linux bin with $("$py" -c 'import sys; print(sys.executable)')"
exec "$py" -m PyInstaller --noconfirm --windowed --hidden-import=psutil \
  --add-data="resources/icon.ico:resources/." \
  --add-data="resources/icon.png:resources/." \
  --add-data="resources/world.bin:resources/." \
  --onefile --icon=resources/icon.ico blackice_traffic.py
