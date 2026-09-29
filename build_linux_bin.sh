#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

# PyInstaller is a project dependency, not a system tool: it lives in ./.venv
# and is invisible to a plain `make` from an unactivated shell. Resolve an
# interpreter that actually has it rather than trusting PATH.
explicit_python=0
if [ -n "${PYTHON:-}" ]; then
  py="$PYTHON"
  explicit_python=1
elif [ -n "${VIRTUAL_ENV:-}" ] && [ -x "$VIRTUAL_ENV/bin/python" ]; then
  py="$VIRTUAL_ENV/bin/python"
  explicit_python=1
elif [ -x .venv/bin/python ]; then
  py=".venv/bin/python"
else
  py="python3"
fi

has_pyinstaller() { "$1" -c 'import PyInstaller' >/dev/null 2>&1; }

# A fresh clone has no .venv, so the first build creates one. Only ever done
# for the project's own .venv: an interpreter the caller named explicitly is
# theirs to manage, and we fail loudly instead of installing into it.
if ! has_pyinstaller "$py"; then
  if [ "$explicit_python" = 1 ]; then
    echo "ERROR: PyInstaller is not installed for $py"
    echo "Install it there, or unset PYTHON/VIRTUAL_ENV to let this script build ./.venv."
    exit 1
  fi
  if [ "${NO_BOOTSTRAP:-0}" = 1 ]; then
    echo "ERROR: PyInstaller is not installed for $py and NO_BOOTSTRAP=1"
    echo "Set one up with:"
    echo "  python3 -m venv .venv && .venv/bin/pip install -r requirements.txt"
    exit 1
  fi

  echo "PyInstaller not found - bootstrapping ./.venv from requirements.txt"
  if [ ! -x .venv/bin/python ]; then
    if ! python3 -m venv .venv; then
      echo "ERROR: 'python3 -m venv .venv' failed."
      echo "On Debian/Ubuntu the venv module ships separately:"
      echo "  sudo apt-get install python3-venv python3-pip"
      rm -rf .venv
      exit 1
    fi
  fi
  .venv/bin/python -m pip install --upgrade pip
  .venv/bin/python -m pip install -r requirements.txt
  py=".venv/bin/python"

  if ! has_pyinstaller "$py"; then
    echo "ERROR: PyInstaller still missing after installing requirements.txt"
    exit 1
  fi
fi

echo "building linux bin with $("$py" -c 'import sys; print(sys.executable)')"
exec "$py" -m PyInstaller --noconfirm --windowed --hidden-import=psutil \
  --add-data="resources/icon.ico:resources/." \
  --add-data="resources/icon.png:resources/." \
  --add-data="resources/world.bin:resources/." \
  --onefile --icon=resources/icon.ico blackice_traffic.py
