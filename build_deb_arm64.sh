#!/usr/bin/env bash
set -e
# Thin wrapper: the arm64 build is identical to amd64 apart from the arch flag.
exec "$(dirname "$0")/build_deb.sh" arm64
