#!/bin/env bash
set -e

echo "building linux bin"
pyinstaller --windowed --add-data="resources/icon.ico:resources/." --onefile --icon=resources/icon.ico blackice_traffic.py
