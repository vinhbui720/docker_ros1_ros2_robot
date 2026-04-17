#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.motomini"

# Auto-create venv if missing
if [ ! -f "$VENV_DIR/bin/python" ]; then
    echo "Virtual environment not found. Running setup first..."
    bash "$SCRIPT_DIR/setup_env.sh"
fi

exec "$VENV_DIR/bin/python" "$SCRIPT_DIR/docker_tui.py"