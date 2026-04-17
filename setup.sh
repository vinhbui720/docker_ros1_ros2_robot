#!/usr/bin/env bash
set -e

VENV_DIR="$(dirname "$0")/.venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment at $VENV_DIR ..."
    python3 -m venv "$VENV_DIR"
fi

echo "Installing dependencies..."
"$VENV_DIR/bin/pip" install --upgrade pip -q
"$VENV_DIR/bin/pip" install -r "$(dirname "$0")/requirements.txt"

echo ""
echo "Done. Run the TUI with:"
echo "  $VENV_DIR/bin/python docker_tui.py"
echo ""
echo "Or activate the venv first:"
echo "  source $VENV_DIR/bin/activate"
echo "  python docker_tui.py"