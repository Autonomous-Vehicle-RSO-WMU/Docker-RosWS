#!/usr/bin/env bash
# Creates a Python venv at ./venv with all robot stack dependencies.
#
# --system-site-packages lets the venv see ROS2 packages installed at
# /opt/ros/humble/ (rclpy, cv_bridge, etc.) without copying them in.
#
# Usage:
#   bash setup_venv.sh          # create/recreate the venv
#   source venv/bin/activate    # activate manually (optional)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
JETSON_PYPI="https://pypi.jetson-ai-lab.io/jp6/cu126"

echo "=== Creating venv at $VENV_DIR ==="
python3 -m venv --system-site-packages "$VENV_DIR"

echo "=== Upgrading pip ==="
"$VENV_DIR/bin/pip" install --upgrade pip

echo "=== Installing PyTorch from Jetson AI Lab ==="
"$VENV_DIR/bin/pip" install \
    torch==2.8.0 torchvision==0.23.0 \
    --index-url "$JETSON_PYPI"

echo "=== Installing remaining requirements ==="
# ultralytics is fetched from PyPI but allowed to pull torch from Jetson index
"$VENV_DIR/bin/pip" install \
    --index-url https://pypi.org/simple \
    --extra-index-url "$JETSON_PYPI" \
    -r "$SCRIPT_DIR/requirements.txt"

echo ""
echo "=== Done ==="
echo "The launch script uses this venv automatically."
echo "To activate manually: source venv/bin/activate"
