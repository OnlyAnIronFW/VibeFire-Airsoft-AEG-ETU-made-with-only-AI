#!/bin/bash
#
# setup_rk3566.sh — One-time setup for Radxa Zero 3E (RK3566) running Armbian/Debian.
#
# Run this ON the board:
#   chmod +x setup_rk3566.sh && ./setup_rk3566.sh

set -euo pipefail

echo "=== Installing system dependencies ==="
sudo apt update
sudo apt install -y libopencv-dev libsdl2-dev libgpiod-dev

echo ""
echo "=== Loading rknpu kernel module ==="
if sudo modprobe rknpu 2>/dev/null; then
    echo "rknpu module loaded."
else
    echo "WARNING: Could not load rknpu module. It may already be loaded or not available."
    echo "         Ensure the kernel includes the rknpu driver."
fi

echo ""
echo "=== Enabling rknpu at boot ==="
if ! grep -q "^rknpu$" /etc/modules 2>/dev/null; then
    echo "rknpu" | sudo tee -a /etc/modules > /dev/null
    echo "Added 'rknpu' to /etc/modules."
else
    echo "'rknpu' already in /etc/modules."
fi

echo ""
echo "=== Setup complete ==="
echo "A reboot is recommended for the NPU driver to fully take effect:"
echo "  sudo reboot"
