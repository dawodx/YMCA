#!/bin/bash
# YMCA Robot Rave - Quick Setup Script
# Run: ./setup.sh

set -e

echo "========================================"
echo "  Team YMCA - Robot Rave Setup"
echo "  Making Go1 dance like Trump!"
echo "========================================"
echo ""

# Check Python version
echo "Checking Python..."
if command -v python3 &> /dev/null; then
    PYTHON=python3
elif command -v python &> /dev/null; then
    PYTHON=python
else
    echo "ERROR: Python not found! Please install Python 3.10+"
    exit 1
fi

$PYTHON --version

# Create virtual environment
echo ""
echo "Creating virtual environment..."
$PYTHON -m venv .venv

# Activate venv
echo "Activating venv..."
source .venv/bin/activate

# Upgrade pip
echo ""
echo "Upgrading pip..."
pip install --upgrade pip --quiet

# Install dependencies
echo ""
echo "Installing dependencies (this may take a few minutes)..."
pip install -r requirements.txt --quiet

echo ""
echo "========================================"
echo "  Setup Complete!"
echo "========================================"
echo ""
echo "To get started:"
echo ""
echo "  1. Activate the environment:"
echo "     source .venv/bin/activate"
echo ""
echo "  2. Run the dashboard:"
echo "     python mo_simulation/go1_dashboard.py"
echo ""
echo "  3. Or run simulation directly:"
echo "     .venv/bin/mjpython mo_simulation/run_go1_keyboard.py"
echo ""
echo "Team folders:"
echo "  - mo_simulation/    (Mo - Simulation)"
echo "  - pawit_control/   (Pawit - Robot Control)"
echo "  - alex_music/       (Alex - Music Analysis)"
echo "  - andrew_vision/    (Andrew - Computer Vision)"
echo ""
echo "Let's make this dog dance!"
echo ""
