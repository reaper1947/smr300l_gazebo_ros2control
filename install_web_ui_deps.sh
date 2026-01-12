#!/bin/bash
# Check and install web UI dependencies

echo "=== Zone Navigation Web UI - Dependency Checker ==="
echo ""

# Check Python version
echo "Checking Python version..."
python3 --version
echo ""

# Check if pip is installed
if ! command -v pip3 &> /dev/null; then
    echo "❌ pip3 not found. Please install: sudo apt install python3-pip"
    exit 1
fi
echo "✅ pip3 found"
echo ""

# Check/install dependencies
echo "Checking Python packages..."
packages=("flask" "flask-cors" "pillow" "pyyaml")
missing=()

for pkg in "${packages[@]}"; do
    if python3 -c "import ${pkg//-/_}" 2>/dev/null; then
        echo "✅ $pkg installed"
    else
        echo "❌ $pkg missing"
        missing+=("$pkg")
    fi
done

echo ""

if [ ${#missing[@]} -gt 0 ]; then
    echo "Installing missing packages..."
    pip3 install "${missing[@]}"
    echo ""
    echo "✅ All dependencies installed!"
else
    echo "✅ All dependencies already installed!"
fi

echo ""
echo "=== Ready to launch! ==="
echo ""
echo "Run this to start the web UI:"
echo "  source install/setup.bash"
echo "  ros2 launch my_bot zone_nav_ui.launch.py"
echo ""
echo "Then open: http://localhost:5000"
echo ""
