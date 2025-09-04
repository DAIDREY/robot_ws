#!/bin/bash

echo "🔍 ROS2 Package Diagnostic Script"
echo "=================================="

# 检查当前位置
echo "📍 Current directory: $(pwd)"

# 检查是否在正确的包目录中
if [[ $(basename $(pwd)) == "robot_visioner" ]]; then
    echo "✅ In robot_visioner package directory"
else
    echo "❌ Not in robot_visioner directory"
    echo "   Please run: cd ~/robot_ws/src/robot_visioner"
fi

echo ""
echo "📂 Package structure:"
if command -v tree &> /dev/null; then
    tree -a .
else
    find . -type f | sort
fi

echo ""
echo "📋 Checking key files:"

# 检查setup.py
if [[ -f "setup.py" ]]; then
    echo "✅ setup.py exists"
    echo "   Checking data_files configuration..."
    if grep -q "launch" setup.py; then
        echo "   ✅ Launch files configured in setup.py"
    else
        echo "   ❌ Launch files NOT configured in setup.py"
    fi
else
    echo "❌ setup.py missing"
fi

# 检查launch目录
if [[ -d "launch" ]]; then
    echo "✅ launch/ directory exists"
    launch_files=$(ls launch/*.launch.py 2>/dev/null | wc -l)
    echo "   📁 Launch files found: $launch_files"
    ls -la launch/
else
    echo "❌ launch/ directory missing"
fi

echo ""
echo "🔧 Installation check:"
install_dir="$HOME/robot_ws/install/robot_visioner/share/robot_visioner"
if [[ -d "$install_dir" ]]; then
    echo "✅ Install directory exists: $install_dir"
    echo "   📁 Installed files:"
    ls -la "$install_dir/"
    
    if [[ -d "$install_dir/launch" ]]; then
        echo "   ✅ Launch files installed:"
        ls -la "$install_dir/launch/"
    else
        echo "   ❌ Launch files NOT installed"
    fi
else
    echo "❌ Install directory does not exist"
    echo "   Run: cd ~/robot_ws && colcon build --packages-select robot_visioner"
fi
