#!/bin/bash
# Nav2 Configuration Verification Script

echo "========================================="
echo "Nav2 Configuration Verification"
echo "========================================="
echo ""

# Check if ROS2 is running
if ! pgrep -x "ros2" > /dev/null; then
    echo "⚠️  Warning: No ROS2 processes detected. Launch your simulation first."
    echo ""
fi

echo "1. Checking Nav2 parameter files..."
echo "-----------------------------------"

# Check nav2_params_working.yaml for critical fixes
echo -n "✓ Robot footprint (should be rectangular): "
grep -A1 "footprint:" /home/aun/Downloads/smr300l_gazebo_ros2control-main/config/nav2_params_working.yaml | grep "\[\[" | head -1

echo -n "✓ Dual lidar in local_costmap: "
grep -A5 "observation_sources:" /home/aun/Downloads/smr300l_gazebo_ros2control-main/config/nav2_params_working.yaml | grep "scan scan2" | head -1

echo -n "✓ Max velocity: "
grep "max_vel_x:" /home/aun/Downloads/smr300l_gazebo_ros2control-main/config/nav2_params_working.yaml | grep -v "#" | head -1

echo -n "✓ Max rotation: "
grep "max_vel_theta:" /home/aun/Downloads/smr300l_gazebo_ros2control-main/config/nav2_params_working.yaml | grep -v "#" | head -1

echo -n "✓ Trajectory samples (vtheta): "
grep "vtheta_samples:" /home/aun/Downloads/smr300l_gazebo_ros2control-main/config/nav2_params_working.yaml

echo -n "✓ A* planner enabled: "
grep "use_astar:" /home/aun/Downloads/smr300l_gazebo_ros2control-main/config/nav2_params_working.yaml

echo -n "✓ Inflation radius (local): "
grep "inflation_radius:" /home/aun/Downloads/smr300l_gazebo_ros2control-main/config/nav2_params_working.yaml | head -1

echo ""
echo "2. Checking launch files..."
echo "-----------------------------------"

if [ -f "/home/aun/Downloads/smr300l_gazebo_ros2control-main/launch/navigation.launch.py" ]; then
    echo "✓ navigation.launch.py exists"
    echo -n "  - Contains controller_server: "
    grep -q "controller_server" /home/aun/Downloads/smr300l_gazebo_ros2control-main/launch/navigation.launch.py && echo "Yes" || echo "No"
    echo -n "  - Contains planner_server: "
    grep -q "planner_server" /home/aun/Downloads/smr300l_gazebo_ros2control-main/launch/navigation.launch.py && echo "Yes" || echo "No"
    echo -n "  - Contains bt_navigator: "
    grep -q "bt_navigator" /home/aun/Downloads/smr300l_gazebo_ros2control-main/launch/navigation.launch.py && echo "Yes" || echo "No"
else
    echo "❌ navigation.launch.py NOT FOUND!"
fi

echo ""
echo "3. Checking zone_nav_ui includes navigation..."
echo "-----------------------------------"
grep -q "navigation.launch.py" /home/aun/Downloads/smr300l_gazebo_ros2control-main/launch/zone_nav_ui.launch.py && echo "✓ Includes navigation.launch.py" || echo "❌ Missing navigation include"

echo ""
echo "4. Checking diagnostics endpoints..."
echo "-----------------------------------"
grep -q "/api/diagnostics/nav2" /home/aun/Downloads/smr300l_gazebo_ros2control-main/src/zone_nav/zone_nav/zone_web_ui.py && echo "✓ Nav2 diagnostics endpoint added" || echo "❌ Missing diagnostics"

echo ""
echo "========================================="
echo "Configuration Check Complete!"
echo "========================================="
echo ""
echo "To test after launching:"
echo "  1. Launch simulation with zone_nav_ui.launch.py"
echo "  2. Check Nav2 status:"
echo "     curl http://localhost:5000/api/diagnostics/nav2 | jq"
echo "  3. Check obstacles:"
echo "     curl http://localhost:5000/api/diagnostics/obstacles | jq"
echo "  4. Verify action server:"
echo "     ros2 action list | grep navigate_to_pose"
echo ""
