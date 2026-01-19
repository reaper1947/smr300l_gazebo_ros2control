#!/bin/bash
# Wrapper for nav2_bringup that reads active map from config
# Usage: ./nav2_launch_wrapper.sh [other args]

# Get active map from config
CONFIG_FILE="/home/aun/Downloads/smr300l_gazebo_ros2control-main/active_map_config.yaml"
DEFAULT_MAP="/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps/smr_map.yaml"
PARAMS_FILE="/home/aun/Downloads/smr300l_gazebo_ros2control-main/config/nav2_params.yaml"

# Read active map from config, fallback to default
if [ -f "$CONFIG_FILE" ]; then
    ACTIVE_MAP=$(grep "active_map:" "$CONFIG_FILE" | awk '{print $2}' | tr -d "'\"")
    if [ -z "$ACTIVE_MAP" ]; then
        ACTIVE_MAP="$DEFAULT_MAP"
    fi
else
    ACTIVE_MAP="$DEFAULT_MAP"
fi

echo "🗺️  Using map: $ACTIVE_MAP"

# Launch Nav2 with active map (don't override if user already provided map)
if [[ "$@" == *"map:="* ]]; then
    # User provided map argument, use their values
    ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true params_file:=$PARAMS_FILE "$@"
else
    # Use active map from config
    ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=$ACTIVE_MAP params_file:=$PARAMS_FILE "$@"
fi
