# SMR300L Gazebo ROS2 Control

A comprehensive ROS2 simulation environment for the SMR300L mobile robot with advanced navigation capabilities, including a web-based UI for zone-based navigation.

## Overview

This project provides a complete simulation stack for the SMR300L autonomous mobile robot using:
- **ROS2** (Robot Operating System 2)
- **ros2_control** for hardware abstraction
- **Gazebo Classic** for physics simulation
- **Nav2** for autonomous navigation
- **SLAM Toolbox** for mapping
- **Web UI** for intuitive zone-based navigation control

## Features

### Core Functionality
- ✅ Full robot simulation in Gazebo with realistic physics
- ✅ ROS2 control integration for differential drive control
- ✅ SLAM capability for environment mapping
- ✅ Autonomous navigation with Nav2 stack
- ✅ Joystick/keyboard teleoperation support
- ✅ RViz2 visualization

### Zone Navigation System
- 🎯 **Web-based UI** for robot control and monitoring
- 📍 **Save named zones** by setting goals in RViz
- 🚀 **Navigate to zones** with a single click
- 🗺️ **Visual map display** with robot position overlay
- 📊 **Real-time robot position** tracking
- 💾 **Persistent zone storage** (zones.yaml)

## System Architecture

### Packages Structure

```
smr300l_gazebo_ros2control/
├── my_bot/                    # Main robot package
│   ├── config/                # Configuration files
│   │   ├── my_controllers.yaml
│   │   ├── nav2_params.yaml
│   │   ├── joystick.yaml
│   │   └── twist_mux.yaml
│   ├── description/           # Robot URDF/Xacro files
│   │   ├── robot.urdf.xacro
│   │   ├── ros2_control.xacro
│   │   ├── gazebo_control.xacro
│   │   └── lidar.xacro
│   ├── launch/                # Launch files
│   │   ├── launch_sim.launch.py
│   │   ├── rsp.launch.py
│   │   └── zone_nav_ui.launch.py
│   ├── maps/                  # Pre-made maps
│   └── worlds/                # Gazebo world files
│
└── src/
    ├── zone_nav/              # Zone navigation package
    │   └── zone_nav/
    │       ├── zone_manager.py      # Zone management node
    │       ├── zone_web_ui.py       # Flask web server
    │       └── web/templates/
    │           └── index.html       # Web interface
    │
    └── zone_nav_interfaces/   # Custom service definitions
        └── srv/
            ├── SaveZone.srv
            └── GoToZone.srv
```

## Prerequisites

- ROS2 (Humble/Iron/Jazzy recommended)
- Gazebo Classic
- Nav2
- SLAM Toolbox
- Python 3.8+
- Flask (for web UI)

## Installation

### 1. Clone the Repository

```bash
cd ~/Downloads
git clone https://github.com/00PrabalK00/smr300l_gazebo_ros2control.git
cd smr300l_gazebo_ros2control
```

### 2. Install Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install ros-<distro>-gazebo-ros-pkgs ros-<distro>-gazebo-ros2-control
sudo apt install ros-<distro>-nav2-bringup ros-<distro>-slam-toolbox
sudo apt install ros-<distro>-twist-mux ros-<distro>-teleop-twist-keyboard

# Install web UI dependencies
./install_web_ui_deps.sh
# or manually:
pip3 install flask flask-cors pillow pyyaml
```

### 3. Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Quick Start (Full System)

Launch everything in separate terminals:

#### Terminal 1: Simulation Environment
```bash
source install/setup.bash
export GAZEBO_MODEL_DATABASE_URI=""
ros2 launch my_bot launch_sim.launch.py use_sim_time:=true
```

#### Terminal 2: Navigation Stack
```bash
source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=./maps/smr_map.yaml \
  params_file:=./config/nav2_params.yaml
```

#### Terminal 3: Zone Navigation Web UI
```bash
source install/setup.bash
ros2 launch my_bot zone_nav_ui.launch.py
```

#### Terminal 4: RViz2 (optional)
```bash
source install/setup.bash
ros2 run rviz2 rviz2 -d config/robot.rviz --ros-args -p use_sim_time:=true
```

### Creating a New Map (SLAM)

If you need to map a new environment:

```bash
# Launch only the simulation (Terminal 1)
ros2 launch my_bot launch_sim.launch.py use_sim_time:=true

# Launch SLAM Toolbox (Terminal 2) - DO NOT run during navigation!
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Teleoperate the robot (Terminal 3)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_joy

# Save the map when complete
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Using the Web UI

1. Open browser to: `http://localhost:5000`
2. **Save a Zone:**
   - Set a goal pose in RViz using "2D Goal Pose" tool
   - Enter a zone name in the web UI
   - Click "Save Zone"
3. **Navigate to Zone:**
   - Select a saved zone from the dropdown
   - Click "Go to Zone"
4. **Monitor:**
   - View robot position in real-time on the map overlay
   - Check zone list and navigation status

## Configuration

### Robot Controllers

Edit [config/my_controllers.yaml](config/my_controllers.yaml) to modify:
- Differential drive parameters
- Joint state broadcaster settings
- Control loop rates

### Navigation Parameters

Edit [config/nav2_params.yaml](config/nav2_params.yaml) to tune:
- Path planning algorithms
- Obstacle avoidance behavior
- Costmap parameters
- Recovery behaviors

### Joystick/Keyboard Control

Edit [config/joystick.yaml](config/joystick.yaml) for input device configuration.

## Web API Endpoints

The zone navigation web server provides:

- `GET /` - Web interface
- `GET /api/zones` - List all saved zones
- `POST /api/save_zone` - Save current goal as a zone
- `POST /api/go_to_zone` - Navigate to a named zone
- `GET /api/robot_position` - Get current robot pose
- `GET /api/map_info` - Get map metadata
- `GET /api/map_image` - Get map image with robot overlay

## Troubleshooting

### Robot Not Moving
- Check controller status: `ros2 control list_controllers`
- Verify cmd_vel topic: `ros2 topic echo /diff_cont/cmd_vel_unstamped`

### Navigation Not Working
- Ensure SLAM Toolbox is NOT running during navigation
- Check if Nav2 is receiving goals: `ros2 topic echo /goal_pose`
- Verify localization: `ros2 topic echo /amcl_pose`

### Web UI Not Loading
- Check if Flask server is running on port 5000
- Verify ROS2 nodes: `ros2 node list`
- Check zone_manager and zone_web_server nodes are active

### AMCL Pose Not Publishing
- Verify QoS settings (RELIABLE + TRANSIENT_LOCAL)
- Check map is loaded: `ros2 topic info /map`
- Set initial pose in RViz using "2D Pose Estimate"

## Screenshots

### Gazebo Simulation
![Screenshot from 2025-03-06 09-52-07](https://github.com/user-attachments/assets/a3ea3530-7bc5-48e0-b5b3-68bb90ae6570)

### RVIZ2 Visualization
![Screenshot from 2025-03-06 09-54-22](https://github.com/user-attachments/assets/20897ce1-d6e8-4fbd-92b8-95e01e1c21a6)

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the terms specified in the LICENSE file.

## Acknowledgments

- ROS2 community for excellent documentation
- Nav2 team for the navigation stack
- Gazebo simulation environment
- SLAM Toolbox developers

## Contact

For questions or issues, please open an issue on the GitHub repository.
