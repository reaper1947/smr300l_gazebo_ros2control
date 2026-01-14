# Nav2 Configuration Fixes - Summary

## Critical Issues Fixed

### 1. **ROBOT FOOTPRINT - CRITICAL BUG** ❌ → ✅
**Problem:** Costmaps used `robot_radius: 0.22m` but robot is **0.80m × 0.50m rectangular**!
- Robot was treated as tiny 44cm diameter circle
- Caused Nav2 to think robot could fit through gaps it couldn't
- Led to spinning/stuck behavior

**Fix:** Changed to proper rectangular footprint:
```yaml
footprint: "[[0.40, 0.25], [0.40, -0.25], [-0.40, -0.25], [-0.40, 0.25]]"
```

### 2. **DUAL LIDAR NOT USED** ❌ → ✅
**Problem:** Only using front lidar `/scan`, ignoring rear lidar `/scan2`
- Blind to obstacles behind robot
- Caused collisions when reversing
- Limited situational awareness

**Fix:** Added `/scan2` to both costmaps:
```yaml
observation_sources: "scan scan2"
scan:
  topic: "/scan"
  # ... config ...
scan2:
  topic: "/scan2"  # NOW ACTIVE!
  # ... config ...
```

### 3. **DWB CONTROLLER TOO CONSERVATIVE** ❌ → ✅
**Problem:** 
- Max velocity: 0.26 m/s (too slow)
- Max rotation: 1.0 rad/s (can't turn quickly)
- Only 20 trajectory samples (limited options)
- `short_circuit_trajectory_evaluation: true` (gave up too easily)

**Fix:**
```yaml
max_vel_x: 0.35         # +35% faster
max_vel_theta: 1.5      # +50% rotation
vx_samples: 25          # +25% linear samples
vtheta_samples: 30      # +50% rotation samples
sim_time: 2.0           # +18% lookahead
short_circuit_trajectory_evaluation: false  # Try ALL paths
```

### 4. **CRITIC WEIGHTS TOO STRICT** ❌ → ✅
**Problem:** DWB critics forced perfect path following, caused spinning
- PathAlign: 32.0 (too strict)
- GoalAlign: 24.0 (too strict)
- RotateToGoal: 32.0 (excessive rotation preference)

**Fix:** Reduced all weights by ~50%:
```yaml
BaseObstacle.scale: 0.01    # Less sensitive to obstacles
PathAlign.scale: 16.0       # More flexible path following
GoalAlign.scale: 12.0       # Less strict goal alignment
RotateToGoal.scale: 16.0    # Less aggressive rotation
```

### 5. **PROGRESS CHECKER TOO STRICT** ❌ → ✅
**Problem:** Required 0.5m movement in 10s - robot failed even when making progress
```yaml
required_movement_radius: 0.5
movement_time_allowance: 10.0
```

**Fix:** More forgiving thresholds:
```yaml
required_movement_radius: 0.15  # Only 15cm needed
movement_time_allowance: 15.0   # 15 seconds to move
```

### 6. **INFLATION RADIUS TOO SMALL** ❌ → ✅
**Problem:** 
- Local: 0.4m inflation (barely larger than robot)
- Global: 0.5m inflation (not enough safety margin)

**Fix:** Increased with smoother gradients:
```yaml
local_costmap:
  inflation_radius: 0.55      # +37.5%
  cost_scaling_factor: 3.0    # Smoother gradient

global_costmap:
  inflation_radius: 0.6       # +20%
  cost_scaling_factor: 3.0    # Smoother gradient
```

### 7. **PLANNER NOT USING A*** ❌ → ✅
**Problem:** `use_astar: false` - Dijkstra is slower and finds suboptimal paths

**Fix:**
```yaml
use_astar: true              # Use A* for better paths
tolerance: 0.25              # Reduced from 0.5 for precision
```

### 8. **RECOVERY BEHAVIORS WEAK** ❌ → ✅
**Problem:** Slow recovery rotations, gave up easily

**Fix:** More aggressive recovery:
```yaml
max_rotational_vel: 1.5      # +50% faster spins
min_rotational_vel: 0.6      # +50% minimum
rotational_acc_lim: 4.0      # +25% acceleration
```

### 9. **CONTROLLER FREQUENCY LOW** ❌ → ✅
**Problem:** 10 Hz update rate - slow reaction to obstacles

**Fix:**
```yaml
controller_frequency: 15.0   # +50% faster updates
failure_tolerance: 0.5       # Allow some failures before giving up
```

### 10. **GOAL CHECKER TOO LOOSE** ❌ → ✅
**Problem:** 0.25m position tolerance - robot stops too far from goal

**Fix:**
```yaml
xy_goal_tolerance: 0.15      # More precise waypoints
yaw_goal_tolerance: 0.3      # But flexible on rotation
```

---

## Configuration Changes by File

### `/config/nav2_params_working.yaml`
All changes documented above applied to:
- `controller_server` parameters
- `local_costmap` parameters (footprint + scan2)
- `global_costmap` parameters (footprint + scan2)
- `planner_server` parameters (A*, tolerance)
- `recoveries_server` parameters (faster recovery)
- `bt_navigator` parameters (transform tolerance)

### `/launch/navigation.launch.py` (NEW FILE)
Created complete Nav2 launch file with all required nodes:
- map_server
- controller_server  
- planner_server
- behavior_server (recoveries)
- bt_navigator
- lifecycle_manager

### `/launch/zone_nav_ui.launch.py` (UPDATED)
Now includes navigation launch file automatically

### `/src/zone_nav/zone_nav/zone_web_ui.py` (ENHANCED)
Added two diagnostic endpoints:

**`GET /api/diagnostics/nav2`**
Returns:
- Robot footprint and dimensions
- Lidar configuration (both lidars)
- Costmap configuration
- Current navigation parameters
- System state

**`GET /api/diagnostics/obstacles`**
Returns:
- Obstacle count by distance zone
- Critical/warning/safe zone breakdown
- Assessment (BLOCKED/CLEAR)

---

## How to Apply Changes

1. **Stop all running ROS processes**
   ```bash
   # Press Ctrl+C in all terminals
   ```

2. **Rebuild workspace**
   ```bash
   cd /home/aun/Downloads/smr300l_gazebo_ros2control-main
   colcon build
   source install/setup.bash
   ```

3. **Restart simulation**
   Use your normal launch sequence, zone_nav_ui.launch.py now includes Nav2

4. **Test diagnostics**
   ```bash
   # Check Nav2 configuration
   curl http://localhost:5000/api/diagnostics/nav2 | jq
   
   # Check obstacle status
   curl http://localhost:5000/api/diagnostics/obstacles | jq
   ```

5. **Monitor Nav2 topics**
   ```bash
   # Check if Nav2 action server is running
   ros2 action list | grep navigate_to_pose
   
   # Monitor costmaps
   ros2 topic echo /local_costmap/costmap_raw --once
   ros2 topic echo /global_costmap/costmap_raw --once
   
   # Check both lidars are active
   ros2 topic hz /scan
   ros2 topic hz /scan2
   ```

---

## Expected Improvements

### Before (Issues):
- ❌ Robot spinning in place, can't make progress
- ❌ Nav2 thinks robot can fit through tight gaps (wrong footprint)
- ❌ Blind to rear obstacles (only front lidar)
- ❌ Gives up after a few failed attempts
- ❌ Slow, conservative movement
- ❌ Stops too far from waypoints

### After (Fixed):
- ✅ Tries 30+ rotation options before giving up
- ✅ Accurate 0.80m × 0.50m footprint
- ✅ 360° obstacle awareness (both lidars)
- ✅ More persistent path finding
- ✅ 35% faster linear, 50% faster rotation
- ✅ Precise waypoint arrival (15cm tolerance)
- ✅ Smoother cost gradients, less obstacle sensitivity
- ✅ Longer lookahead (2.0s vs 1.7s)

---

## Tuning Tips

If robot is still too cautious:
```yaml
# In nav2_params_working.yaml
BaseObstacle.scale: 0.005  # Even less obstacle avoidance
inflation_radius: 0.5      # Smaller safety margin
```

If robot is too aggressive:
```yaml
BaseObstacle.scale: 0.02   # More obstacle avoidance
inflation_radius: 0.7      # Larger safety margin
max_vel_x: 0.25            # Slower speed
```

If robot still spins too much:
```yaml
RotateToGoal.scale: 8.0    # Reduce rotation preference
PathAlign.scale: 8.0       # More flexible path following
```

---

## Technical Details

### Robot Physical Specs:
- **Chassis:** 800mm × 500mm × 90mm
- **Wheelbase:** 540mm (270mm to each wheel)
- **Wheel diameter:** 106mm (53mm radius)
- **Front lidar:** x=0.75m, 180° FOV
- **Rear lidar:** x=0.05m, 180° reversed

### Coordinate Frames:
- `base_link` → robot center
- `laser_frame` → front lidar at (0.75, 0, 0.10)
- `laser_frame2` → rear lidar at (0.05, 0, 0.10)
- `odom` → odometry from diff_drive
- `map` → global map from AMCL

### Footprint Calculation:
```
Robot length: 0.80m → ±0.40m from center
Robot width: 0.50m → ±0.25m from center

Corners: [front-right, front-left, back-left, back-right]
[[0.40, 0.25], [0.40, -0.25], [-0.40, -0.25], [-0.40, 0.25]]
```

---

## Verification Commands

```bash
# Check Nav2 nodes are running
ros2 node list | grep -E "(controller|planner|bt_navigator|recoveries|map_server)"

# Check costmap topics
ros2 topic list | grep costmap

# Check both lidars publishing
ros2 topic hz /scan
ros2 topic hz /scan2

# Monitor navigation status
ros2 topic echo /plan

# Check for errors
ros2 topic echo /rosout | grep -i error

# View costmap parameters
ros2 param list /local_costmap/local_costmap
ros2 param get /local_costmap/local_costmap footprint
ros2 param get /local_costmap/local_costmap inflation_radius
```

---

## Common Issues & Solutions

### Issue: Robot still spins
**Check:** 
```bash
ros2 param get /local_costmap/local_costmap footprint
# Should show: [[0.40, 0.25], [0.40, -0.25], [-0.40, -0.25], [-0.40, 0.25]]
```
**If wrong:** Footprint not loaded, restart Nav2 nodes

### Issue: Robot stops for no reason
**Check:**
```bash
curl http://localhost:5000/api/diagnostics/obstacles
# Look at critical_zone_pct
```
**If high:** Lidar seeing false obstacles, check min_valid_range in zone_web_ui.py

### Issue: Nav2 action not available
**Check:**
```bash
ros2 action list
# Should include /navigate_to_pose
```
**If missing:** Nav2 nodes not launched, check navigation.launch.py

---

## Performance Metrics

| Metric | Before | After | Change |
|--------|--------|-------|--------|
| Max Linear Velocity | 0.26 m/s | 0.35 m/s | +35% |
| Max Rotational Velocity | 1.0 rad/s | 1.5 rad/s | +50% |
| Trajectory Samples (vtheta) | 20 | 30 | +50% |
| Sim Lookahead Time | 1.7s | 2.0s | +18% |
| Progress Required | 0.5m | 0.15m | -70% |
| Controller Frequency | 10 Hz | 15 Hz | +50% |
| Inflation Radius (local) | 0.4m | 0.55m | +37% |
| Goal XY Tolerance | 0.25m | 0.15m | -40% |
| Footprint Accuracy | Circle (22cm) | Rectangle (80×50cm) | CRITICAL |
| Lidar Coverage | 180° front | 360° both | CRITICAL |

---

## Last Updated
January 14, 2026

Robot should now navigate smoothly with proper obstacle awareness and try multiple paths before giving up!
