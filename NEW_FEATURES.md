# Zone Navigation UI - New Features

## 🎨 Modern UI Redesign
- **Card-based Layout**: All controls organized in clean, modern cards
- **Sticky Header**: Green gradient header stays visible while scrolling
- **Better Organization**: Grouped tools and controls for easier access
- **Professional Styling**: Shadows, gradients, and smooth transitions throughout

## 🗺️ Enhanced Map Tools

### Hand/Pan Tool (H)
- **Drag to Pan**: Click and drag to move around the map
- **Keyboard Shortcut**: Press `H` to activate
- **Space Key**: Hold `Space` to temporarily activate pan mode
- **Cursor Feedback**: Changes to hand cursor when active

### Select Tool (V)
- **Default Mode**: Click on map to set zone coordinates
- **Keyboard Shortcut**: Press `V` to activate
- **Crosshair Cursor**: Precise coordinate selection

### Edit/Move Tool (M)
- **Drag Zones**: Click and drag existing zones to reposition them
- **Keyboard Shortcut**: Press `M` to activate
- **Auto-Save**: Zones are automatically updated when moved

## 🚨 Emergency Stop (E-STOP)

### Activation
- **Red Button**: Large red E-STOP button at top of sidebar
- **Keyboard Shortcut**: Press `E` to activate emergency stop
- **Immediate Action**: Instantly halts all robot movement

### E-STOP Behavior
- ✓ Cancels all active navigation goals
- ✓ Stops path following sequences
- ✓ Disables manual joystick control
- ✓ Sends zero velocity commands
- ✓ Shows prominent notification popup

### Reset
- **Reset & Resume Button**: In notification popup
- **Restoration**: All systems can resume after reset

## 🚫 NO-GO Zones

### Creating NO-GO Zones
1. Click the **⛔ NO-GO** button in zone type selector
2. Click on map to place the zone
3. Enter zone name and save

### Visual Representation
- **Red Marker**: NO-GO zones appear as red circles
- **Exclusion Radius**: 1-meter radius shown with red shaded area
- **Icon**: ⛔ icon displayed in zone label
- **No Arrow**: NO-GO zones don't show orientation arrows

### Behavior
- Robot navigation should avoid these areas
- Acts as virtual obstacles in path planning
- Clear visual warning to operators

## 🏎️ Speed Zones

### Creating Speed Zones
1. Click the **🏎️ Speed** button in zone type selector
2. Enter speed limit (0.1 - 1.0 m/s) in the revealed input field
3. Click on map to place the zone
4. Enter zone name and save

### Visual Representation
- **Amber/Yellow Marker**: Speed zones appear in yellow
- **Icon**: 🏎️ icon displayed in zone label
- **Speed Limit**: Stored with zone metadata

### Behavior
- Override default navigation speed when robot enters this zone
- Allows for slow zones in sensitive areas
- Faster travel in open areas

## 🛡️ Safety Validation

### Automatic Checks
When creating a new zone, the system validates:

1. **Obstacle Distance**
   - Minimum 0.5m from any obstacle
   - Uses live laser scan data
   - Prevents placing zones inside walls

2. **Map Bounds**
   - Maximum 50m from origin
   - Ensures zones are within reachable area
   - Prevents unrealistic placements

3. **Safety Warnings**
   - Clear error messages if validation fails
   - Popup notifications for safety violations
   - Zone creation blocked until issues resolved

## ⌨️ Keyboard Shortcuts

### Map Tools
- `H` - Activate Pan/Hand tool
- `V` - Activate Select tool  
- `M` - Activate Move/Edit tool
- `Space` - Temporary pan mode (hold)

### View Controls
- `+` or `=` - Zoom in
- `-` or `_` - Zoom out
- `R` - Reset view
- `F` - Fit map to screen

### Emergency
- `E` - Emergency stop (E-STOP)

### Manual Control (when expanded)
- `W` / `↑` - Move forward
- `S` / `↓` - Move backward
- `A` / `←` - Rotate left
- `D` / `→` - Rotate right

## 🎛️ Collapsible Sections

All major sections can be collapsed/expanded:
- **Saved Zones**: Hide zone list when not needed
- **Manual Control**: Expand only when actively controlling
- **Sequence Controls**: Collapse when not in use
- **Path Drawing**: Hide drawing controls when navigating

## 🔄 Workflow Improvements

### Zone Creation Workflow
1. Select zone type (Normal, NO-GO, or Speed)
2. Set speed limit (if speed zone)
3. Click map or enter coordinates
4. System validates safety
5. Enter zone name
6. Save - automatic validation occurs

### Navigation Workflow
1. Use Hand tool (H) to explore map
2. Use Select tool (V) to place zones
3. Switch to Edit tool (M) to reposition zones
4. Test with manual control before autonomous navigation
5. Keep E-STOP (E) accessible at all times

## 🎨 Visual Indicators

### Zone Colors
- 🟢 **Green**: Normal waypoint zones
- 🔴 **Red**: NO-GO zones (prohibited areas)
- 🟡 **Yellow**: Speed override zones

### Tool Buttons
- **Active Tool**: Highlighted in green with glow effect
- **Hover State**: Button lifts and border glows
- **Icons**: Clear visual representation of each tool

## 🚀 Launch Instructions

```bash
# Terminal 1: Start ROS 2 navigation
ros2 launch my_bot zone_nav_ui.launch.py

# Terminal 2: Open web browser
firefox http://192.168.20.28:5000
```

## 💡 Best Practices

1. **Always Test E-STOP**: Press `E` to verify emergency stop before autonomous operation
2. **Use NO-GO Zones**: Mark doorways, stairs, or sensitive equipment areas
3. **Speed Zones**: Create slow zones near people or valuable equipment
4. **Save Often**: Zones are persistent, but create backups of zones.yaml
5. **Keyboard Shortcuts**: Use `H`, `V`, `M` for faster workflow
6. **Pan with Space**: Hold space while using other tools for quick repositioning

## 🔧 Technical Details

### API Endpoints
- `POST /api/estop/activate` - Trigger emergency stop
- `POST /api/estop/reset` - Reset emergency stop
- `GET /api/estop/status` - Check E-STOP state
- `POST /api/zones/save` - Save zone with type and speed limit
- `POST /api/zones/update` - Update zone position (drag & drop)

### Safety Parameters
- **Safety Radius**: 0.5 meters minimum from obstacles
- **Max Range**: 50 meters from origin
- **NO-GO Exclusion**: 1 meter radius around marker

### Zone Metadata
Zones now support additional fields:
```yaml
zones:
  zone_name:
    position: {x: 1.0, y: 2.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    metadata:
      type: "no-go"  # or "speed" or "normal"
      speed_limit: 0.3  # m/s (only for speed zones)
```
