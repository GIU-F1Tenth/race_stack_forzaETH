# Berlin F1TENTH Track Map

This directory contains the Berlin track map files for F1TENTH autonomous racing, now updated for full compatibility with the race stack.

## 📁 File Structure

### Map Files
- **`Berlin.yaml`** - Main map configuration (YAML format)
- **`Berlin_map.yaml`** - Stack master map configuration  
- **`Berlin.png`** - Track map image (converted from PGM)
- **`Berlin_map.png`** - Stack master map image
- **`berlin.yaml`** - Legacy map configuration (backward compatibility)
- **`berlin.pgm`** - Original PGM map image

### Trajectory Files
- **`Berlin_centerline.csv`** - Track centerline with track width info
- **`Berlin_raceline.csv`** - Optimized racing line
- **`Berlin_DonkeySim_waypoints.txt`** - DonkeySim compatible waypoints
- **`global_waypoints.json`** - Complete waypoint data with metadata
- **`traj_race_cl-2025-08-03.csv`** - Original trajectory file (x, y, velocity)

### Configuration Files
- **`speed_scaling.yaml`** - Sector-based speed limits
- **`ot_sectors.yaml`** - Overtaking zones configuration

## 🏁 Track Information

- **Track Name**: Berlin
- **Resolution**: 0.05m per pixel
- **Origin**: [-11.606540, -26.520793, 0.0]
- **Total Waypoints**: 328
- **Track Width**: 2.2m (1.1m each side)
- **Sectors**: 2 (for speed tuning)

## 🔧 Compatibility Updates

### Changes Made:
1. **✅ Image Format**: Converted from PGM to PNG for better compatibility
2. **✅ YAML Format**: Updated to modern YAML structure with proper formatting
3. **✅ Centerline Format**: Converted trajectory to standard centerline format with track widths
4. **✅ Waypoint Files**: Created all standard waypoint file formats
5. **✅ Configuration Files**: Added speed scaling and overtaking zones
6. **✅ JSON Metadata**: Created structured waypoint data with metadata

### File Naming Convention:
- `Berlin.yaml` - Main configuration (matches directory name)
- `Berlin_*.csv` - Trajectory files (matches F1TENTH convention)
- `Berlin_map.*` - Map files for stack master

## 🎯 Usage

### For Simulation:
```bash
# Load map in F1TENTH gym
map_path = "Berlin.yaml"

# Or use in stack master
map_name = "Berlin"
```

### For Navigation Stack:
```bash
# Use centerline for path planning
centerline_file = "Berlin_centerline.csv"

# Use raceline for racing
raceline_file = "Berlin_raceline.csv"
```

### For Speed Control:
```bash
# Load speed scaling configuration  
speed_config = "speed_scaling.yaml"
```

## 📊 Track Characteristics

- **Sector 0** (Waypoints 0-164): Speed limit 0.6 (60% of max)
- **Sector 1** (Waypoints 164-328): Speed limit 0.8 (80% of max)
- **Global Limit**: 0.8 (80% of maximum vehicle speed)

## 🔄 Legacy Support

The original files are preserved for backward compatibility:
- `berlin.yaml` - Original configuration
- `berlin.pgm` - Original PGM image
- `traj_race_cl-2025-08-03.csv` - Original trajectory data

## 🚀 Integration

This map is now fully compatible with:
- ✅ F1TENTH Gym simulator
- ✅ Stack Master navigation
- ✅ LQR/LQG controllers
- ✅ Path planning algorithms
- ✅ Speed scaling systems
- ✅ Overtaking detection
- ✅ Visualization tools (RViz, etc.)

---

**Last Updated**: August 3, 2025  
**Compatibility Version**: v2.0  
**Original Source**: F1TENTH Berlin track
