#!/bin/bash

# Script to fix all maps with Austin-style configuration
cd /home/mohammedazab/ws/src/race_stack/stack_master/maps

for dir in */; do
    map_name=${dir%/}
    
    # Skip maps that are already properly configured
    if [[ "$map_name" == "Austin" || "$map_name" == "glc_ot_ez" || "$map_name" == "hangar_1905_v0" || "$map_name" == "GLC_smile_small" ]]; then
        echo "Skipping $map_name (already configured)"
        continue
    fi
    
    echo "=== Processing $map_name ==="
    cd "$map_name"
    
    # 1. Create main YAML file if it doesn't exist
    if [[ ! -f "${map_name}.yaml" ]]; then
        if [[ -f "${map_name}_map.yaml" ]]; then
            echo "Creating ${map_name}.yaml from ${map_name}_map.yaml"
            cp "${map_name}_map.yaml" "${map_name}.yaml"
            # Update image reference to use map_name.png
            sed -i "s/image: .*_map.png/image: ${map_name}.png/" "${map_name}.yaml"
        fi
    fi
    
    # 2. Create main PNG file if it doesn't exist
    if [[ ! -f "${map_name}.png" ]]; then
        if [[ -f "${map_name}_map.png" ]]; then
            echo "Creating symlink ${map_name}.png -> ${map_name}_map.png"
            ln -sf "${map_name}_map.png" "${map_name}.png"
        fi
    fi
    
    # 3. Create global_waypoints.json if it doesn't exist
    if [[ ! -f "global_waypoints.json" ]]; then
        echo "Creating global_waypoints.json for $map_name"
        cat > global_waypoints.json << EOF
{
  "map_info_str": {
    "data": "${map_name} track - Basic configuration for simulator"
  },
  "est_lap_time": {
    "data": 60.0
  },
  "centerline_markers": {
    "markers": []
  },
  "global_waypoints": {
    "markers": []
  },
  "raceline_markers": {
    "markers": []
  },
  "centerline_waypoints": {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "map"
    },
    "wpnts": [
      {
        "id": 0,
        "s_m": 0,
        "d_m": 0,
        "x_m": 0.0,
        "y_m": 0.0,
        "d_right": 1.1,
        "d_left": 1.1,
        "psi_rad": 0.0,
        "kappa_radpm": 0.0,
        "vx_mps": 0,
        "ax_mps2": 0
      }
    ]
  },
  "global_traj_markers_iqp": {
    "markers": []
  },
  "global_traj_markers_sp": {
    "markers": []
  },
  "global_traj_wpnts_iqp": {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "map"
    },
    "wpnts": []
  },
  "global_traj_wpnts_sp": {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "map"
    },
    "wpnts": []
  },
  "trackbounds_markers": {
    "markers": []
  }
}
EOF
    fi
    
    # 4. Create ot_sectors.yaml if it doesn't exist
    if [[ ! -f "ot_sectors.yaml" ]]; then
        echo "Creating ot_sectors.yaml for $map_name"
        cat > ot_sectors.yaml << EOF
ot_interpolator:
  ros__parameters:
    map_name: "${map_name}"
    n_sectors: 2
    yeet_factor: 1.25
    spline_len: 30
    ot_sector_begin: 0.5
    
    Overtaking_sector0:
      start: 0
      end: 100
      ot_flag: false
    
    Overtaking_sector1:
      start: 101
      end: 200
      ot_flag: false
EOF
    fi
    
    # 5. Create speed_scaling.yaml if it doesn't exist
    if [[ ! -f "speed_scaling.yaml" ]]; then
        echo "Creating speed_scaling.yaml for $map_name"
        cat > speed_scaling.yaml << EOF
sector_tuner:
  ros__parameters:
    map_name: "${map_name}"
    global_limit: 0.8
    n_sectors: 2
    
    Sector0:
      start: 0
      end: 100
      scaling: 0.8
      only_FTG: false
      no_FTG: false
    
    Sector1:
      start: 101
      end: 200
      scaling: 0.8
      only_FTG: false
      no_FTG: false
EOF
    fi
    
    cd ..
    echo "Completed $map_name"
done

echo "=== All maps processed ==="
