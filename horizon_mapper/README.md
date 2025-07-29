# Horizon Mapper

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

A ROS2 package for trajectory processing and publishing designed for F1TENTH autonomous racing applications. The Horizon Mapper processes optimal trajectories and provides predictive trajectory horizons for Model Predictive Control (MPC) systems.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Topics](#topics)
- [Parameters](#parameters)
- [Dependencies](#dependencies)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Overview

The Horizon Mapper is a crucial component in the F1TENTH autonomous racing stack that bridges the gap between global trajectory planning and local model predictive control. It processes optimal racing trajectories and publishes predictive trajectory horizons that enable real-time vehicle control.

### Key Capabilities

- **Trajectory Processing**: Loads and preprocesses optimal racing trajectories from CSV files
- **Predictive Horizons**: Generates rolling trajectory horizons for MPC controllers
- **State Estimation**: Integrates vehicle odometry and pose estimates for accurate trajectory tracking
- **Real-time Publishing**: Provides continuous trajectory updates at configurable frequencies
- **Validation**: Ensures trajectory data integrity with comprehensive validation checks

## Features

- ✅ **ROS2 Native**: Built for ROS2 Humble with modern Python practices
- ✅ **Configurable Horizons**: Adjustable prediction horizon length and time steps
- ✅ **Multi-source Localization**: Supports both odometry and RViz pose estimates
- ✅ **Trajectory Validation**: Comprehensive data validation and error handling
- ✅ **Real-time Performance**: Optimized for 10Hz+ control loops
- ✅ **Visualization Support**: Publishes Path messages for RViz visualization
- ✅ **Parameter Server Integration**: Full ROS2 parameter system support

## Installation

### Prerequisites

- ROS2 Humble
- Python 3.8+
- NumPy
- tf_transformations

### Build Instructions

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your-username/horizon_mapper.git
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select horizon_mapper
   source install/setup.bash
   ```

## Usage

### Basic Launch

Launch the Horizon Mapper with default parameters:

```bash
ros2 run horizon_mapper horizon_mapper_node
```

### Launch with Custom Parameters

```bash
ros2 run horizon_mapper horizon_mapper_node --ros-args \
  -p horizon_N:=10 \
  -p horizon_T:=0.5 \
  -p enable_logging:=true
```

### Using Launch Files

```bash
ros2 launch horizon_mapper horizon_mapper.launch.py
```

### Integration Example

The Horizon Mapper is typically used as part of a larger autonomous racing stack:

```bash
# Launch the full stack (example)
ros2 launch race_stack autonomous_racing.launch.py \
  map_name:=your_track \
  enable_horizon_mapper:=true
```

## Configuration

### Trajectory Files

The package expects trajectory files in CSV format with the following columns:

```csv
x,y,v,theta
0.0,0.0,1.0,0.0
1.0,0.1,1.2,0.05
...
```

- `x`, `y`: Position coordinates (meters)
- `v`: Velocity (m/s)
- `theta`: Heading angle (radians)

### Configuration File

Edit `config/horizon_mapper.yaml` to customize behavior:

```yaml
horizon_mapper:
  ros__parameters:
    horizon_N: 8              # Prediction horizon steps
    horizon_T: 0.5            # Time per horizon step (seconds)
    wheelbase: 0.33           # Vehicle wheelbase (meters)
    max_steering_angle: 0.5   # Maximum steering angle (radians)
    min_speed: 0.1            # Minimum vehicle speed (m/s)
    enable_logging: false     # Enable detailed logging
```

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/car_state/odom` | `nav_msgs/Odometry` | Vehicle odometry data |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 2D pose estimates |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/horizon_mapper/reference_trajectory` | `giu_f1t_interfaces/VehicleStateArray` | Predictive trajectory horizon for MPC |
| `/horizon_mapper/reference_path` | `nav_msgs/Path` | Full reference path for visualization |
| `/horizon_mapper/path_ready` | `std_msgs/Bool` | Status indicator for trajectory readiness |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_logging` | bool | `false` | Enable detailed logging output |
| `optimal_trajectory_path` | string | Auto-detected | Path to input trajectory CSV file |
| `reference_trajectory_path` | string | Auto-detected | Path to processed trajectory CSV file |
| `horizon_N` | int | `8` | Number of prediction horizon steps |
| `horizon_T` | double | `0.5` | Time step for prediction horizon |
| `wheelbase` | double | `0.33` | Vehicle wheelbase length |
| `max_steering_angle` | double | `0.5` | Maximum steering angle limit |
| `min_speed` | double | `0.1` | Minimum vehicle speed |
| `odom_topic` | string | `"car_state/odom"` | Odometry topic name |

## Dependencies

### ROS2 Packages
- `rclpy` - ROS2 Python client library
- `std_msgs` - Standard ROS2 message types
- `geometry_msgs` - Geometry-related message types
- `nav_msgs` - Navigation message types
- `tf_transformations` - Coordinate transformation utilities
- `giu_f1t_interfaces` - Custom F1TENTH interface definitions

### Python Packages
- `numpy` - Numerical computing
- `csv` - CSV file processing
- `math` - Mathematical functions

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Odometry      │───▶│  Horizon        │───▶│   MPC           │
│   /car_state/   │    │  Mapper         │    │   Controller    │
│   odom          │    │                 │    │                 │
└─────────────────┘    │  ┌───────────┐  │    └─────────────────┘
                       │  │Trajectory │  │
┌─────────────────┐    │  │Processing │  │    ┌─────────────────┐
│   RViz Pose     │───▶│  └───────────┘  │───▶│   Visualization │
│   /initialpose  │    │                 │    │   /rviz         │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Contributing

We welcome contributions! Please follow these steps:

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/amazing-feature`
3. **Make your changes** and ensure tests pass
4. **Commit your changes**: `git commit -m 'Add amazing feature'`
5. **Push to the branch**: `git push origin feature/amazing-feature`
6. **Open a Pull Request**

### Development Guidelines

- Follow PEP 8 Python style guidelines
- Add docstrings to all functions and classes
- Include unit tests for new functionality
- Update documentation as needed

### Testing

Run the test suite:

```bash
cd ~/ros2_ws
colcon test --packages-select horizon_mapper
colcon test-result --verbose
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **F1TENTH Community**: For the autonomous racing platform and ecosystem
- **ROS2 Team**: For the excellent robotics middleware
- **ETH Zurich PBL**: For the ForzaETH race stack inspiration
- **Contributors**: Thanks to all who have contributed to this project

## Citation

If you use this package in your research, please cite:

```bibtex
@software{horizon_mapper_2025,
  title={Horizon Mapper: Trajectory Processing for F1TENTH Autonomous Racing},
  author={Mohammed Azab},
  year={2025},
  url={https://github.com/your-username/horizon_mapper}
}
```

## Support

- **Issues**: Report bugs and request features via [GitHub Issues](https://github.com/your-username/horizon_mapper/issues)
- **Discussions**: Join the conversation in [GitHub Discussions](https://github.com/your-username/horizon_mapper/discussions)
- **Email**: For direct support, contact mo7ammed3zab@outlook.com

---

**Happy Racing! 🏁**
