# AeroController — Custom Instructions for AI Code Assistants (Copilot / Claude Sonnet)

## 1) Project context
- **Goal:** Reliable MPC/LQR control for an aerial platform in ROS 2 with clean integration to:
  - **HorizonMapper** (trajectory/path or setpoint provider)
  - **DualEKF** (state estimation: odometry/**)
- **Primary node:** `AeroController` (Python, rclpy)
- **Constraint:** Edit existing AeroController code in-place. **Do not** create duplicate files with new names for the same logic. Create a new file **only if** it’s missing and truly required.

## 2) What to analyze first
- Scan these sources (adjust to actual repo layout):
  - `aero_controller/**.py`, `aero_controller/__init__.py`
  - `launch/**.launch.py`, `config/**.yaml`
  - `horizon_mapper/**`, `dual_ekf/**`
- Build a **ROS Graph Contract** directly from code + launch:
  - For each node: publishers, subscribers, **exact topic names**, message types, QoS, frame_ids.
  - Confirm with running graph (`ros2 topic list`, `ros2 node info`, `ros2 interface show`).

> **Do not invent** topic names or message types. Align AeroController to what HorizonMapper and DualEKF already use.

## 3) Expected interfaces (fill from code)
Create/maintain this table in PR/commit output:


ROS Graph Contract
Based on my analysis of the codebase, here's the current ROS graph contract:

Signal	Topic (exact)	Msg type	QoS	From → To	Notes
Trajectory/Setpoint	/horizon_mapper/reference_trajectory	giu_f1t_interfaces/VehicleStateArray	reliable	HorizonMapper → AeroController	Required for MPC/LQR input
State/Odometry	/odom	nav_msgs/Odometry	best_effort	DualEKF → AeroController	Current: simulator/EKF
EKF Estimate	/dual_ekf/estimate	nav_msgs/Odometry	reliable	DualEKF → AeroController	Improved state estimation
Control Command	/drive	ackermann_msgs/AckermannDriveStamped	reliable	AeroController → Actuation	Vehicle control commands
Horizon Control	/aero_controller/horizon_control	giu_f1t_interfaces/HorizonControl	reliable	AeroController → HorizonMapper	Adaptive horizon feedback
Horizon Status	/aero_controller/horizon_status	giu_f1t_interfaces/HorizonStatus	reliable	HorizonMapper → AeroController	Horizon status updates
LIDAR Scan	/scan	sensor_msgs/LaserScan	best_effort	Simulator → AeroController	For corridor building
Controller Status	/controller_status	std_msgs/String	reliable	AeroController → System	Controller diagnostics

## 4) Implementation rules
- **Edit in place:** Update existing `aero_controller` files rather than cloning them under new names.
- **New files:** Allowed only if they don’t exist and are necessary. If added:
  - Export in `__init__.py` as needed.
  - Update imports and launch files.
- **rclpy best practices:**
  - Use timers/callback groups, no blocking sleeps.
  - Validate parameters at startup; log config.
  - Structured logging of key signals: setpoint, state, control output, solver status, cycle time.

## 5) Validation flow (MPC first, then LQR)
1. **Wire-up check**
   - Launch system; confirm topics, types, QoS, frame_ids match the contract.
   - Add QoS profiles compatible with EKF/mapper (e.g., `sensor_data` where appropriate).
2. **MPC validation**
   - Run with real or synthetic inputs (e.g., small step/ramp or short path).
   - Verify stable control outputs, correct timestamps, and no missed deadlines.
   - Log solver convergence/iterations; measure loop latency.
3. **LQR validation**
   - Re-run the same harness with `controller_mode="lqr"`.
   - Confirm parity of inputs/outputs and stability criteria.
4. **Common checks**
   - Consistent frames (`map`, `odom`, `base_link`) and correct `header.stamp`.
   - No exceptions on parameter changes or lifecycle events.
   - Clean shutdown on Ctrl-C.

## 7) Test & tooling
Provide minimal, reproducible test assets:
- A small **launch** or **script** to run AeroController with HorizonMapper and DualEKF.
- Optionally a tiny publisher script for synthetic inputs if upstream nodes are unavailable.
- Basic assertions (even print/log checks) to confirm message flow.
