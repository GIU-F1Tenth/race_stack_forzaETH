from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from giu_f1t_interfaces.msg import VehicleState, VehicleStateArray

from .preprocess_trajectory import preprocess_trajectory

import rclpy
from rclpy.node import Node
import csv
import math
import numpy as np
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory



# Import configuration defaults
import sys, os
try:
    # Try multiple paths to find config
    possible_config_paths = [
        os.path.join(os.path.dirname(__file__), '..', 'config'),  # Source tree
        '/home/mohammedazab/ws/src/race_stack/horizon_mapper/config',  # Absolute path
        os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config')  # Alternative relative
    ]
    
    config_imported = False
    for config_path in possible_config_paths:
        if os.path.exists(config_path) and config_path not in sys.path:
            sys.path.insert(0, config_path)
            try:
                import config as default_config
                print(f"Using config.py defaults from {config_path}")
                config_imported = True
                break
            except ImportError:
                continue
    
    if not config_imported:
        raise ImportError("Config module not found in any expected location")
        
except ImportError as e:
    # Fallback if config.py is not available
    class DefaultConfig:
        def __init__(self):
            # Get package share directory for data files
            try:
                pkg_share = get_package_share_directory('horizon_mapper')
                self.optimal_trajectory_path = os.path.join(pkg_share, 'data', 'optimal_trajectory.csv')
                self.reference_trajectory_path = os.path.join(pkg_share, 'data', 'ref_trajectory.csv')
            except:
                # Fallback to relative paths
                pkg_dir = os.path.dirname(os.path.abspath(__file__))
                self.optimal_trajectory_path = os.path.join(pkg_dir, 'optimal_trajectory.csv')
                self.reference_trajectory_path = os.path.join(pkg_dir, 'ref_trajectory.csv')

            self.enable_trajectory_generation = True
            self.horizon_N = 8
            self.wheelbase = 0.33
            self.max_steering_angle = 0.5
            self.min_speed = 0.1
            self.odom_topic = "car_state/odom"
    
    default_config = DefaultConfig()





class HorizonMapperNode(Node):
    """
    Horizon Mapper Node - Processes and publishes trajectory data for MPC controller
    
    This node:
    1. Loads and preprocesses trajectory data
    2. Publishes reference trajectories for MPC horizon
    3. Handles odometry updates for current vehicle state
    4. Provides trajectory validation and error handling
    """

    def __init__(self):
        super().__init__('horizon_mapper_node')

        # Declare ROS2 parameters
        self.declare_parameter('enable_logging', False)
        self.declare_parameter('optimal_trajectory_path', default_config.optimal_trajectory_path)
        self.declare_parameter('reference_trajectory_path', default_config.reference_trajectory_path)
        self.declare_parameter('horizon_N', default_config.horizon_N)
        self.declare_parameter('horizon_T', 0.5)
        self.declare_parameter('wheelbase', default_config.wheelbase)
        self.declare_parameter('max_steering_angle', default_config.max_steering_angle)
        self.declare_parameter('min_speed', default_config.min_speed)
        self.declare_parameter('odom_topic', default_config.odom_topic)

        # Load parameters from ROS2 parameter server
        self.enable_logging = self.get_parameter('enable_logging').value
        self.input_path = self.get_parameter('optimal_trajectory_path').value
        self.output_path = self.get_parameter('reference_trajectory_path').value
        self.horizon = self.get_parameter('horizon_N').value
        self.horizon_T = self.get_parameter('horizon_T').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.min_speed = self.get_parameter('min_speed').value
        self.odom_topic = self.get_parameter('odom_topic').value

        if self.enable_logging:
            self.get_logger().info("Detailed logging enabled")
        else:
            self.get_logger().info("Detailed logging disabled - only warnings and errors will be shown")

        if not self.input_path or not self.output_path:
            self.get_logger().error("Missing trajectory paths in parameters!")
            return

        self.get_logger().info(f"Input trajectory: {self.input_path}")
        self.get_logger().info(f"Output trajectory: {self.output_path}")
        self.get_logger().info(f"Horizon: {self.horizon} steps")

        # State variables
        self.reference_trajectory = []
        self.path_ready = False
        self.current_vehicle_state = VehicleState()
        # Initialize vehicle state with safe default values
        self.current_vehicle_state.x = 0.0
        self.current_vehicle_state.y = 0.0
        self.current_vehicle_state.v = 0.0
        self.current_vehicle_state.theta = 0.0  # Initialize heading
        self.current_vehicle_state.delta = 0.0
        self.current_pose_estimate = None
        self.trajectory_index = 0
        
        # Debug: Log initial vehicle state
        self.get_logger().info(f"Initialized vehicle state: "
                              f"x={self.current_vehicle_state.x}, y={self.current_vehicle_state.y}, "
                              f"v={self.current_vehicle_state.v}, theta={self.current_vehicle_state.theta}")

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odometry_callback,
            10)
        
        self.get_logger().info(f"Subscribing to odometry topic: {self.odom_topic}")

        # Subscribe to RViz 2D Pose Estimate
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_estimate_callback,
            10)

        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            '/horizon_mapper/reference_path',
            10)

        self.trajectory_pub = self.create_publisher(
            VehicleStateArray,
            '/horizon_mapper/reference_trajectory',
            10)

        self.status_pub = self.create_publisher(
            Bool,
            '/horizon_mapper/path_ready',
            10)

        # Timers
        self.publish_timer = self.create_timer(0.1, self.publish_predictive_trajectory)  # 10 Hz for MPC

        # Run preprocessing on startup
        self.preprocess_and_load_trajectory()

    def log_info(self, message):
        """Log info messages only if logging is enabled"""
        if self.enable_logging:
            self.get_logger().info(message)

    def log_debug(self, message):
        """Log debug messages only if logging is enabled"""
        if self.enable_logging:
            self.get_logger().debug(message)

    def odometry_callback(self, msg):
        """Update current vehicle state from odometry"""
        self.log_info("Odometry callback triggered")
        try:
            # Extract position
            self.current_vehicle_state.x = msg.pose.pose.position.x
            self.current_vehicle_state.y = msg.pose.pose.position.y
            
            # Extract velocity (assuming this is in body frame)
            linear_velocity = msg.twist.twist.linear
            self.current_vehicle_state.v = np.sqrt(linear_velocity.x**2 + linear_velocity.y**2)

            # Extract yaw from quaternion
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])
            
            # Set the theta (heading angle) - this is critical for MPC!
            self.current_vehicle_state.theta = yaw

            # For now, set delta to 0 (we'll calculate this from trajectory tracking)
            self.current_vehicle_state.delta = 0.0

            self.log_debug(f"Odometry update: x={self.current_vehicle_state.x:.2f}, "
                           f"y={self.current_vehicle_state.y:.2f}, "
                           f"v={self.current_vehicle_state.v:.2f}, "
                           f"theta={self.current_vehicle_state.theta:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {str(e)}")

    def pose_estimate_callback(self, msg):
        """Update pose estimate from RViz 2D Pose Estimate tool"""
        try:
            self.current_pose_estimate = msg

            # Use pose estimate to correct vehicle state
            self.current_vehicle_state.x = msg.pose.pose.position.x
            self.current_vehicle_state.y = msg.pose.pose.position.y

            # Extract yaw from quaternion for heading
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])
            
            # Set the theta (heading angle) - this is critical for MPC!
            self.current_vehicle_state.theta = yaw

            self.log_info(f"Pose estimate from RViz: x={self.current_vehicle_state.x:.2f}, "
                          f"y={self.current_vehicle_state.y:.2f}, "
                          f"theta={self.current_vehicle_state.theta:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error processing pose estimate: {str(e)}")

    def preprocess_and_load_trajectory(self):
        """Run preprocessing script and load trajectory"""
        try:
            self.log_info("Starting trajectory preprocessing...")

            # Run the preprocessing script with parameters
            success = preprocess_trajectory(
                self.input_path,
                self.output_path,
                wheelbase=self.wheelbase,
                max_steering=self.max_steering,
                min_velocity=self.min_speed
            )

            if success:
                self.log_info("Preprocessing completed successfully")
                self.reference_trajectory = self.load_trajectory_from_csv(self.output_path)
                self.path_ready = True
                self.log_info(f"Reference trajectory loaded: {len(self.reference_trajectory)} points")
            else:
                self.get_logger().error("Preprocessing failed")
                self.path_ready = False

        except Exception as e:
            self.get_logger().error(f"Error during preprocessing: {str(e)}")
            self.path_ready = False

    def load_trajectory_from_csv(self, path):
        """Load processed trajectory from CSV file with theta support"""
        data = []
        try:
            with open(path, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    state = VehicleState()
                    state.x = float(row['x'])
                    state.y = float(row['y'])
                    state.v = float(row['v'])

                    # Handle theta (heading angle) - this is what MPC needs
                    if 'theta' in row:
                        state.theta = float(row['theta'])
                    else:
                        # Calculate theta from the data if not provided
                        state.theta = 0.0  # Will be calculated later

                    # MPC controller computes steering angles, so set to 0.0
                    state.delta = 0.0
                    data.append(state)

            # If theta was not provided, calculate it from consecutive points
            if len(data) > 1 and data[0].theta == 0.0:
                for i in range(len(data)):
                    if i < len(data) - 1:
                        dx = data[i + 1].x - data[i].x
                        dy = data[i + 1].y - data[i].y
                        data[i].theta = math.atan2(dy, dx)
                    else:
                        # For the last point, use the same theta as the previous point
                        data[i].theta = data[i - 1].theta

            # Validate all loaded trajectory points
            valid_points = []
            invalid_count = 0
            for i, state in enumerate(data):
                if self._validate_vehicle_state(state):
                    valid_points.append(state)
                else:
                    invalid_count += 1
                    self.get_logger().warn(f"Invalid trajectory point at index {i}: "
                                         f"x={state.x}, y={state.y}, v={state.v}, theta={state.theta}")
            
            if invalid_count > 0:
                self.get_logger().warn(f"Removed {invalid_count} invalid trajectory points")
            
            # Log sample of loaded trajectory for debugging
            if len(valid_points) > 0:
                self.log_info(f"Sample trajectory points loaded:")
                for i in range(min(3, len(valid_points))):
                    state = valid_points[i]
                    self.log_info(f"  Point {i}: x={state.x:.2f}, y={state.y:.2f}, v={state.v:.2f}, theta={state.theta:.2f}")
            
            self.log_info(f"Loaded {len(valid_points)} valid trajectory points (removed {invalid_count} invalid)")
            return valid_points
            
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory: {str(e)}")
        return data

    def find_closest_trajectory_point(self):
        """Find the closest point on the reference trajectory to current position"""
        if not self.reference_trajectory:
            return 0

        min_distance = float('inf')
        closest_index = 0

        current_x = self.current_vehicle_state.x
        current_y = self.current_vehicle_state.y

        for i, point in enumerate(self.reference_trajectory):
            distance = np.sqrt((point.x - current_x)**2 + (point.y - current_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def create_predictive_trajectory(self):
        """Create predictive trajectory horizon starting from closest reference point"""
        try:
            if not self.reference_trajectory:
                self.get_logger().warn("No reference trajectory available")
                return VehicleStateArray()

            # Validate current vehicle state before using it
            if not self._validate_vehicle_state(self.current_vehicle_state):
                self.get_logger().error(f"Current vehicle state contains invalid values: "
                                      f"x={self.current_vehicle_state.x}, y={self.current_vehicle_state.y}, "
                                      f"v={self.current_vehicle_state.v}, theta={self.current_vehicle_state.theta}")
                return VehicleStateArray()

            # Find closest point on reference trajectory
            closest_index = self.find_closest_trajectory_point()

            # Create trajectory array for MPC horizon
            trajectory_msg = VehicleStateArray()

            # Start with current vehicle state as first point
            trajectory_msg.states.append(self.current_vehicle_state)

            # Add reference trajectory points for the prediction horizon
            for i in range(1, self.horizon):
                ref_index = (closest_index + i) % len(self.reference_trajectory)
                ref_point = self.reference_trajectory[ref_index]

                # Validate reference point before using it
                if not self._validate_vehicle_state(ref_point):
                    self.get_logger().error(f"Reference point {ref_index} contains invalid values: "
                                          f"x={ref_point.x}, y={ref_point.y}, v={ref_point.v}, theta={ref_point.theta}")
                    return VehicleStateArray()

                # Create predicted state
                predicted_state = VehicleState()
                predicted_state.x = ref_point.x
                predicted_state.y = ref_point.y
                predicted_state.v = ref_point.v
                predicted_state.theta = ref_point.theta  # Include heading angle - critical for MPC!
                predicted_state.delta = 0.0  # MPC computes steering, not trajectory publisher

                trajectory_msg.states.append(predicted_state)

            return trajectory_msg
        
        except Exception as e:
            self.get_logger().error(f"Error creating predictive trajectory: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            return VehicleStateArray()

    def _validate_vehicle_state(self, state):
        """Validate that a vehicle state contains finite, reasonable values"""
        try:
            # Check for NaN or infinite values
            if not (np.isfinite(state.x) and np.isfinite(state.y) and 
                    np.isfinite(state.v) and np.isfinite(state.theta)):
                return False
            
            # Check for reasonable ranges (same as MPC node validation)
            if abs(state.x) > 1000 or abs(state.y) > 1000:  # Position check
                return False
            
            if state.v < -2 or state.v > 25:  # Velocity check (same as MPC: -2 to 25 m/s)
                return False
            
            # Theta should be reasonable (same as MPC: allows up to 10 radians)
            if abs(state.theta) > 10:  # Allow some flexibility for unnormalized angles
                return False
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error validating vehicle state: {e}")
            return False

    def create_path_msg(self):
        """Create Path message for RViz visualization"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for state in self.reference_trajectory:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation for simplicity
            path_msg.poses.append(pose)

        return path_msg

    def publish_predictive_trajectory(self):
        """Publish predictive trajectory for MPC and status"""
        # Always publish status
        status_msg = Bool()
        status_msg.data = self.path_ready
        self.status_pub.publish(status_msg)

        # Only publish trajectory data if ready and we have current state
        if self.path_ready and len(self.reference_trajectory) > 0:
            # Publish full reference path for RViz (less frequent)
            if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # ~10% of the time
                path_msg = self.create_path_msg()
                self.path_pub.publish(path_msg)
                self.log_debug("Published reference path for RViz visualization")

            # Always publish predictive trajectory for MPC
            trajectory_msg = self.create_predictive_trajectory()
            if len(trajectory_msg.states) > 0:
                # Double-check that all states in the trajectory are valid
                valid_trajectory = True
                for i, state in enumerate(trajectory_msg.states):
                    if not self._validate_vehicle_state(state):
                        self.get_logger().error(f"Invalid state at index {i} in trajectory, not publishing")
                        valid_trajectory = False
                        break
                
                if valid_trajectory:
                    self.trajectory_pub.publish(trajectory_msg)
                    self.log_debug(
                        f"Published valid predictive trajectory: {len(trajectory_msg.states)} points, "
                        f"current pos: ({self.current_vehicle_state.x:.2f}, {self.current_vehicle_state.y:.2f}), "
                        f"current theta: {self.current_vehicle_state.theta:.2f}, "
                        f"closest ref idx: {self.find_closest_trajectory_point()}")
                    
                    # Debug: Log first few trajectory points being sent to MPC
                    if self.enable_logging and len(trajectory_msg.states) > 0:
                        self.log_debug("Sample trajectory states sent to MPC:")
                        for i in range(min(3, len(trajectory_msg.states))):
                            state = trajectory_msg.states[i]
                            self.log_debug(f"  State {i}: x={state.x:.2f}, y={state.y:.2f}, v={state.v:.2f}, theta={state.theta:.2f}")
                else:
                    self.get_logger().warn("Skipping trajectory publication due to invalid states")
            else:
                self.get_logger().warn("Empty trajectory created, not publishing")
        else:
            if not self.path_ready:
                self.log_debug("Path not ready, not publishing trajectory")
            if len(self.reference_trajectory) == 0:
                self.log_debug("No reference trajectory loaded, not publishing")


def main(args=None):
    rclpy.init(args=args)
    node = HorizonMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log_info("Horizon Mapper node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
