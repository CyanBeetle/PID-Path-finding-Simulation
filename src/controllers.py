"""
Control Systems for Basic Robotics Simulator

This module provides the core control algorithms used in the simulation:
- PrecisionMovementController: Advanced PID-based navigation with obstacle avoidance
- Emergency braking and velocity-aware control for precision movement

Key concepts demonstrated:
- PID control theory
- Velocity feedback and momentum handling  
- Adaptive obstacle avoidance
- Distance-based parameter scaling
"""

import math
from typing import Tuple, Dict, Any, List, Optional
from .physics import Vector2D
from .robot import DifferentialDriveRobot, RobotController
from .sensors import SensorArray
from .pathfinding import AStarPathfinder, NavigationGrid
from .config import CONTROLLER_CONFIG


class PrecisionMovementController(RobotController):
    """
    Precision movement controller focused on hitting targets with minimal time.
    
    Key features:
    - Distance-adaptive velocity scaling (slower near target)
    - Progressive velocity damping (more damping as target approaches)
    - Enhanced obstacle avoidance with multi-level response
    - Emergency braking system for immediate stops
    - Optimized for precision over speed initially
    
    Strategy:
    1. High precision near target (low velocity, high damping)
    2. Moderate speed far from target
    3. Smooth transition between precision and speed modes
    4. Strong obstacle avoidance that adapts to proximity
    """
    
    def __init__(self, robot: DifferentialDriveRobot, sensor_array: SensorArray, environment=None, config: Dict[str, Any] = None):
        super().__init__(robot)
        self.sensor_array = sensor_array
        self.environment = environment
        
        # Load configuration
        self.config = config if config is not None else CONTROLLER_CONFIG
        self._load_config()

        # Pathfinding setup
        self.navigation_grid = None
        self.pathfinder = None
        self.current_path = []
        self.current_waypoint_index = 0
        
        # Initialize pathfinding if environment is provided
        if environment:
            self._initialize_pathfinding()
        
        # Target management
        self.target_position = None

        # Internal state
        self.position_integral = 0.0
        self.angle_integral = 0.0
        self.prev_position_error = 0.0
        self.prev_angle_error = 0.0

    def _load_config(self):
        """Load parameters from the configuration dictionary."""
        # Pathfinding and Navigation
        self.grid_size = self.config.get("grid_size", 20.0)
        self.waypoint_tolerance = self.config["waypoint_tolerance"]
        self.target_tolerance = self.config["target_tolerance"]
        self.angle_tolerance = self.config["angle_tolerance"]
        self.lookahead_distance = self.config["lookahead_distance"]

        # Velocity Control
        self.max_velocity = self.config["max_velocity"]
        self.min_velocity = self.config["min_velocity"]
        self.max_distance_for_full_speed = self.config["max_distance_for_full_speed"]
        self.precision_distance = self.config["precision_distance"]

        # PID Gains (Position)
        self.position_kp = self.config["position_kp"]
        self.position_ki = self.config["position_ki"]
        self.position_kd = self.config["position_kd"]
        self.position_integral_clamp = self.config["position_integral_clamp"]

        # PID Gains (Angular)
        self.angle_kp = self.config["angle_kp"]
        self.angle_ki = self.config["angle_ki"]
        self.angle_kd = self.config["angle_kd"]
        self.angle_integral_clamp = self.config["angle_integral_clamp"]

        # Damping and Braking
        self.base_damping = self.config["velocity_damping_base"]
        self.max_damping = self.config["velocity_damping_max"]
        self.angular_damping = self.config["angular_damping"]
        self.brake_distance = self.config["brake_distance"]
        self.brake_force = self.config["brake_force"]
        self.max_angular_velocity = self.config.get("max_angular_velocity", 4.0)

        # Obstacle Avoidance
        self.wall_avoidance_strength = self.config["obstacle_avoidance_strength"]
        self.base_wall_avoidance_distance = self.config["wall_detection_distance"]
        self.emergency_avoidance_distance = self.config["emergency_detection_distance"]

        # Speed-adaptive parameters (can be derived or set)
        self.max_wall_avoidance_distance = self.config.get("max_wall_avoidance_distance", self.base_wall_avoidance_distance * 2)
        self.max_emergency_distance = self.config.get("max_emergency_distance", self.emergency_avoidance_distance * 2)
    
    def _initialize_pathfinding(self):
        """Initialize the pathfinding grid based on environment"""
        if not self.environment:
            return
            
        # Create navigation grid using the environment
        self.navigation_grid = NavigationGrid(self.environment, self.grid_size, robot_radius=20.0)  # Increased safety margin
        
        # Create pathfinder
        self.pathfinder = AStarPathfinder(self.navigation_grid)
    
    def _plan_path_to_target(self, start: Vector2D, goal: Vector2D) -> List[Vector2D]:
        """Plan a path from start to goal using A* pathfinding"""
        if not self.pathfinder or not self.navigation_grid:
            # Fallback to direct path if no pathfinding available
            return [goal]
        
        # Plan path using A* pathfinder (it takes Vector2D directly)
        path = self.pathfinder.find_path(start, goal)
        
        if not path:
            # No path found, return direct path as fallback
            return [goal]
        
        return path
    
    def set_target(self, target: Vector2D):
        """Set target position with safety validation, plan path, and reset control state"""
        # SAFETY CHECK: Validate target is not too close to obstacles
        validated_target = self._validate_target_safety(target)
        self.target_position = validated_target
        
        # Plan path to target using A*
        if self.environment:
            current_pos = self.robot.get_position()
            self.current_path = self._plan_path_to_target(current_pos, validated_target)
            self.current_waypoint_index = 0
        else:
            # Fallback to direct navigation
            self.current_path = [validated_target]
            self.current_waypoint_index = 0
        
        # Reset integral terms for clean approach
        self.position_integral = 0.0
        self.angle_integral = 0.0
        self.prev_position_error = 0.0
        self.prev_angle_error = 0.0
    
    def _validate_target_safety(self, target: Vector2D) -> Vector2D:
        """
        Validate and adjust target position to ensure it's safely away from obstacles
        This prevents the robot from trying to reach targets too close to obstacle edges
        """
        if not self.environment:
            return target
        
        # Safety distance from obstacles (larger than robot radius for high-speed approach)
        safety_distance = 25.0  # Increased safety margin for high-speed operation
        
        # Check if target is too close to any obstacle
        if self.environment.is_collision(target, safety_distance):
            # Find the nearest safe position
            search_radius = safety_distance
            search_step = 5.0
            
            # Search in expanding circles for a safe position
            for radius in range(int(search_step), int(search_radius * 3), int(search_step)):
                for angle in range(0, 360, 15):  # Check every 15 degrees
                    angle_rad = math.radians(angle)
                    candidate = Vector2D(
                        target.x + radius * math.cos(angle_rad),
                        target.y + radius * math.sin(angle_rad)
                    )
                    
                    # Check if this candidate position is safe
                    if not self.environment.is_collision(candidate, safety_distance):
                        # Also check it's within world bounds (basic check)
                        if (50 < candidate.x < 750 and 50 < candidate.y < 550):
                            return candidate
        
        return target  # Return original if no adjustment needed or found
    
    def get_lookahead_waypoint(self, current_pos: Vector2D, lookahead_distance: float = 40.0) -> Tuple[Vector2D, int]:
        """
        Get waypoint that's at least lookahead_distance ahead for smoother navigation
        Returns: (waypoint, waypoint_index) - the index of the waypoint being targeted
        """
        if not self.current_path:
            return self.target_position, len(self.current_path) - 1 if self.current_path else 0
        
        # Start from current waypoint index and find the farthest reachable waypoint
        target_waypoint = self.current_path[self.current_waypoint_index]
        target_index = self.current_waypoint_index
        
        # Look ahead through the path to find a good target waypoint
        for i in range(self.current_waypoint_index, len(self.current_path)):
            waypoint = self.current_path[i]
            distance = (waypoint - current_pos).magnitude()
            
            if distance >= lookahead_distance:
                return waypoint, i
            else:
                # Keep track of the farthest waypoint we can reach
                target_waypoint = waypoint
                target_index = i
        
        # Return the farthest waypoint in the path
        return target_waypoint, target_index
    
    def update(self, dt: float):
        """Update the controller, orchestrating waypoint navigation, safety, and control commands."""
        if not self.enabled or self.target_position is None or not self.current_path:
            self.robot.stop()
            return

        current_pos = self.robot.get_position()

        if not self._update_waypoint(current_pos):
            self.robot.emergency_brake()
            self.target_position = None
            return

        target_waypoint, waypoint_idx = self.get_lookahead_waypoint(current_pos, self.lookahead_distance)

        if self._perform_safety_checks(current_pos, waypoint_idx):
            return

        linear_velocity, angular_velocity, state = self._calculate_pid_commands(dt, current_pos, target_waypoint)
        
        linear_velocity, angular_velocity = self._apply_obstacle_avoidance(linear_velocity, angular_velocity)

        self._finalize_and_send_commands(linear_velocity, angular_velocity, state['distance'], state['angle_error'])

    def _update_waypoint(self, current_pos: Vector2D) -> bool:
        """
        Advances the current waypoint if the robot is close enough.
        Returns False if the end of the path is reached.
        """
        while self.current_waypoint_index < len(self.current_path) - 1:
            waypoint = self.current_path[self.current_waypoint_index]
            if (waypoint - current_pos).magnitude() < self.waypoint_tolerance:
                self.current_waypoint_index += 1
            else:
                break
        
        return self.current_waypoint_index < len(self.current_path)

    def _perform_safety_checks(self, current_pos: Vector2D, waypoint_idx: int) -> bool:
        """
        Checks for arrival, high-speed collisions, and engages emergency braking.
        Returns True if a safety override was triggered.
        """
        final_target_dist = (self.target_position - current_pos).magnitude()
        if final_target_dist < self.target_tolerance:
            self.robot.emergency_brake()
            self.target_position = None
            return True

        current_speed = self.robot.body.velocity.magnitude()
        is_final_approach = waypoint_idx >= len(self.current_path) - 1
        
        if is_final_approach and final_target_dist < self.brake_distance and current_speed > 5.0:
            self.robot.emergency_brake()
            return True

        if current_speed > 100.0:
            direction = self.robot.body.velocity.normalize()
            lookahead_pos = current_pos + direction * (current_speed * 0.3)
            if self.environment.is_collision(lookahead_pos, 15.0):
                self.robot.emergency_brake()
                return True
        
        return False

    def _calculate_pid_commands(self, dt: float, pos: Vector2D, target: Vector2D) -> Tuple[float, float, Dict]:
        """Calculates linear and angular velocities from PID controllers."""
        to_target = target - pos
        distance = to_target.magnitude()
        angle_error = self._normalize_angle(math.atan2(to_target.y, to_target.x) - self.robot.get_orientation())

        linear_velocity = self._calculate_linear_pid(dt, distance, to_target.normalize())
        angular_velocity = self._calculate_angular_pid(dt, angle_error)

        state = {'distance': distance, 'angle_error': angle_error}
        return linear_velocity, angular_velocity, state

    def _calculate_linear_pid(self, dt: float, distance: float, target_direction: Vector2D) -> float:
        """Calculates the target linear velocity."""
        self.position_integral += distance * dt
        self.position_integral = max(-self.position_integral_clamp, min(self.position_integral_clamp, self.position_integral))
        derivative = (distance - self.prev_position_error) / dt if dt > 0 else 0.0
        
        desired_velocity = (self.position_kp * distance +
                            self.position_ki * self.position_integral +
                            self.position_kd * derivative)

        velocity_scale = self._get_velocity_scale(distance)
        damping = self._get_damping(distance)
        
        robot_velocity_in_direction = self.robot.body.velocity.dot(target_direction)
        
        linear_velocity = (desired_velocity * velocity_scale) - (damping * robot_velocity_in_direction)
        
        max_vel = self.max_velocity * velocity_scale
        min_vel = self.min_velocity if distance > self.target_tolerance else 0
        
        if distance > self.precision_distance:
            return max(0, min(max_vel, linear_velocity))
        return max(min_vel, min(max_vel, linear_velocity))

    def _calculate_angular_pid(self, dt: float, angle_error: float) -> float:
        """Calculates the target angular velocity."""
        self.angle_integral += angle_error * dt
        self.angle_integral = max(-self.angle_integral_clamp, min(self.angle_integral_clamp, self.angle_integral))
        derivative = (angle_error - self.prev_angle_error) / dt if dt > 0 else 0.0

        angular_velocity = (self.angle_kp * angle_error +
                            self.angle_ki * self.angle_integral +
                            self.angle_kd * derivative)
        
        angular_velocity -= self.angular_damping * self.robot.body.angular_velocity
        return max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity))

    def _get_velocity_scale(self, distance: float) -> float:
        """Calculates a velocity scale factor based on distance to target."""
        if distance > self.max_distance_for_full_speed:
            return 1.0
        if distance < self.precision_distance:
            return (self.min_velocity / self.max_velocity) + \
                   (1 - self.min_velocity / self.max_velocity) * (distance / self.precision_distance)
        
        scale_range = (self.max_distance_for_full_speed - self.precision_distance)
        return (self.min_velocity / self.max_velocity) + \
               (1 - self.min_velocity / self.max_velocity) * ((distance - self.precision_distance) / scale_range)

    def _get_damping(self, distance: float) -> float:
        """Calculates a damping factor based on distance to target."""
        if distance < self.precision_distance:
            return self.base_damping + (self.max_damping - self.base_damping) * (1.0 - distance / self.precision_distance)
        return self.base_damping

    def _finalize_and_send_commands(self, linear: float, angular: float, dist: float, angle_err: float):
        """Clamps final velocities, sends them to the robot, and updates state."""
        final_linear = max(0, min(self.max_velocity, linear))
        final_angular = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular))

        self.robot.set_velocity_command(final_linear, final_angular)

        self.prev_position_error = dist
        self.prev_angle_error = angle_err

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to be within [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _apply_obstacle_avoidance(self, linear_velocity: float, angular_velocity: float) -> Tuple[float, float]:
        """Applies adjustments to velocity commands based on sensor readings."""
        readings = self.sensor_array.get_all_readings()
        closest_obstacle = min((d for d in readings.values() if d is not None), default=float('inf'))
        
        current_speed = self.robot.body.velocity.magnitude()
        speed_factor = min(current_speed / 150.0, 1.0)
        
        dynamic_wall_dist = self.base_wall_avoidance_distance + \
                            (self.max_wall_avoidance_distance - self.base_wall_avoidance_distance) * speed_factor

        if closest_obstacle < dynamic_wall_dist:
            obstacle_factor = closest_obstacle / dynamic_wall_dist
            speed_adjusted_factor = max(0.3, 0.5 + 0.5 * obstacle_factor - 0.2 * speed_factor)
            linear_velocity *= speed_adjusted_factor

            angular_factor = max(0.5, 0.7 + 0.3 * obstacle_factor - 0.1 * speed_factor)
            angular_velocity *= angular_factor

        avoid_linear, avoid_angular = self._calculate_avoidance_forces(readings, dynamic_wall_dist)
        
        linear_velocity += avoid_linear * self.wall_avoidance_strength
        angular_velocity += avoid_angular * self.wall_avoidance_strength
        
        return linear_velocity, angular_velocity

    def _calculate_avoidance_forces(self, readings: Dict[str, Optional[float]], wall_dist: float) -> Tuple[float, float]:
        """Calculates avoidance forces based on sensor readings."""
        linear_force = 0.0
        angular_force = 0.0
        
        emergency_dist = self.emergency_avoidance_distance

        if (front_dist := readings.get('front')) and front_dist < wall_dist:
            linear_force = -120.0 * (1 - front_dist / emergency_dist) if front_dist < emergency_dist else -60.0 * (1 - front_dist / wall_dist)

        left_push = 0.0
        if (left_dist := readings.get('left')) and left_dist < wall_dist:
            left_push = (1 - left_dist / wall_dist) * (2 if left_dist < emergency_dist else 1)
        if (front_left_dist := readings.get('front_left')) and front_left_dist < wall_dist:
            left_push += 0.7 * (1 - front_left_dist / wall_dist)

        right_push = 0.0
        if (right_dist := readings.get('right')) and right_dist < wall_dist:
            right_push = (1 - right_dist / wall_dist) * (2 if right_dist < emergency_dist else 1)
        if (front_right_dist := readings.get('front_right')) and front_right_dist < wall_dist:
            right_push += 0.7 * (1 - front_right_dist / wall_dist)

        angular_force = max(-2.0, min(2.0, (right_push - left_push) * 1.2))
        return linear_force, angular_force
    
    def is_at_target(self) -> bool:
        """Check if robot has reached the target"""
        if self.target_position is None:
            return True
        
        distance = (self.target_position - self.robot.get_position()).magnitude()
        return distance < self.target_tolerance
    
    def get_debug_info(self) -> Dict[str, Any]:
        """Get debug information for display"""
        if self.target_position is None:
            return {"status": "No target"}
        
        current_pos = self.robot.get_position()
        distance = (self.target_position - current_pos).magnitude()
        current_speed = self.robot.body.velocity.magnitude()
        
        # Get current waypoint info
        current_waypoint = None
        waypoint_distance = 0.0
        if self.current_path and self.current_waypoint_index < len(self.current_path):
            current_waypoint = self.current_path[self.current_waypoint_index]
            waypoint_distance = (current_waypoint - current_pos).magnitude()
        
        # Check for obstacles
        readings = self.sensor_array.get_all_readings()
        closest_obstacle = min([d for d in readings.values() if d is not None], default=float('inf'))
        obstacle_detected = closest_obstacle < self.base_wall_avoidance_distance
        
        # Determine status
        status = self._determine_robot_status(distance, current_speed, closest_obstacle)
        
        # Calculate current velocity scale and damping for display
        velocity_scale = 1.0
        if distance < self.precision_distance:
            velocity_scale = (self.min_velocity / self.max_velocity) + \
                             (1 - self.min_velocity / self.max_velocity) * (distance / self.precision_distance)
        
        damping_factor = self.base_damping
        if distance < self.precision_distance:
            damping_factor = self.base_damping + (self.max_damping - self.base_damping) * \
                           (1.0 - distance / self.precision_distance)
        
        return {
            "Status": status,
            "Distance": f"{distance:.1f}",
            "Speed": f"{current_speed:.1f}",
            "Velocity Scale": f"{velocity_scale:.2f}",
            "Damping": f"{damping_factor:.2f}",
            "Closest Obstacle": f"{closest_obstacle:.1f}" if closest_obstacle != float('inf') else "None",
            "Waypoint Dist": f"{waypoint_distance:.1f}" if current_waypoint else "N/A",
            "Pathfinding": "A*" if self.pathfinder else "Direct",
        }

    def _determine_robot_status(self, distance: float, speed: float, closest_obstacle: float) -> str:
        """Determine the current status of the robot for debugging."""
        if distance < self.target_tolerance:
            return "Target Reached"
        if closest_obstacle < self.emergency_avoidance_distance:
            return "Emergency Avoid"
        if closest_obstacle < self.base_wall_avoidance_distance:
            return "Avoiding Obstacle"
        if distance < self.brake_distance and speed > 5.0:
            return "Braking"
        if distance < self.precision_distance:
            return "Precision Mode"
        return "Approach Mode"