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
    
    def __init__(self, robot: DifferentialDriveRobot, sensor_array: SensorArray, environment=None):
        super().__init__(robot)
        self.sensor_array = sensor_array
        self.environment = environment
        
        # Pathfinding setup
        self.grid_size = 10.0  # Size of each grid cell
        self.navigation_grid = None
        self.pathfinder = None
        self.current_path = []
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 15.0  # Distance to consider waypoint reached
        
        # Initialize pathfinding if environment is provided
        if environment:
            self._initialize_pathfinding()
        
        # Target management
        self.target_position = None
        self.target_tolerance = 8.0  # Tight tolerance for precision
        self.angle_tolerance = 0.05  # Very precise angle tolerance
        
        # Distance-based velocity scaling
        self.max_distance = 300.0        # Distance at which we use max velocity
        self.precision_distance = 50.0   # Distance at which precision mode kicks in
        self.max_velocity = 120.0        # Maximum velocity when far from target
        self.min_velocity = 15.0         # Minimum velocity for smooth approach
        
        # Emergency braking zone
        self.brake_distance = 15.0       # Distance to start emergency braking
        self.brake_force = 200.0         # Force applied during emergency braking
        
        # Velocity damping - increases as we get closer
        self.base_damping = 0.3          # Base damping when far
        self.max_damping = 1.2           # Maximum damping when very close
        
        # Angular control
        self.max_angular_velocity = 2.0
        self.angular_damping = 0.6
        
        # Wall avoidance (enhanced for better obstacle handling)
        self.wall_avoidance_distance = 45.0  # Increased for earlier detection
        self.wall_avoidance_strength = 0.6   # Increased for stronger response
        self.emergency_avoidance_distance = 20.0  # Critical distance for emergency maneuvers
        
        # Control gains - optimized for precision
        self.position_kp = 1.8    # Strong position response
        self.position_ki = 0.02   # Minimal integral (prevents oscillation)
        self.position_kd = 0.4    # Moderate derivative for smoothing
        
        self.angle_kp = 3.0       # Strong angular response
        self.angle_ki = 0.05      # Minimal angular integral  
        self.angle_kd = 0.5       # Good angular smoothing
        
        # Internal state
        self.position_integral = 0.0
        self.angle_integral = 0.0
        self.prev_position_error = 0.0
        self.prev_angle_error = 0.0
    
    def _initialize_pathfinding(self):
        """Initialize the pathfinding grid based on environment"""
        if not self.environment:
            return
            
        # Create navigation grid using the environment
        self.navigation_grid = NavigationGrid(self.environment, self.grid_size, robot_radius=15.0)
        
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
        """Set target position, plan path, and reset control state"""
        self.target_position = target
        
        # Plan path to target using A*
        if self.environment:
            current_pos = self.robot.get_position()
            self.current_path = self._plan_path_to_target(current_pos, target)
            self.current_waypoint_index = 0
        else:
            # Fallback to direct navigation
            self.current_path = [target]
            self.current_waypoint_index = 0
        
        # Reset integral terms for clean approach
        self.position_integral = 0.0
        self.angle_integral = 0.0
        self.prev_position_error = 0.0
        self.prev_angle_error = 0.0
    
    def update(self, dt: float):
        """Update precision movement controller with A* pathfinding"""
        if not self.enabled or self.target_position is None or not self.current_path:
            self.robot.stop()
            return
        
        current_pos = self.robot.get_position()
        
        # Get current waypoint to navigate to
        if self.current_waypoint_index >= len(self.current_path):
            # Reached end of path
            self.robot.emergency_brake()
            self.target_position = None
            return
        
        current_waypoint = self.current_path[self.current_waypoint_index]
        to_waypoint = current_waypoint - current_pos
        waypoint_distance = to_waypoint.magnitude()
        
        # Check if we've reached the current waypoint
        if waypoint_distance < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            
            # Check if this was the final waypoint
            if self.current_waypoint_index >= len(self.current_path):
                # Reached final target
                self.robot.emergency_brake()
                self.target_position = None
                return
            
            # Move to next waypoint
            current_waypoint = self.current_path[self.current_waypoint_index]
            to_waypoint = current_waypoint - current_pos
            waypoint_distance = to_waypoint.magnitude()
        
        # Check if very close to final target for precision stop
        final_target_distance = (self.target_position - current_pos).magnitude()
        if final_target_distance < self.target_tolerance:
            self.robot.emergency_brake()
            self.target_position = None
            return
        
        # EMERGENCY BRAKE ZONE: Apply active braking when very close to target
        robot_velocity = self.robot.body.velocity
        current_speed = robot_velocity.magnitude()
        
        # Use waypoint distance for control calculations
        distance = waypoint_distance
        to_target = to_waypoint
        
        if distance < self.brake_distance:
            if current_speed > 5.0:  # Only brake if moving fast enough
                # Use the robot's emergency braking system
                self.robot.emergency_brake()
        
        # Calculate desired direction
        target_direction = to_target.normalize() if distance > 0 else Vector2D(0, 0)
        
        # Get current robot state
        velocity_toward_target = robot_velocity.dot(target_direction) if distance > 0 else 0.0
        
        # Calculate angle to target
        current_angle = self.robot.get_orientation()
        target_angle = math.atan2(to_target.y, to_target.x)
        angle_error = target_angle - current_angle
        
        # Normalize angle error to [-π, π]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # === DISTANCE-ADAPTIVE VELOCITY CONTROL ===
        
        # Calculate velocity scale based on distance (1.0 = far, 0.0 = very close)
        if distance > self.max_distance:
            velocity_scale = 1.0
        elif distance < self.precision_distance:
            velocity_scale = 0.2 + 0.8 * (distance / self.precision_distance)
        else:
            velocity_scale = 0.5 + 0.5 * (distance / self.max_distance)
        
        # Calculate adaptive damping (more damping when closer)
        if distance < self.precision_distance:
            damping_factor = self.base_damping + (self.max_damping - self.base_damping) * \
                           (1.0 - distance / self.precision_distance)
        else:
            damping_factor = self.base_damping
        
        # === POSITION CONTROL WITH ADAPTIVE PARAMETERS ===
        
        # PID for position
        self.position_integral += distance * dt
        self.position_integral = max(-50, min(50, self.position_integral))  # Clamp integral
        
        position_derivative = (distance - self.prev_position_error) / dt if dt > 0 else 0.0
        
        # Base velocity command
        desired_velocity = (self.position_kp * distance + 
                          self.position_ki * self.position_integral + 
                          self.position_kd * position_derivative)
        
        # Apply distance-based scaling
        desired_velocity *= velocity_scale
        
        # Apply velocity damping (counter current momentum)
        velocity_damping = damping_factor * velocity_toward_target
        linear_velocity = desired_velocity - velocity_damping
        
        # Clamp to velocity limits
        max_vel = self.max_velocity * velocity_scale
        linear_velocity = max(self.min_velocity if distance > 5 else 0, 
                            min(max_vel, linear_velocity))
        
        # === ANGULAR CONTROL ===
        
        self.angle_integral += angle_error * dt
        self.angle_integral = max(-1, min(1, self.angle_integral))  # Tight integral clamp
        
        angle_derivative = (angle_error - self.prev_angle_error) / dt if dt > 0 else 0.0
        
        angular_velocity = (self.angle_kp * angle_error + 
                          self.angle_ki * self.angle_integral + 
                          self.angle_kd * angle_derivative)
        
        # Angular velocity damping
        current_angular_velocity = self.robot.body.angular_velocity
        angular_velocity -= self.angular_damping * current_angular_velocity
        
        # Clamp angular velocity
        angular_velocity = max(-self.max_angular_velocity, 
                             min(self.max_angular_velocity, angular_velocity))
        
        # === ENHANCED OBSTACLE AVOIDANCE ===
        
        wall_avoidance_linear, wall_avoidance_angular = self._calculate_wall_avoidance()
        
        # Apply wall avoidance with enhanced strength
        linear_velocity += wall_avoidance_linear * self.wall_avoidance_strength
        angular_velocity += wall_avoidance_angular * self.wall_avoidance_strength
        
        # OBSTACLE-ADAPTIVE BEHAVIOR: Slow down more when obstacles are detected
        readings = self.sensor_array.get_all_readings()
        closest_obstacle = min([d for d in readings.values() if d is not None], default=float('inf'))
        
        if closest_obstacle < self.wall_avoidance_distance:
            # Reduce speed proportionally to obstacle proximity
            obstacle_factor = closest_obstacle / self.wall_avoidance_distance
            linear_velocity *= (0.3 + 0.7 * obstacle_factor)  # Slow down by 30-100%
            
            # Also reduce angular velocity for more controlled movements
            angular_velocity *= (0.5 + 0.5 * obstacle_factor)
        
        # === PRECISION ADJUSTMENTS ===
        
        # Reduce speed when turning sharply for better precision
        if abs(angle_error) > math.pi/4:
            linear_velocity *= 0.3  # Slow down significantly for sharp turns
        elif abs(angle_error) > math.pi/6:
            linear_velocity *= 0.6  # Moderate slowdown for medium turns
        
        # Final velocity clamping
        linear_velocity = max(0, min(self.max_velocity, linear_velocity))
        angular_velocity = max(-self.max_angular_velocity, 
                             min(self.max_angular_velocity, angular_velocity))
        
        # Send commands to robot
        self.robot.set_velocity_command(linear_velocity, angular_velocity)
        
        # Store for next iteration
        self.prev_position_error = distance
        self.prev_angle_error = angle_error
    
    def _calculate_wall_avoidance(self) -> Tuple[float, float]:
        """Calculate enhanced wall avoidance forces with multi-level response"""
        readings = self.sensor_array.get_all_readings()
        
        linear_avoidance = 0.0
        angular_avoidance = 0.0
        
        # Get sensor readings
        front_distance = readings.get('front')
        left_distance = readings.get('left')
        right_distance = readings.get('right')
        front_left_distance = readings.get('front_left')
        front_right_distance = readings.get('front_right')
        
        # FRONT OBSTACLE HANDLING
        if front_distance and front_distance < self.wall_avoidance_distance:
            # Progressive braking based on distance
            if front_distance < self.emergency_avoidance_distance:
                # Emergency stop for very close obstacles
                linear_avoidance = -80.0 * (1.0 - front_distance / self.emergency_avoidance_distance)
            else:
                # Gradual slowdown for moderate distances
                linear_avoidance = -40.0 * (1.0 - front_distance / self.wall_avoidance_distance)
        
        # SIDE OBSTACLE HANDLING - Enhanced steering response
        left_force = 0.0
        right_force = 0.0
        
        # Left side obstacles
        if left_distance and left_distance < self.wall_avoidance_distance:
            left_force = 1.0 - left_distance / self.wall_avoidance_distance
            if left_distance < self.emergency_avoidance_distance:
                left_force *= 2.0  # Double strength for emergency
        
        # Right side obstacles  
        if right_distance and right_distance < self.wall_avoidance_distance:
            right_force = 1.0 - right_distance / self.wall_avoidance_distance
            if right_distance < self.emergency_avoidance_distance:
                right_force *= 2.0  # Double strength for emergency
        
        # FRONT CORNER SENSORS - Help with turning decisions
        if front_left_distance and front_left_distance < self.wall_avoidance_distance:
            left_force += 0.5 * (1.0 - front_left_distance / self.wall_avoidance_distance)
            
        if front_right_distance and front_right_distance < self.wall_avoidance_distance:
            right_force += 0.5 * (1.0 - front_right_distance / self.wall_avoidance_distance)
        
        # Calculate net angular avoidance
        angular_avoidance = (right_force - left_force) * 1.2  # Increased steering strength
        
        # Clamp angular avoidance to reasonable limits
        angular_avoidance = max(-2.0, min(2.0, angular_avoidance))
        
        return linear_avoidance, angular_avoidance
    
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
        obstacle_detected = closest_obstacle < self.wall_avoidance_distance
        
        # Determine status
        if distance < self.target_tolerance:
            status = "TARGET REACHED"
        elif obstacle_detected and closest_obstacle < self.emergency_avoidance_distance:
            status = "EMERGENCY AVOID"
        elif obstacle_detected:
            status = "AVOIDING OBSTACLE"
        elif distance < self.brake_distance and current_speed > 5.0:
            status = "EMERGENCY BRAKE"
        elif distance < self.precision_distance:
            status = "PRECISION MODE"
        else:
            status = "APPROACH MODE"
        
        # Calculate current velocity scale and damping
        if distance > self.max_distance:
            velocity_scale = 1.0
        elif distance < self.precision_distance:
            velocity_scale = 0.2 + 0.8 * (distance / self.precision_distance)
        else:
            velocity_scale = 0.5 + 0.5 * (distance / self.max_distance)
        
        if distance < self.precision_distance:
            damping_factor = self.base_damping + (self.max_damping - self.base_damping) * \
                           (1.0 - distance / self.precision_distance)
        else:
            damping_factor = self.base_damping
        
        return {
            "distance": f"{distance:.1f}",
            "speed": f"{current_speed:.1f}",
            "velocity_scale": f"{velocity_scale:.2f}",
            "damping": f"{damping_factor:.2f}",
            "brake_zone": "YES" if distance < self.brake_distance else "NO",
            "closest_obstacle": f"{closest_obstacle:.1f}" if closest_obstacle != float('inf') else "NONE",
            "obstacle_avoid": "YES" if obstacle_detected else "NO",
            "waypoint_dist": f"{waypoint_distance:.1f}" if current_waypoint else "NONE",
            "waypoints_left": f"{len(self.current_path) - self.current_waypoint_index}" if self.current_path else "0",
            "pathfinding": "A*" if self.pathfinder else "DIRECT",
            "status": status
        }