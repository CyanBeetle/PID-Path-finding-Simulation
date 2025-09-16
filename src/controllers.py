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
        self.grid_size = 3.0  # Smaller grid for smoother, more direct paths
        self.navigation_grid = None
        self.pathfinder = None
        self.current_path = []
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 8.0       # Reduced for more aggressive advancement
        
        # Initialize pathfinding if environment is provided
        if environment:
            self._initialize_pathfinding()
        
        # Target management
        self.target_position = None
        self.target_tolerance = 10.0  # Slightly larger for final target
        self.angle_tolerance = 0.05  # Very precise angle tolerance
        
        # Distance-based velocity scaling
        self.max_distance = 300.0        # Distance at which we use max velocity
        self.precision_distance = 50.0       # Increased for higher speeds closer to target  
        self.max_velocity = 300.0            # Increased for much faster autonomous movement
        self.min_velocity = 20.0         # Increased minimum velocity for faster approach
        
        # Emergency braking zone
        self.brake_distance = 15.0           # Slightly larger for higher speeds
        self.brake_force = 300.0         # Increased brake force for higher speeds
        
        # Velocity damping - adjusted for momentum
        self.base_damping = 0.1          # Further reduced base damping for speed 
        self.max_damping = 0.6           # Reduced maximum damping for speed
        
        # Angular control
        self.max_angular_velocity = 4.0  # Increased for faster turning
        self.angular_damping = 0.3       # Reduced angular damping for faster response
        
        # Wall avoidance (SPEED-ADAPTIVE for high-speed navigation)
        self.base_wall_avoidance_distance = 25.0  # Base detection distance
        self.max_wall_avoidance_distance = 60.0   # Extended distance at high speed
        self.wall_avoidance_strength = 0.6        # Increased for stronger response
        self.emergency_avoidance_distance = 20.0  # Critical distance for emergency maneuvers
        self.max_emergency_distance = 40.0        # Extended emergency distance at high speed
        
        # Control gains - optimized for speed while maintaining precision
        self.position_kp = 4.0    # Much higher gain for faster response
        self.position_ki = 0.08   # Slightly increased integral for steady tracking
        self.position_kd = 0.4    # Reduced derivative to avoid over-damping at speed
        
        self.angle_kp = 6.0       # Higher angular response for quick turns
        self.angle_ki = 0.05      # Slightly increased angular integral  
        self.angle_kd = 0.3       # Reduced to allow faster angular response
        
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
        """Update precision movement controller with A* pathfinding"""
        if not self.enabled or self.target_position is None or not self.current_path:
            self.robot.stop()
            return
        
        current_pos = self.robot.get_position()
        
        # PROPER WAYPOINT ADVANCEMENT: Skip waypoints that are behind us
        while (self.current_waypoint_index < len(self.current_path) - 1):
            current_waypoint = self.current_path[self.current_waypoint_index]
            waypoint_distance = (current_waypoint - current_pos).magnitude()
            
            if waypoint_distance < self.waypoint_tolerance:
                # Skip to next waypoint
                self.current_waypoint_index += 1
            else:
                break
        
        # Check if we've reached the end of the path
        if self.current_waypoint_index >= len(self.current_path):
            # Reached end of path
            self.robot.emergency_brake()
            self.target_position = None
            return
        
        # Use improved lookahead waypoint selection that tracks which waypoint we're targeting
        target_waypoint, target_waypoint_index = self.get_lookahead_waypoint(current_pos, lookahead_distance=30.0)  # Increased for high speed
        
        # CRITICAL FIX: Update current_waypoint_index based on what we're actually targeting
        # If we're targeting a waypoint ahead, advance our index appropriately
        if target_waypoint_index > self.current_waypoint_index:
            # Check if we've passed waypoints in between
            for i in range(self.current_waypoint_index, target_waypoint_index):
                intermediate_waypoint = self.current_path[i]
                dist_to_intermediate = (intermediate_waypoint - current_pos).magnitude()
                if dist_to_intermediate < self.waypoint_tolerance:
                    self.current_waypoint_index = i + 1
        
        to_waypoint = target_waypoint - current_pos
        waypoint_distance = to_waypoint.magnitude()
        
        # Check if very close to final target for precision stop
        final_target_distance = (self.target_position - current_pos).magnitude()
        if final_target_distance < self.target_tolerance:
            self.robot.emergency_brake()
            self.target_position = None
            return
        
        # ADDITIONAL SAFETY: High-speed obstacle proximity check
        current_speed = self.robot.body.velocity.magnitude()
        if current_speed > 100.0:  # Only at high speeds
            # Check if there are obstacles very close in movement direction
            velocity_direction = self.robot.body.velocity.normalize() if current_speed > 0 else Vector2D(1, 0)
            look_ahead_distance = current_speed * 0.3  # Look ahead based on speed
            look_ahead_pos = current_pos + velocity_direction * look_ahead_distance
            
            # If obstacle detected in movement direction, apply emergency braking
            if self.environment.is_collision(look_ahead_pos, 15.0):
                self.robot.emergency_brake()
                return
        
        # EMERGENCY BRAKE ZONE: Apply active braking when very close to FINAL target only
        robot_velocity = self.robot.body.velocity
        current_speed = robot_velocity.magnitude()
        
        # Use waypoint distance for control calculations
        distance = waypoint_distance
        to_target = to_waypoint
        
        # ONLY brake at final target, not at waypoints
        is_final_approach = (target_waypoint_index >= len(self.current_path) - 1)

        if is_final_approach and final_target_distance < self.brake_distance:
            # Apply emergency braking only for final target
            if current_speed > 5.0:
                self.robot.emergency_brake()
                return
        
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
        
        # Calculate velocity scale based on distance - MUCH LESS CONSERVATIVE
        if distance > self.max_distance:
            velocity_scale = 1.0  # Full speed when far
        elif distance < self.precision_distance:
            velocity_scale = 0.8 + 0.2 * (distance / self.precision_distance)  # 80-100% instead of 50-100%
        else:
            velocity_scale = 0.9 + 0.1 * (distance / self.max_distance)       # 90-100% instead of 70-100%
        
        # NEW: Angle-based speed scaling - only slow down for sharp turns
        angle_magnitude = abs(angle_error)
        if angle_magnitude > math.pi/3:  # Sharp turn (>60 degrees)
            angle_speed_factor = 0.6  # Slow down significantly
        elif angle_magnitude > math.pi/6:  # Medium turn (>30 degrees)
            angle_speed_factor = 0.8  # Moderate slowdown
        else:  # Small turn (<30 degrees)
            angle_speed_factor = 1.0  # Full speed when well-aligned
        
        # Apply both distance and angle scaling (take minimum for safety)
        velocity_scale = min(velocity_scale, angle_speed_factor)
        
        # Calculate adaptive damping - REDUCED for faster movement
        if distance < self.precision_distance:
            damping_factor = self.base_damping + (self.max_damping - self.base_damping) * \
                           (1.0 - distance / self.precision_distance) * 0.5  # Reduce damping effect by 50%
        else:
            damping_factor = self.base_damping * 0.5  # Much less damping when far
        
        # === POSITION CONTROL WITH ADAPTIVE PARAMETERS ===
        
        # PID for position
        self.position_integral += distance * dt
        self.position_integral = max(-50, min(50, self.position_integral))  # Clamp integral
        
        position_derivative = (distance - self.prev_position_error) / dt if dt > 0 else 0.0
        
        # Base velocity command
        desired_velocity = (self.position_kp * distance + 
                          self.position_ki * self.position_integral + 
                          self.position_kd * position_derivative)
        
        # Add feedforward term for better high-speed tracking
        # This reduces lag by anticipating the required velocity
        feedforward_velocity = self.max_velocity * 0.7 if distance > 50 else 0
        desired_velocity += feedforward_velocity
        
        # Apply distance-based scaling
        desired_velocity *= velocity_scale
        
        # Apply velocity damping (counter current momentum)
        velocity_damping = damping_factor * velocity_toward_target
        linear_velocity = desired_velocity - velocity_damping
        
        # Clamp to velocity limits - LESS RESTRICTIVE
        max_vel = self.max_velocity * velocity_scale
        # Remove aggressive min_velocity enforcement except when very close
        if distance > 15:  # Only enforce min velocity when close
            linear_velocity = max(0, min(max_vel, linear_velocity))  # Allow zero velocity when far
        else:
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
        
        # === SPEED-ADAPTIVE OBSTACLE AVOIDANCE ===
        
        # Calculate speed-adaptive avoidance distances
        current_speed = self.robot.body.velocity.magnitude()
        speed_factor = min(current_speed / 150.0, 1.0)  # Normalize to max expected speed
        
        # Increase avoidance distances based on current speed
        dynamic_wall_distance = self.base_wall_avoidance_distance + \
                               (self.max_wall_avoidance_distance - self.base_wall_avoidance_distance) * speed_factor
        dynamic_emergency_distance = self.emergency_avoidance_distance + \
                                    (self.max_emergency_distance - self.emergency_avoidance_distance) * speed_factor
        
        wall_avoidance_linear, wall_avoidance_angular = self._calculate_wall_avoidance(dynamic_wall_distance, dynamic_emergency_distance)
        
        # Apply wall avoidance with enhanced strength
        linear_velocity += wall_avoidance_linear * self.wall_avoidance_strength
        angular_velocity += wall_avoidance_angular * self.wall_avoidance_strength
        
        # SPEED-ADAPTIVE OBSTACLE SLOWDOWN: More aggressive at high speeds
        readings = self.sensor_array.get_all_readings()
        closest_obstacle = min([d for d in readings.values() if d is not None], default=float('inf'))
        
        # Use dynamic wall distance for obstacle slowdown
        if closest_obstacle < dynamic_wall_distance:
            # More aggressive slowdown based on current speed
            obstacle_factor = closest_obstacle / dynamic_wall_distance
            speed_adjusted_factor = max(0.3, 0.5 + 0.5 * obstacle_factor - 0.2 * speed_factor)
            linear_velocity *= speed_adjusted_factor
            
            # Reduce angular velocity for more controlled movements at high speeds
            angular_factor = max(0.5, 0.7 + 0.3 * obstacle_factor - 0.1 * speed_factor)
            angular_velocity *= angular_factor
        
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
    
    def _calculate_wall_avoidance(self, wall_distance: float = None, emergency_distance: float = None) -> Tuple[float, float]:
        """Calculate speed-adaptive wall avoidance forces with multi-level response"""
        readings = self.sensor_array.get_all_readings()
        
        # Use provided distances or defaults
        if wall_distance is None:
            wall_distance = self.base_wall_avoidance_distance
        if emergency_distance is None:
            emergency_distance = self.emergency_avoidance_distance
        
        linear_avoidance = 0.0
        angular_avoidance = 0.0
        
        # Get sensor readings
        front_distance = readings.get('front')
        left_distance = readings.get('left')
        right_distance = readings.get('right')
        front_left_distance = readings.get('front_left')
        front_right_distance = readings.get('front_right')
        
        # SPEED-ADAPTIVE FRONT OBSTACLE HANDLING
        if front_distance and front_distance < wall_distance:
            # Progressive braking based on distance - more aggressive at high speeds
            if front_distance < emergency_distance:
                # Emergency stop for very close obstacles - stronger at high speeds
                linear_avoidance = -120.0 * (1.0 - front_distance / emergency_distance)
            else:
                # Gradual slowdown for moderate distances - more responsive
                linear_avoidance = -60.0 * (1.0 - front_distance / wall_distance)
        
        # SIDE OBSTACLE HANDLING - Enhanced steering response
        left_force = 0.0
        right_force = 0.0
        
        # Left side obstacles - with speed-adaptive distances
        if left_distance and left_distance < wall_distance:
            left_force = 1.0 - left_distance / wall_distance
            if left_distance < emergency_distance:
                left_force *= 2.0  # Double strength for emergency
        
        # Right side obstacles - with speed-adaptive distances  
        if right_distance and right_distance < wall_distance:
            right_force = 1.0 - right_distance / wall_distance
            if right_distance < emergency_distance:
                right_force *= 2.0  # Double strength for emergency
        
        # FRONT CORNER SENSORS - Help with turning decisions
        if front_left_distance and front_left_distance < wall_distance:
            left_force += 0.7 * (1.0 - front_left_distance / wall_distance)
            
        if front_right_distance and front_right_distance < wall_distance:
            right_force += 0.7 * (1.0 - front_right_distance / wall_distance)
        
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
        obstacle_detected = closest_obstacle < self.base_wall_avoidance_distance
        
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
            velocity_scale = 0.5 + 0.5 * (distance / self.precision_distance)  # 50-100% instead of 20-100%
        else:
            velocity_scale = 0.7 + 0.3 * (distance / self.max_distance)       # 70-100% instead of 50-100%
        
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
            "current_wp_idx": f"{self.current_waypoint_index}",
            "waypoints_left": f"{len(self.current_path) - self.current_waypoint_index}" if self.current_path else "0",
            "pathfinding": "A*" if self.pathfinder else "DIRECT",
            "status": status
        }