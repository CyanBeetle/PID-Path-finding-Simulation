"""
Robot Class for Basic Robotics Simulator

This module defines the Robot class which represents a mobile robot with:
- Differential drive kinematics
- Sensor capabilities
- Control interfaces
- State tracking
"""

import math
from typing import List, Tuple, Optional
from .physics import Vector2D, RigidBody, PhysicsWorld


class DifferentialDriveRobot:
    """
    A robot with differential drive kinematics (like most mobile robots)
    
    Key concepts for interviews:
    - Differential drive: Two wheels that can be controlled independently
    - Kinematics: How wheel speeds translate to robot motion
    - Dead reckoning: Estimating position from wheel encoder data
    """
    
    def __init__(self, position: Vector2D, wheel_base: float = 20.0, 
                 wheel_radius: float = 5.0, max_wheel_speed: float = 100.0):
        """
        Initialize the differential drive robot
        
        Args:
            position: Starting position in world coordinates
            wheel_base: Distance between left and right wheels
            wheel_radius: Radius of the wheels
            max_wheel_speed: Maximum angular velocity of wheels (rad/s)
        """
        # Physical properties
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.max_wheel_speed = max_wheel_speed
        
        # Create physics body
        self.body = RigidBody(position, mass=10.0, radius=wheel_base/2)
        
        # Control inputs (wheel speeds in rad/s)
        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0
        
        # Sensor data storage
        self.sensor_readings = {}
        
        # Path tracking for visualization
        self.path_history = [position.copy()]
        self.max_path_length = 1000
        
        # Robot state
        self.is_autonomous = False
        self.target_position = None
        
        # Performance metrics
        self.distance_traveled = 0.0
        self.last_position = position.copy()
    
    def set_wheel_speeds(self, left_speed: float, right_speed: float):
        """
        Set the wheel speeds (rad/s)
        This is the primary control interface for the robot
        """
        # Clamp speeds to maximum values
        self.left_wheel_speed = max(-self.max_wheel_speed, 
                                   min(self.max_wheel_speed, left_speed))
        self.right_wheel_speed = max(-self.max_wheel_speed, 
                                    min(self.max_wheel_speed, right_speed))
    
    def set_velocity_command(self, linear_velocity: float, angular_velocity: float):
        """
        Set desired linear and angular velocities (higher-level control)
        
        This converts high-level velocity commands to wheel speeds
        Key interview topic: Inverse kinematics
        """
        # Convert to wheel speeds using differential drive kinematics
        # v_left = (2*v - ω*L) / (2*R)
        # v_right = (2*v + ω*L) / (2*R)
        # where v = linear velocity, ω = angular velocity, L = wheel_base, R = wheel_radius
        
        left_wheel_speed = (2*linear_velocity - angular_velocity*self.wheel_base) / (2*self.wheel_radius)
        right_wheel_speed = (2*linear_velocity + angular_velocity*self.wheel_base) / (2*self.wheel_radius)
        
        self.set_wheel_speeds(left_wheel_speed, right_wheel_speed)
    
    def get_velocity(self) -> Tuple[float, float]:
        """
        Get current linear and angular velocities
        
        This demonstrates forward kinematics - converting wheel speeds to robot motion
        """
        # Forward kinematics for differential drive
        # v = R * (ω_left + ω_right) / 2
        # ω = R * (ω_right - ω_left) / L
        
        linear_velocity = self.wheel_radius * (self.left_wheel_speed + self.right_wheel_speed) / 2
        angular_velocity = self.wheel_radius * (self.right_wheel_speed - self.left_wheel_speed) / self.wheel_base
        
        return linear_velocity, angular_velocity
    
    def update(self, dt: float, physics_world: PhysicsWorld):
        """
        Update robot state based on wheel commands
        This is called every simulation step
        """
        # Get velocities from wheel speeds
        linear_vel, angular_vel = self.get_velocity()
        
        # Apply forces and torques to physics body
        if linear_vel != 0:
            # Apply force in forward direction
            forward_direction = self.body.get_forward_vector()
            force = forward_direction * (linear_vel * self.body.mass)
            self.body.apply_force(force)
        
        if angular_vel != 0:
            # Apply torque for rotation
            torque = angular_vel * self.body.moment_of_inertia
            self.body.apply_torque(torque)
        
        # Add some friction to make movement more realistic
        friction_coefficient = 0.2
        angular_friction_coefficient = 1.5  # Much higher for angular motion
        
        friction_force = self.body.velocity * (-friction_coefficient * self.body.mass)
        self.body.apply_force(friction_force)
        
        # Higher angular friction to reduce spinning
        angular_friction = -self.body.angular_velocity * angular_friction_coefficient * self.body.moment_of_inertia
        self.body.apply_torque(angular_friction)
        
        # Update path history
        current_pos = self.body.position.copy()
        distance_moved = (current_pos - self.last_position).magnitude()
        self.distance_traveled += distance_moved
        self.last_position = current_pos
        
        # Add to path if robot moved significantly
        if len(self.path_history) == 0 or distance_moved > 2.0:
            self.path_history.append(current_pos)
            if len(self.path_history) > self.max_path_length:
                self.path_history.pop(0)
    
    def get_position(self) -> Vector2D:
        """Get current position"""
        return self.body.position.copy()
    
    def get_orientation(self) -> float:
        """Get current orientation in radians"""
        return self.body.angle
    
    def get_pose(self) -> Tuple[float, float, float]:
        """Get current pose (x, y, theta) - common robotics representation"""
        return (self.body.position.x, self.body.position.y, self.body.angle)
    
    def distance_to(self, target: Vector2D) -> float:
        """Calculate distance to a target position"""
        return (target - self.body.position).magnitude()
    
    def angle_to(self, target: Vector2D) -> float:
        """
        Calculate angle to a target position relative to current orientation
        Returns angle in range [-π, π]
        """
        to_target = target - self.body.position
        target_angle = math.atan2(to_target.y, to_target.x)
        angle_diff = target_angle - self.body.angle
        
        # Normalize to [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff
    
    def stop(self):
        """Stop the robot by setting wheel speeds to zero"""
        self.set_wheel_speeds(0, 0)
    
    def emergency_brake(self):
        """
        Apply emergency braking forces to immediately stop the robot
        
        This simulates real braking systems that apply forces opposing motion:
        - Strong braking force opposing current linear velocity
        - Strong angular braking opposing rotation
        - Much more effective than just setting wheel speeds to zero
        """
        # Strong braking force opposing current motion
        brake_coefficient = 5.0  # Much stronger than normal friction (0.1)
        
        # Linear braking - apply force opposing current velocity
        if self.body.velocity.magnitude() > 0.1:  # Only brake if moving
            brake_force = self.body.velocity * (-brake_coefficient * self.body.mass)
            self.body.apply_force(brake_force)
        
        # Angular braking - apply torque opposing current rotation
        if abs(self.body.angular_velocity) > 0.01:  # Only brake if rotating
            angular_brake = -self.body.angular_velocity * brake_coefficient * self.body.moment_of_inertia
            self.body.apply_torque(angular_brake)
        
        # Also set wheel speeds to zero to prevent new motion
        self.set_wheel_speeds(0, 0)
    
    def is_stopped(self, velocity_threshold: float = 0.5) -> bool:
        """
        Check if robot has come to a complete stop
        
        Args:
            velocity_threshold: Speed below which robot is considered stopped
        
        Returns:
            True if robot is effectively stopped
        """
        linear_speed = self.body.velocity.magnitude()
        angular_speed = abs(self.body.angular_velocity)
        
        return linear_speed < velocity_threshold and angular_speed < 0.05
    
    def is_near_target(self, target: Vector2D, threshold: float = 10.0) -> bool:
        """Check if robot is near a target position"""
        return self.distance_to(target) < threshold
    
    def reset_path(self):
        """Clear the path history"""
        self.path_history = [self.body.position.copy()]
        self.distance_traveled = 0.0
    
    def get_bounding_box(self) -> Tuple[Vector2D, Vector2D]:
        """Get axis-aligned bounding box for collision detection"""
        pos = self.body.position
        radius = self.body.radius
        min_point = Vector2D(pos.x - radius, pos.y - radius)
        max_point = Vector2D(pos.x + radius, pos.y + radius)
        return min_point, max_point
    
    def __repr__(self):
        x, y, theta = self.get_pose()
        return f"Robot(pos=({x:.1f}, {y:.1f}), angle={math.degrees(theta):.1f}°)"


class SimpleRobot(DifferentialDriveRobot):
    """
    Simplified robot interface for basic demonstrations
    Provides higher-level movement commands
    """
    
    def move_forward(self, speed: float = 50.0):
        """Move forward at specified speed"""
        self.set_wheel_speeds(speed, speed)
    
    def move_backward(self, speed: float = 50.0):
        """Move backward at specified speed"""
        self.set_wheel_speeds(-speed, -speed)
    
    def turn_left(self, speed: float = 30.0):
        """Turn left in place"""
        self.set_wheel_speeds(-speed, speed)
    
    def turn_right(self, speed: float = 30.0):
        """Turn right in place"""
        self.set_wheel_speeds(speed, -speed)
    
    def arc_left(self, forward_speed: float = 50.0, turn_rate: float = 0.5):
        """Move in a left arc"""
        left_speed = forward_speed * (1 - turn_rate)
        right_speed = forward_speed * (1 + turn_rate)
        self.set_wheel_speeds(left_speed, right_speed)
    
    def arc_right(self, forward_speed: float = 50.0, turn_rate: float = 0.5):
        """Move in a right arc"""
        left_speed = forward_speed * (1 + turn_rate)
        right_speed = forward_speed * (1 - turn_rate)
        self.set_wheel_speeds(left_speed, right_speed)


class RobotController:
    """
    Base class for robot controllers
    This demonstrates the strategy pattern for different control algorithms
    """
    
    def __init__(self, robot: DifferentialDriveRobot):
        self.robot = robot
        self.enabled = True
    
    def update(self, dt: float) -> None:
        """Update controller - to be implemented by subclasses"""
        pass
    
    def enable(self):
        """Enable the controller"""
        self.enabled = True
    
    def disable(self):
        """Disable the controller"""
        self.enabled = False
        self.robot.stop()


class ManualController(RobotController):
    """
    Manual control for keyboard/joystick input
    Useful for teleoperation and debugging
    """
    
    def __init__(self, robot: DifferentialDriveRobot):
        super().__init__(robot)
        self.linear_command = 0.0
        self.angular_command = 0.0
        self.max_linear_speed = 80.0
        self.max_angular_speed = 2.0
    
    def set_commands(self, linear: float, angular: float):
        """Set manual control commands (-1 to 1 range)"""
        self.linear_command = max(-1, min(1, linear))
        self.angular_command = max(-1, min(1, angular))
    
    def update(self, dt: float):
        """Update robot based on manual commands"""
        if not self.enabled:
            return
        
        linear_vel = self.linear_command * self.max_linear_speed
        angular_vel = self.angular_command * self.max_angular_speed
        
        self.robot.set_velocity_command(linear_vel, angular_vel)


# Factory function for creating different robot types
def create_robot(robot_type: str = "differential", **kwargs) -> DifferentialDriveRobot:
    """
    Factory function for creating different types of robots
    Demonstrates factory pattern - common in robotics software
    """
    position = kwargs.pop('position', Vector2D(100, 100))
    
    if robot_type == "differential":
        return DifferentialDriveRobot(position, **kwargs)
    elif robot_type == "simple":
        return SimpleRobot(position, **kwargs)
    else:
        raise ValueError(f"Unknown robot type: {robot_type}")