"""
Physics Engine for Basic Robotics Simulator

This module provides the core physics simulation including:
- 2D vector operations and physics
- Collision detection
- Robot dynamics and kinematics
- Time integration methods
"""

import numpy as np
import math
from typing import Tuple, List, Optional


class Vector2D:
    """2D Vector class for position, velocity, and acceleration calculations"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y
    
    def __add__(self, other: 'Vector2D') -> 'Vector2D':
        return Vector2D(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other: 'Vector2D') -> 'Vector2D':
        return Vector2D(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar: float) -> 'Vector2D':
        return Vector2D(self.x * scalar, self.y * scalar)
    
    def __rmul__(self, scalar: float) -> 'Vector2D':
        return self.__mul__(scalar)
    
    def magnitude(self) -> float:
        """Calculate the magnitude (length) of the vector"""
        return math.sqrt(self.x**2 + self.y**2)
    
    def normalize(self) -> 'Vector2D':
        """Return a normalized (unit) vector"""
        mag = self.magnitude()
        if mag == 0:
            return Vector2D(0, 0)
        return Vector2D(self.x / mag, self.y / mag)
    
    def dot(self, other: 'Vector2D') -> float:
        """Calculate dot product with another vector"""
        return self.x * other.x + self.y * other.y
    
    def to_tuple(self) -> Tuple[float, float]:
        """Convert to tuple for easy use with pygame"""
        return (self.x, self.y)
    
    def copy(self) -> 'Vector2D':
        """Create a copy of this vector"""
        return Vector2D(self.x, self.y)
    
    def __repr__(self):
        return f"Vector2D({self.x:.2f}, {self.y:.2f})"


class RigidBody:
    """
    Represents a rigid body with physics properties
    Fundamental for robot simulation - handles position, velocity, forces
    """
    
    def __init__(self, position: Vector2D, mass: float = 1.0, radius: float = 10.0):
        self.position = position.copy()
        self.velocity = Vector2D(0, 0)
        self.acceleration = Vector2D(0, 0)
        self.mass = mass
        self.radius = radius  # For collision detection
        
        # Angular properties
        self.angle = 0.0  # Orientation in radians
        self.angular_velocity = 0.0
        self.angular_acceleration = 0.0
        self.moment_of_inertia = 0.5 * mass * radius**2  # For circular body
        
        # Forces and torques
        self.forces = []
        self.torques = []
    
    def apply_force(self, force: Vector2D, application_point: Optional[Vector2D] = None):
        """Apply a force to the rigid body"""
        self.forces.append(force)
        
        # If application point is specified, calculate torque
        if application_point is not None:
            r = application_point - self.position
            # Torque = r x F (cross product in 2D gives scalar)
            torque = r.x * force.y - r.y * force.x
            self.torques.append(torque)
    
    def apply_torque(self, torque: float):
        """Apply a torque (rotational force) to the rigid body"""
        self.torques.append(torque)
    
    def update(self, dt: float):
        """
        Update physics using Euler integration
        This is the core physics update loop
        """
        # Calculate net force and acceleration
        net_force = Vector2D(0, 0)
        for force in self.forces:
            net_force = net_force + force
        
        # F = ma, so a = F/m
        self.acceleration = net_force * (1.0 / self.mass)
        
        # Update linear motion
        self.velocity = self.velocity + self.acceleration * dt
        self.position = self.position + self.velocity * dt
        
        # Calculate net torque and angular acceleration
        net_torque = sum(self.torques)
        self.angular_acceleration = net_torque / self.moment_of_inertia
        
        # Update angular motion
        self.angular_velocity += self.angular_acceleration * dt
        self.angle += self.angular_velocity * dt
        
        # Normalize angle to [0, 2π]
        self.angle = self.angle % (2 * math.pi)
        
        # Clear forces and torques for next frame
        self.forces.clear()
        self.torques.clear()
    
    def get_forward_vector(self) -> Vector2D:
        """Get the forward direction vector based on current orientation"""
        return Vector2D(math.cos(self.angle), math.sin(self.angle))
    
    def get_right_vector(self) -> Vector2D:
        """Get the right direction vector based on current orientation"""
        return Vector2D(-math.sin(self.angle), math.cos(self.angle))


class CollisionDetector:
    """
    Handles collision detection between different geometric shapes
    Essential for robotics - robots need to avoid obstacles
    """
    
    @staticmethod
    def circle_circle_collision(pos1: Vector2D, radius1: float, 
                               pos2: Vector2D, radius2: float) -> bool:
        """Check collision between two circles"""
        distance = (pos1 - pos2).magnitude()
        return distance <= (radius1 + radius2)
    
    @staticmethod
    def circle_point_collision(circle_pos: Vector2D, radius: float, 
                              point: Vector2D) -> bool:
        """Check if a point is inside a circle"""
        distance = (circle_pos - point).magnitude()
        return distance <= radius
    
    @staticmethod
    def circle_line_collision(circle_pos: Vector2D, radius: float,
                             line_start: Vector2D, line_end: Vector2D) -> bool:
        """Check collision between circle and line segment"""
        # Vector from line start to circle center
        start_to_circle = circle_pos - line_start
        # Line direction vector
        line_dir = line_end - line_start
        line_length = line_dir.magnitude()
        
        if line_length == 0:
            # Line is a point
            return CollisionDetector.circle_point_collision(circle_pos, radius, line_start)
        
        # Normalize line direction
        line_dir_norm = line_dir.normalize()
        
        # Project circle center onto line
        projection_length = start_to_circle.dot(line_dir_norm)
        
        # Clamp projection to line segment
        projection_length = max(0, min(line_length, projection_length))
        
        # Find closest point on line segment
        closest_point = line_start + line_dir_norm * projection_length
        
        # Check distance to closest point
        return CollisionDetector.circle_point_collision(circle_pos, radius, closest_point)

    @staticmethod
    def circle_rectangle_collision(circle_pos: Vector2D, radius: float,
                                  rect_center: Vector2D, rect_width: float, rect_height: float) -> bool:
        """Check collision between circle and axis-aligned rectangle"""
        half_width = rect_width / 2
        half_height = rect_height / 2
        
        # Find closest point on rectangle to circle center
        closest_x = max(rect_center.x - half_width, 
                       min(circle_pos.x, rect_center.x + half_width))
        closest_y = max(rect_center.y - half_height, 
                       min(circle_pos.y, rect_center.y + half_height))
        
        # Check if distance to closest point is within radius
        distance_sq = (circle_pos.x - closest_x)**2 + (circle_pos.y - closest_y)**2
        return distance_sq <= radius * radius

    @staticmethod
    def resolve_circle_rectangle_collision(circle_body: RigidBody, 
                                         rect_center: Vector2D, rect_width: float, rect_height: float) -> None:
        """Resolve collision between circle and rectangle"""
        half_width = rect_width / 2
        half_height = rect_height / 2
        
        # Find closest point on rectangle
        closest_x = max(rect_center.x - half_width, 
                       min(circle_body.position.x, rect_center.x + half_width))
        closest_y = max(rect_center.y - half_height, 
                       min(circle_body.position.y, rect_center.y + half_height))
        
        closest_point = Vector2D(closest_x, closest_y)
        
        # Calculate penetration vector
        penetration = circle_body.position - closest_point
        penetration_distance = penetration.magnitude()
        
        if penetration_distance == 0:
            # Circle center is exactly on rectangle boundary, push up
            penetration = Vector2D(0, 1)
            penetration_distance = 1
        
        # Normalize penetration vector
        penetration_normal = penetration.normalize()
        
        # Calculate required separation
        required_separation = circle_body.radius
        overlap = required_separation - penetration_distance
        
        if overlap > 0:
            # Move circle out of rectangle
            separation = penetration_normal * overlap
            circle_body.position = circle_body.position + separation
            
            # Reflect velocity off the wall (elastic collision)
            # Remove component of velocity in penetration direction
            velocity_dot_normal = circle_body.velocity.dot(penetration_normal)
            if velocity_dot_normal < 0:  # Moving towards wall
                circle_body.velocity = circle_body.velocity - penetration_normal * (2 * velocity_dot_normal)
    
    @staticmethod
    def resolve_circle_collision(body1: RigidBody, body2: RigidBody) -> None:
        """
        Resolve collision between two circular rigid bodies
        Implements elastic collision with conservation of momentum
        """
        # Calculate collision normal
        collision_normal = (body2.position - body1.position).normalize()
        
        # Separate objects to prevent overlap
        overlap = body1.radius + body2.radius - (body2.position - body1.position).magnitude()
        if overlap > 0:
            separation = collision_normal * (overlap / 2)
            body1.position = body1.position - separation
            body2.position = body2.position + separation
        
        # Calculate relative velocity
        relative_velocity = body2.velocity - body1.velocity
        velocity_along_normal = relative_velocity.dot(collision_normal)
        
        # Do not resolve if velocities are separating
        if velocity_along_normal > 0:
            return
        
        # Calculate restitution (bounciness) - using 0.8 for realistic bouncing
        restitution = 0.8
        
        # Calculate impulse scalar
        impulse_scalar = -(1 + restitution) * velocity_along_normal
        impulse_scalar /= (1/body1.mass + 1/body2.mass)
        
        # Apply impulse
        impulse = collision_normal * impulse_scalar
        body1.velocity = body1.velocity - impulse * (1/body1.mass)
        body2.velocity = body2.velocity + impulse * (1/body2.mass)


class PhysicsWorld:
    """
    Main physics simulation world that manages all rigid bodies
    Handles time stepping and collision detection/resolution
    """
    
    def __init__(self, gravity: Vector2D = Vector2D(0, 0)):
        self.bodies: List[RigidBody] = []
        self.gravity = gravity
        self.time = 0.0
    
    def add_body(self, body: RigidBody):
        """Add a rigid body to the simulation"""
        self.bodies.append(body)
    
    def remove_body(self, body: RigidBody):
        """Remove a rigid body from the simulation"""
        if body in self.bodies:
            self.bodies.remove(body)
    
    def step(self, dt: float):
        """
        Advance the simulation by one time step
        This is called every frame to update the physics
        """
        # Apply gravity to all bodies
        for body in self.bodies:
            if self.gravity.magnitude() > 0:
                body.apply_force(self.gravity * body.mass)
        
        # Update all bodies
        for body in self.bodies:
            body.update(dt)
        
        # Check and resolve collisions
        self._handle_collisions()
        
        self.time += dt
    
    def _handle_collisions(self):
        """Check and resolve all collisions between bodies"""
        # Handle body-to-body collisions
        for i in range(len(self.bodies)):
            for j in range(i + 1, len(self.bodies)):
                body1 = self.bodies[i]
                body2 = self.bodies[j]
                
                if CollisionDetector.circle_circle_collision(
                    body1.position, body1.radius,
                    body2.position, body2.radius
                ):
                    CollisionDetector.resolve_circle_collision(body1, body2)
    
    def handle_environment_collisions(self, environment):
        """Handle collisions between bodies and environment obstacles"""
        from .environment import Wall, RectangularObstacle
        
        for body in self.bodies:
            for obstacle in environment.obstacles:
                if isinstance(obstacle, (Wall, RectangularObstacle)):
                    if CollisionDetector.circle_rectangle_collision(
                        body.position, body.radius,
                        obstacle.position, obstacle.width, obstacle.height
                    ):
                        CollisionDetector.resolve_circle_rectangle_collision(
                            body, obstacle.position, obstacle.width, obstacle.height
                        )
    
    def query_collision(self, position: Vector2D, radius: float) -> List[RigidBody]:
        """Query which bodies collide with a given circle"""
        colliding_bodies = []
        for body in self.bodies:
            if CollisionDetector.circle_circle_collision(
                position, radius, body.position, body.radius
            ):
                colliding_bodies.append(body)
        return colliding_bodies