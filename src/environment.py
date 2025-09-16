"""
Environment and Obstacle System for Basic Robotics Simulator

This module provides:
- Simulation environment with boundaries
- Various obstacle types (static and dynamic)
- Collision detection with environment
- Environment generation utilities
"""

import math
import random
from typing import List, Tuple, Optional
from .physics import Vector2D, RigidBody, PhysicsWorld, CollisionDetector


class Obstacle:
    """Base class for obstacles in the environment"""
    
    def __init__(self, position: Vector2D, obstacle_type: str = "static"):
        self.position = position
        self.obstacle_type = obstacle_type
        self.body = None  # Will be set by subclasses
    
    def get_position(self) -> Vector2D:
        return self.position.copy()
    
    def is_collision(self, other_pos: Vector2D, other_radius: float) -> bool:
        """Check if there's a collision with another circular object"""
        return False  # To be implemented by subclasses
    
    def draw_info(self) -> dict:
        """Return drawing information for visualization"""
        return {"type": "unknown", "position": self.position}


class CircularObstacle(Obstacle):
    """Circular obstacle - most common type in robotics simulations"""
    
    def __init__(self, position: Vector2D, radius: float, is_static: bool = True):
        super().__init__(position, "static" if is_static else "dynamic")
        self.radius = radius
        self.is_static = is_static
        
        # Create physics body
        mass = float('inf') if is_static else 20.0
        self.body = RigidBody(position, mass=mass, radius=radius)
    
    def is_collision(self, other_pos: Vector2D, other_radius: float) -> bool:
        """Check collision with another circular object"""
        return CollisionDetector.circle_circle_collision(
            self.position, self.radius, other_pos, other_radius
        )
    
    def update(self, dt: float):
        """Update obstacle (mainly for dynamic obstacles)"""
        if not self.is_static and self.body:
            # Update position from physics body
            self.position = self.body.position.copy()
    
    def draw_info(self) -> dict:
        return {
            "type": "circle",
            "position": self.position,
            "radius": self.radius,
            "is_static": self.is_static
        }


class RectangularObstacle(Obstacle):
    """Rectangular obstacle for walls and barriers"""
    
    def __init__(self, position: Vector2D, width: float, height: float):
        super().__init__(position, "static")
        self.width = width
        self.height = height
        self.half_width = width / 2
        self.half_height = height / 2
    
    def is_collision(self, other_pos: Vector2D, other_radius: float) -> bool:
        """Check collision with a circular object using AABB vs Circle"""
        # Convert to AABB collision detection
        closest_x = max(self.position.x - self.half_width, 
                       min(other_pos.x, self.position.x + self.half_width))
        closest_y = max(self.position.y - self.half_height, 
                       min(other_pos.y, self.position.y + self.half_height))
        
        distance = math.sqrt((other_pos.x - closest_x)**2 + (other_pos.y - closest_y)**2)
        return distance <= other_radius
    
    def get_corners(self) -> List[Vector2D]:
        """Get the four corners of the rectangle"""
        return [
            Vector2D(self.position.x - self.half_width, self.position.y - self.half_height),
            Vector2D(self.position.x + self.half_width, self.position.y - self.half_height),
            Vector2D(self.position.x + self.half_width, self.position.y + self.half_height),
            Vector2D(self.position.x - self.half_width, self.position.y + self.half_height)
        ]
    
    def get_edges(self) -> List[Tuple[Vector2D, Vector2D]]:
        """Get the four edges of the rectangle as line segments"""
        corners = self.get_corners()
        return [
            (corners[0], corners[1]),  # Bottom edge
            (corners[1], corners[2]),  # Right edge
            (corners[2], corners[3]),  # Top edge
            (corners[3], corners[0])   # Left edge
        ]
    
    def draw_info(self) -> dict:
        return {
            "type": "rectangle",
            "position": self.position,
            "width": self.width,
            "height": self.height
        }


class Wall(RectangularObstacle):
    """Wall obstacle - thin rectangular obstacle"""
    
    def __init__(self, start: Vector2D, end: Vector2D, thickness: float = 5.0):
        # Calculate center position, width, and height
        center = Vector2D((start.x + end.x) / 2, (start.y + end.y) / 2)
        length = (end - start).magnitude()
        
        # Determine if wall is more horizontal or vertical
        dx = abs(end.x - start.x)
        dy = abs(end.y - start.y)
        
        if dx > dy:  # More horizontal
            width = length
            height = thickness
        else:  # More vertical
            width = thickness
            height = length
        
        super().__init__(center, width, height)
        self.start = start
        self.end = end
        self.thickness = thickness
    
    def draw_info(self) -> dict:
        return {
            "type": "wall",
            "start": self.start,
            "end": self.end,
            "thickness": self.thickness
        }


class Environment:
    """
    Main environment class that manages obstacles and boundaries
    Represents the world where the robot operates
    """
    
    def __init__(self, width: float, height: float, has_boundaries: bool = True):
        self.width = width
        self.height = height
        self.has_boundaries = has_boundaries
        self.obstacles: List[Obstacle] = []
        
        # Create boundary walls if requested
        if has_boundaries:
            self._create_boundaries()
    
    def _create_boundaries(self):
        """Create boundary walls around the environment"""
        wall_thickness = 20.0
        half_thickness = wall_thickness / 2
        
        # Bottom wall (horizontal at bottom)
        self.obstacles.append(Wall(
            Vector2D(-half_thickness, -half_thickness), 
            Vector2D(self.width + half_thickness, -half_thickness), 
            wall_thickness
        ))
        
        # Top wall (horizontal at top)
        self.obstacles.append(Wall(
            Vector2D(-half_thickness, self.height + half_thickness), 
            Vector2D(self.width + half_thickness, self.height + half_thickness), 
            wall_thickness
        ))
        
        # Left wall (vertical at left)
        self.obstacles.append(Wall(
            Vector2D(-half_thickness, -half_thickness), 
            Vector2D(-half_thickness, self.height + half_thickness), 
            wall_thickness
        ))
        
        # Right wall (vertical at right)
        self.obstacles.append(Wall(
            Vector2D(self.width + half_thickness, -half_thickness), 
            Vector2D(self.width + half_thickness, self.height + half_thickness), 
            wall_thickness
        ))
    
    def add_obstacle(self, obstacle: Obstacle) -> None:
        """Add an obstacle to the environment"""
        self.obstacles.append(obstacle)
    
    def remove_obstacle(self, obstacle: Obstacle) -> None:
        """Remove an obstacle from the environment"""
        if obstacle in self.obstacles:
            self.obstacles.remove(obstacle)
    
    def is_collision(self, position: Vector2D, radius: float) -> bool:
        """Check if a circular object collides with any obstacle"""
        for obstacle in self.obstacles:
            if obstacle.is_collision(position, radius):
                return True
        return False
    
    def get_colliding_obstacles(self, position: Vector2D, radius: float) -> List[Obstacle]:
        """Get list of obstacles that collide with a circular object"""
        colliding = []
        for obstacle in self.obstacles:
            if obstacle.is_collision(position, radius):
                colliding.append(obstacle)
        return colliding
    
    def is_position_free(self, position: Vector2D, radius: float = 5.0) -> bool:
        """Check if a position is free of obstacles"""
        return not self.is_collision(position, radius)
    
    def find_nearest_obstacle(self, position: Vector2D) -> Tuple[Optional[Obstacle], float]:
        """Find the nearest obstacle to a given position"""
        nearest_obstacle = None
        min_distance = float('inf')
        
        for obstacle in self.obstacles:
            distance = (obstacle.position - position).magnitude()
            if isinstance(obstacle, CircularObstacle):
                distance -= obstacle.radius
            
            if distance < min_distance:
                min_distance = distance
                nearest_obstacle = obstacle
        
        return nearest_obstacle, min_distance
    
    def update(self, dt: float) -> None:
        """Update dynamic obstacles"""
        for obstacle in self.obstacles:
            if hasattr(obstacle, 'update'):
                obstacle.update(dt)
    
    def clear_obstacles(self, keep_boundaries: bool = True) -> None:
        """Clear all obstacles, optionally keeping boundaries"""
        if keep_boundaries and self.has_boundaries:
            # Keep only the first 4 obstacles (boundaries)
            self.obstacles = self.obstacles[:4]
        else:
            self.obstacles.clear()
    
    def get_random_free_position(self, radius: float = 10.0, 
                                max_attempts: int = 100) -> Optional[Vector2D]:
        """Find a random position that's free of obstacles"""
        for _ in range(max_attempts):
            x = random.uniform(radius, self.width - radius)
            y = random.uniform(radius, self.height - radius)
            position = Vector2D(x, y)
            
            if self.is_position_free(position, radius):
                return position
        
        return None  # Couldn't find free position
    
    def add_physics_obstacles(self, physics_world: PhysicsWorld) -> None:
        """Add obstacle rigid bodies to the physics world"""
        for obstacle in self.obstacles:
            if hasattr(obstacle, 'body') and obstacle.body:
                physics_world.add_body(obstacle.body)


class EnvironmentGenerator:
    """Utility class for generating different types of environments"""
    
    @staticmethod
    def create_empty_environment(width: float = 800, height: float = 600) -> Environment:
        """Create an empty environment with just boundaries"""
        return Environment(width, height, has_boundaries=True)
    
    @staticmethod
    def create_maze_environment(width: float = 800, height: float = 600, 
                               complexity: float = 0.3) -> Environment:
        """Create a maze-like environment with random obstacles"""
        env = Environment(width, height, has_boundaries=True)
        
        # Add random circular obstacles
        num_obstacles = int(complexity * (width * height) / 10000)
        
        for _ in range(num_obstacles):
            pos = env.get_random_free_position(radius=30)
            if pos:
                radius = random.uniform(15, 40)
                obstacle = CircularObstacle(pos, radius)
                env.add_obstacle(obstacle)
        
        return env
    
    @staticmethod
    def create_corridor_environment(width: float = 800, height: float = 600,
                                   corridor_width: float = 80) -> Environment:
        """Create an environment with corridor-like structure"""
        env = Environment(width, height, has_boundaries=True)
        
        # Add walls to create corridors
        wall_thickness = 10
        
        # Horizontal corridor walls
        for y in [height * 0.3, height * 0.7]:
            # Left section
            env.add_obstacle(Wall(
                Vector2D(0, y),
                Vector2D(width * 0.4, y),
                wall_thickness
            ))
            # Right section
            env.add_obstacle(Wall(
                Vector2D(width * 0.6, y),
                Vector2D(width, y),
                wall_thickness
            ))
        
        return env
    
    @staticmethod
    def create_obstacle_course(width: float = 800, height: float = 600) -> Environment:
        """Create an obstacle course for robot navigation testing"""
        env = Environment(width, height, has_boundaries=True)
        
        # Add various types of obstacles
        obstacles_data = [
            # Large central obstacle
            (Vector2D(width * 0.5, height * 0.5), 50),
            # Corner obstacles
            (Vector2D(width * 0.2, height * 0.2), 30),
            (Vector2D(width * 0.8, height * 0.8), 30),
            (Vector2D(width * 0.2, height * 0.8), 25),
            (Vector2D(width * 0.8, height * 0.2), 25),
            # Random smaller obstacles
            (Vector2D(width * 0.3, height * 0.7), 20),
            (Vector2D(width * 0.7, height * 0.3), 20),
        ]
        
        for pos, radius in obstacles_data:
            env.add_obstacle(CircularObstacle(pos, radius))
        
        # Add some walls
        env.add_obstacle(Wall(
            Vector2D(width * 0.1, height * 0.4),
            Vector2D(width * 0.3, height * 0.4),
            8
        ))
        
        env.add_obstacle(Wall(
            Vector2D(width * 0.7, height * 0.6),
            Vector2D(width * 0.9, height * 0.6),
            8
        ))
        
        return env