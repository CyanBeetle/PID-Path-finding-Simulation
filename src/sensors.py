"""
Sensor System for Basic Robotics Simulator

This module provides various sensor implementations:
- Distance sensors (lidar-like)
- Camera simulation
- Proximity sensors
- Encoder sensors for odometry

Key concepts for AI-robotics interviews:
- Sensor noise and uncertainty
- Sensor fusion
- Range finding algorithms
- Perception pipeline
"""

import math
import random
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from .physics import Vector2D, CollisionDetector
from .environment import Environment, Obstacle


class Sensor:
    """Base class for all sensors"""
    
    def __init__(self, position: Vector2D, orientation: float = 0.0, 
                 noise_level: float = 0.0):
        self.position = position  # Relative to robot
        self.orientation = orientation  # Relative to robot orientation
        self.noise_level = noise_level
        self.last_reading = None
        self.enabled = True
    
    def update(self, robot_position: Vector2D, robot_orientation: float, 
               environment: Environment) -> Any:
        """Update sensor and return reading"""
        if not self.enabled:
            return None
        
        # Calculate absolute sensor position and orientation
        abs_position = self._get_absolute_position(robot_position, robot_orientation)
        abs_orientation = robot_orientation + self.orientation
        
        # Get raw reading
        reading = self._get_reading(abs_position, abs_orientation, environment)
        
        # Add noise if specified
        if self.noise_level > 0 and reading is not None:
            reading = self._add_noise(reading)
        
        self.last_reading = reading
        return reading
    
    def _get_absolute_position(self, robot_position: Vector2D, robot_orientation: float) -> Vector2D:
        """Calculate absolute sensor position based on robot pose"""
        # Rotate sensor position by robot orientation
        cos_theta = math.cos(robot_orientation)
        sin_theta = math.sin(robot_orientation)
        
        rotated_x = self.position.x * cos_theta - self.position.y * sin_theta
        rotated_y = self.position.x * sin_theta + self.position.y * cos_theta
        
        return Vector2D(robot_position.x + rotated_x, robot_position.y + rotated_y)
    
    def _get_reading(self, position: Vector2D, orientation: float, 
                    environment: Environment) -> Any:
        """Get raw sensor reading - to be implemented by subclasses"""
        raise NotImplementedError
    
    def _add_noise(self, reading: Any) -> Any:
        """Add noise to sensor reading - to be implemented by subclasses"""
        return reading
    
    def enable(self):
        """Enable the sensor"""
        self.enabled = True
    
    def disable(self):
        """Disable the sensor"""
        self.enabled = False


class DistanceSensor(Sensor):
    """
    Distance sensor (like ultrasonic or laser rangefinder)
    
    Key interview topics:
    - Ray casting for distance measurement
    - Sensor limitations (max range, beam width)
    - Noise modeling in sensors
    """
    
    def __init__(self, position: Vector2D, orientation: float = 0.0,
                 max_range: float = 200.0, beam_width: float = 0.1,
                 noise_level: float = 0.02):
        super().__init__(position, orientation, noise_level)
        self.max_range = max_range
        self.beam_width = beam_width  # Angular width of sensor beam (radians)
    
    def _get_reading(self, position: Vector2D, orientation: float, 
                    environment: Environment) -> float:
        """Cast a ray and find the nearest obstacle"""
        min_distance = self.max_range
        
        # Cast multiple rays within beam width for more realistic sensing
        num_rays = 5
        angle_step = self.beam_width / (num_rays - 1) if num_rays > 1 else 0
        start_angle = orientation - self.beam_width / 2
        
        for i in range(num_rays):
            ray_angle = start_angle + i * angle_step
            distance = self._cast_ray(position, ray_angle, environment)
            min_distance = min(min_distance, distance)
        
        return min_distance
    
    def _cast_ray(self, start: Vector2D, angle: float, environment: Environment) -> float:
        """Cast a single ray and find intersection distance"""
        ray_dir = Vector2D(math.cos(angle), math.sin(angle))
        
        min_distance = self.max_range
        step_size = 2.0  # Step size for ray marching
        current_distance = 0.0
        
        # Ray marching approach for simplicity
        while current_distance < self.max_range:
            current_pos = start + ray_dir * current_distance
            
            # Check collision with any obstacle
            for obstacle in environment.obstacles:
                if obstacle.is_collision(current_pos, 1.0):  # Small radius for ray
                    return current_distance
            
            current_distance += step_size
        
        return self.max_range
    
    def _add_noise(self, reading: float) -> float:
        """Add Gaussian noise to distance reading"""
        noise = random.gauss(0, self.noise_level * reading)
        return max(0, reading + noise)
    
    def get_detection_cone(self, robot_position: Vector2D, robot_orientation: float) -> List[Vector2D]:
        """Get points representing the sensor detection cone for visualization"""
        abs_position = self._get_absolute_position(robot_position, robot_orientation)
        abs_orientation = robot_orientation + self.orientation
        
        # Create cone points
        points = [abs_position]
        
        num_points = 10
        for i in range(num_points + 1):
            angle = abs_orientation - self.beam_width/2 + (i * self.beam_width / num_points)
            end_point = abs_position + Vector2D(
                math.cos(angle) * self.max_range,
                math.sin(angle) * self.max_range
            )
            points.append(end_point)
        
        return points


class LidarSensor(Sensor):
    """
    360-degree lidar sensor
    
    Returns distance measurements in all directions
    Key for SLAM and localization algorithms
    """
    
    def __init__(self, position: Vector2D, max_range: float = 300.0,
                 angular_resolution: float = 1.0, noise_level: float = 0.01):
        super().__init__(position, 0.0, noise_level)
        self.max_range = max_range
        self.angular_resolution = math.radians(angular_resolution)  # Convert to radians
        self.num_beams = int(2 * math.pi / self.angular_resolution)
    
    def _get_reading(self, position: Vector2D, orientation: float, 
                    environment: Environment) -> List[float]:
        """Get 360-degree distance measurements"""
        distances = []
        
        for i in range(self.num_beams):
            angle = i * self.angular_resolution
            distance = self._cast_ray(position, angle, environment)
            distances.append(distance)
        
        return distances
    
    def _cast_ray(self, start: Vector2D, angle: float, environment: Environment) -> float:
        """Cast a ray and find intersection distance"""
        ray_dir = Vector2D(math.cos(angle), math.sin(angle))
        
        step_size = 2.0
        current_distance = 0.0
        
        while current_distance < self.max_range:
            current_pos = start + ray_dir * current_distance
            
            # Check collision with any obstacle
            for obstacle in environment.obstacles:
                if obstacle.is_collision(current_pos, 1.0):
                    return current_distance
            
            current_distance += step_size
        
        return self.max_range
    
    def _add_noise(self, reading: List[float]) -> List[float]:
        """Add noise to all distance readings"""
        noisy_reading = []
        for distance in reading:
            noise = random.gauss(0, self.noise_level * distance)
            noisy_reading.append(max(0, distance + noise))
        return noisy_reading
    
    def get_points_cloud(self, robot_position: Vector2D, robot_orientation: float) -> List[Vector2D]:
        """Convert lidar readings to 2D point cloud"""
        if self.last_reading is None:
            return []
        
        abs_position = self._get_absolute_position(robot_position, robot_orientation)
        points = []
        
        for i, distance in enumerate(self.last_reading):
            if distance < self.max_range:
                angle = i * self.angular_resolution
                point = abs_position + Vector2D(
                    math.cos(angle) * distance,
                    math.sin(angle) * distance
                )
                points.append(point)
        
        return points


class ProximitySensor(Sensor):
    """
    Simple proximity sensor that detects nearby obstacles
    Similar to bumper sensors or infrared proximity sensors
    """
    
    def __init__(self, position: Vector2D, orientation: float = 0.0,
                 detection_range: float = 30.0, detection_angle: float = math.pi/4):
        super().__init__(position, orientation, 0.0)  # Usually no noise for proximity
        self.detection_range = detection_range
        self.detection_angle = detection_angle
    
    def _get_reading(self, position: Vector2D, orientation: float, 
                    environment: Environment) -> bool:
        """Check if any obstacle is within detection range and angle"""
        for obstacle in environment.obstacles:
            # Calculate distance to obstacle
            distance = (obstacle.position - position).magnitude()
            
            # Check if within range
            if distance <= self.detection_range:
                # Calculate angle to obstacle
                to_obstacle = obstacle.position - position
                obstacle_angle = math.atan2(to_obstacle.y, to_obstacle.x)
                angle_diff = abs(obstacle_angle - orientation)
                
                # Normalize angle difference
                angle_diff = min(angle_diff, 2*math.pi - angle_diff)
                
                # Check if within detection angle
                if angle_diff <= self.detection_angle / 2:
                    return True
        
        return False


class CameraSensor(Sensor):
    """
    Simple camera sensor simulation
    
    Detects obstacles within field of view
    Can be extended for more sophisticated vision processing
    """
    
    def __init__(self, position: Vector2D, orientation: float = 0.0,
                 field_of_view: float = math.pi/3, max_range: float = 150.0,
                 resolution: int = 64):
        super().__init__(position, orientation, 0.02)
        self.field_of_view = field_of_view
        self.max_range = max_range
        self.resolution = resolution
    
    def _get_reading(self, position: Vector2D, orientation: float, 
                    environment: Environment) -> Dict[str, Any]:
        """
        Get camera reading as a simplified image representation
        Returns detected obstacles with their positions and sizes
        """
        detected_objects = []
        
        # Scan within field of view
        angle_step = self.field_of_view / self.resolution
        start_angle = orientation - self.field_of_view / 2
        
        for i in range(self.resolution):
            ray_angle = start_angle + i * angle_step
            distance = self._cast_ray(position, ray_angle, environment)
            
            if distance < self.max_range:
                # Calculate object position
                object_pos = position + Vector2D(
                    math.cos(ray_angle) * distance,
                    math.sin(ray_angle) * distance
                )
                
                detected_objects.append({
                    'position': object_pos,
                    'distance': distance,
                    'angle': ray_angle - orientation,  # Relative to camera
                    'pixel': i
                })
        
        return {
            'objects': detected_objects,
            'timestamp': 0,  # Could add real timestamp
            'fov': self.field_of_view,
            'resolution': self.resolution
        }
    
    def _cast_ray(self, start: Vector2D, angle: float, environment: Environment) -> float:
        """Cast a ray and find intersection distance"""
        ray_dir = Vector2D(math.cos(angle), math.sin(angle))
        
        step_size = 2.0
        current_distance = 0.0
        
        while current_distance < self.max_range:
            current_pos = start + ray_dir * current_distance
            
            for obstacle in environment.obstacles:
                if obstacle.is_collision(current_pos, 1.0):
                    return current_distance
            
            current_distance += step_size
        
        return self.max_range


class SensorArray:
    """
    Manages multiple sensors on a robot
    Provides sensor fusion capabilities
    """
    
    def __init__(self):
        self.sensors: Dict[str, Sensor] = {}
        self.readings: Dict[str, Any] = {}
    
    def add_sensor(self, name: str, sensor: Sensor):
        """Add a sensor to the array"""
        self.sensors[name] = sensor
    
    def remove_sensor(self, name: str):
        """Remove a sensor from the array"""
        if name in self.sensors:
            del self.sensors[name]
        if name in self.readings:
            del self.readings[name]
    
    def update_all(self, robot_position: Vector2D, robot_orientation: float, 
                   environment: Environment):
        """Update all sensors and store readings"""
        for name, sensor in self.sensors.items():
            reading = sensor.update(robot_position, robot_orientation, environment)
            self.readings[name] = reading
    
    def get_reading(self, sensor_name: str) -> Any:
        """Get the latest reading from a specific sensor"""
        return self.readings.get(sensor_name)
    
    def get_all_readings(self) -> Dict[str, Any]:
        """Get all sensor readings"""
        return self.readings.copy()
    
    def enable_sensor(self, name: str):
        """Enable a specific sensor"""
        if name in self.sensors:
            self.sensors[name].enable()
    
    def disable_sensor(self, name: str):
        """Disable a specific sensor"""
        if name in self.sensors:
            self.sensors[name].disable()
    
    def get_nearest_obstacle_distance(self) -> Optional[float]:
        """Get minimum distance from all distance sensors"""
        min_distance = float('inf')
        found_obstacle = False
        
        for name, reading in self.readings.items():
            sensor = self.sensors[name]
            
            if isinstance(sensor, DistanceSensor) and reading is not None:
                if reading < sensor.max_range:
                    min_distance = min(min_distance, reading)
                    found_obstacle = True
            elif isinstance(sensor, LidarSensor) and reading is not None:
                for distance in reading:
                    if distance < sensor.max_range:
                        min_distance = min(min_distance, distance)
                        found_obstacle = True
        
        return min_distance if found_obstacle else None


def create_default_sensor_array() -> SensorArray:
    """Create a default sensor configuration for a robot"""
    sensors = SensorArray()
    
    # Front distance sensor
    sensors.add_sensor("front", DistanceSensor(
        Vector2D(15, 0), 0.0, max_range=150.0, noise_level=0.02
    ))
    
    # Side distance sensors
    sensors.add_sensor("left", DistanceSensor(
        Vector2D(0, 10), math.pi/2, max_range=100.0, noise_level=0.02
    ))
    
    sensors.add_sensor("right", DistanceSensor(
        Vector2D(0, -10), -math.pi/2, max_range=100.0, noise_level=0.02
    ))
    
    # Rear distance sensor
    sensors.add_sensor("rear", DistanceSensor(
        Vector2D(-15, 0), math.pi, max_range=100.0, noise_level=0.02
    ))
    
    # Proximity sensors for collision avoidance
    sensors.add_sensor("prox_front_left", ProximitySensor(
        Vector2D(12, 8), math.pi/4, detection_range=25.0
    ))
    
    sensors.add_sensor("prox_front_right", ProximitySensor(
        Vector2D(12, -8), -math.pi/4, detection_range=25.0
    ))
    
    return sensors