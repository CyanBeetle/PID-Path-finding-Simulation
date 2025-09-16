"""
Visualization System for Basic Robotics Simulator

This module provides real-time visualization using pygame:
- Robot rendering with orientation
- Environment and obstacle display
- Sensor visualization (range, lidar, etc.)
- Path and trajectory display
- Debug information overlay

Key concepts for demonstrations:
- Real-time rendering
- UI design for robotics applications
- Data visualization
"""

import pygame
import math
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from .physics import Vector2D
from .robot import DifferentialDriveRobot
from .environment import Environment, CircularObstacle, RectangularObstacle, Wall
from .sensors import SensorArray, DistanceSensor, LidarSensor


# Color definitions
class Colors:
    """Color constants for visualization"""
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 0)
    CYAN = (0, 255, 255)
    MAGENTA = (255, 0, 255)
    
    # Robot colors
    ROBOT_BODY = (0, 100, 200)
    ROBOT_DIRECTION = (255, 100, 0)
    
    # Environment colors
    OBSTACLE = (100, 100, 100)
    WALL = (80, 80, 80)
    BOUNDARY = (60, 60, 60)
    
    # Path colors
    PATH_HISTORY = (0, 200, 0)
    PLANNED_PATH = (200, 0, 200)
    TARGET = (255, 0, 0)
    
    # Sensor colors
    SENSOR_BEAM = (255, 255, 0, 100)  # Semi-transparent yellow
    SENSOR_HIT = (255, 0, 0)
    LIDAR_POINTS = (0, 255, 255)
    
    # UI colors
    UI_BACKGROUND = (40, 40, 40)
    UI_TEXT = (255, 255, 255)
    UI_ACCENT = (0, 150, 255)


class SimulationRenderer:
    """
    Main rendering class for the robotics simulation
    Handles all visualization aspects
    """
    
    def __init__(self, width: int = 1200, height: int = 800):
        """Initialize the renderer"""
        pygame.init()
        
        # Screen setup
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Basic Robotics Simulator")
        
        # Fonts for text rendering
        self.font_small = pygame.font.Font(None, 20)
        self.font_medium = pygame.font.Font(None, 28)
        self.font_large = pygame.font.Font(None, 36)
        
        # Rendering options
        self.show_sensors = True
        self.show_path_history = True
        self.show_planned_path = True
        self.show_debug_info = True
        self.show_lidar_points = True
        
        # Camera/viewport
        self.camera_offset = Vector2D(0, 0)
        self.zoom = 1.0
        
        # UI panel
        self.ui_panel_width = 300
        self.main_area_width = width - self.ui_panel_width
        
        # Clock for frame rate
        self.clock = pygame.time.Clock()
        self.target_fps = 60
    
    def world_to_screen(self, world_pos: Vector2D) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates"""
        screen_x = int((world_pos.x + self.camera_offset.x) * self.zoom)
        screen_y = int((world_pos.y + self.camera_offset.y) * self.zoom)
        return (screen_x, screen_y)
    
    def screen_to_world(self, screen_pos: Tuple[int, int]) -> Vector2D:
        """Convert screen coordinates to world coordinates"""
        world_x = (screen_pos[0] / self.zoom) - self.camera_offset.x
        world_y = (screen_pos[1] / self.zoom) - self.camera_offset.y
        return Vector2D(world_x, world_y)
    
    def render_frame(self, robot: DifferentialDriveRobot, environment: Environment,
                    sensor_array: SensorArray, planned_path: Optional[List[Vector2D]] = None,
                    target_position: Optional[Vector2D] = None,
                    debug_info: Optional[Dict[str, Any]] = None):
        """Render a complete frame"""
        
        # Clear screen
        self.screen.fill(Colors.BLACK)
        
        # Create main simulation area surface
        main_surface = pygame.Surface((self.main_area_width, self.height))
        main_surface.fill(Colors.BLACK)
        
        # Render simulation components on main surface
        self._render_environment(main_surface, environment)
        self._render_planned_path(main_surface, planned_path)
        self._render_robot_path_history(main_surface, robot)
        self._render_sensors(main_surface, robot, sensor_array, environment)
        self._render_robot(main_surface, robot)
        self._render_target(main_surface, target_position)
        
        # Blit main surface to screen
        self.screen.blit(main_surface, (0, 0))
        
        # Render UI panel
        self._render_ui_panel(robot, sensor_array, debug_info)
        
        # Update display
        pygame.display.flip()
        self.clock.tick(self.target_fps)
    
    def _render_robot(self, surface: pygame.Surface, robot: DifferentialDriveRobot):
        """Render the robot"""
        pos = self.world_to_screen(robot.get_position())
        angle = robot.get_orientation()
        radius = int(robot.body.radius * self.zoom)
        
        # Robot body (circle)
        pygame.draw.circle(surface, Colors.ROBOT_BODY, pos, radius)
        pygame.draw.circle(surface, Colors.WHITE, pos, radius, 2)
        
        # Direction indicator
        dir_length = radius * 1.5
        end_x = pos[0] + int(math.cos(angle) * dir_length)
        end_y = pos[1] + int(math.sin(angle) * dir_length)
        pygame.draw.line(surface, Colors.ROBOT_DIRECTION, pos, (end_x, end_y), 3)
        
        # Wheels (simplified representation)
        wheel_offset = radius * 0.7
        left_wheel_x = pos[0] + int(math.cos(angle + math.pi/2) * wheel_offset)
        left_wheel_y = pos[1] + int(math.sin(angle + math.pi/2) * wheel_offset)
        right_wheel_x = pos[0] + int(math.cos(angle - math.pi/2) * wheel_offset)
        right_wheel_y = pos[1] + int(math.sin(angle - math.pi/2) * wheel_offset)
        
        wheel_size = 6
        pygame.draw.circle(surface, Colors.BLACK, (left_wheel_x, left_wheel_y), wheel_size)
        pygame.draw.circle(surface, Colors.BLACK, (right_wheel_x, right_wheel_y), wheel_size)
    
    def _render_environment(self, surface: pygame.Surface, environment: Environment):
        """Render environment obstacles"""
        for obstacle in environment.obstacles:
            self._render_obstacle(surface, obstacle)
    
    def _render_obstacle(self, surface: pygame.Surface, obstacle):
        """Render a single obstacle"""
        if isinstance(obstacle, CircularObstacle):
            pos = self.world_to_screen(obstacle.position)
            radius = int(obstacle.radius * self.zoom)
            color = Colors.OBSTACLE if obstacle.is_static else Colors.RED
            pygame.draw.circle(surface, color, pos, radius)
            pygame.draw.circle(surface, Colors.WHITE, pos, radius, 1)
            
        elif isinstance(obstacle, Wall):
            start_pos = self.world_to_screen(obstacle.start)
            end_pos = self.world_to_screen(obstacle.end)
            thickness = max(1, int(obstacle.thickness * self.zoom))
            pygame.draw.line(surface, Colors.WALL, start_pos, end_pos, thickness)
            
        elif isinstance(obstacle, RectangularObstacle):
            pos = self.world_to_screen(obstacle.position)
            width = int(obstacle.width * self.zoom)
            height = int(obstacle.height * self.zoom)
            rect = pygame.Rect(pos[0] - width//2, pos[1] - height//2, width, height)
            pygame.draw.rect(surface, Colors.OBSTACLE, rect)
            pygame.draw.rect(surface, Colors.WHITE, rect, 1)
    
    def _render_sensors(self, surface: pygame.Surface, robot: DifferentialDriveRobot,
                       sensor_array: SensorArray, environment: Environment):
        """Render sensor visualizations"""
        if not self.show_sensors:
            return
        
        robot_pos = robot.get_position()
        robot_angle = robot.get_orientation()
        
        for sensor_name, sensor in sensor_array.sensors.items():
            if isinstance(sensor, DistanceSensor):
                self._render_distance_sensor(surface, sensor, robot_pos, robot_angle)
            elif isinstance(sensor, LidarSensor):
                self._render_lidar_sensor(surface, sensor, robot_pos, robot_angle)
    
    def _render_distance_sensor(self, surface: pygame.Surface, sensor: DistanceSensor,
                               robot_pos: Vector2D, robot_angle: float):
        """Render distance sensor beam and reading"""
        # Get sensor absolute position and orientation
        abs_position = sensor._get_absolute_position(robot_pos, robot_angle)
        abs_orientation = robot_angle + sensor.orientation
        
        # Create sensor beam polygon (cone shape)
        beam_points = []
        beam_points.append(self.world_to_screen(abs_position))
        
        # Add cone edges
        num_points = 8
        for i in range(num_points + 1):
            angle = abs_orientation - sensor.beam_width/2 + (i * sensor.beam_width / num_points)
            range_to_use = sensor.last_reading if sensor.last_reading else sensor.max_range
            end_point = abs_position + Vector2D(
                math.cos(angle) * range_to_use,
                math.sin(angle) * range_to_use
            )
            beam_points.append(self.world_to_screen(end_point))
        
        # Draw semi-transparent sensor beam
        if len(beam_points) > 2:
            # Create surface for alpha blending
            beam_surface = pygame.Surface((surface.get_width(), surface.get_height()), pygame.SRCALPHA)
            pygame.draw.polygon(beam_surface, Colors.SENSOR_BEAM, beam_points)
            surface.blit(beam_surface, (0, 0))
        
        # Draw sensor reading point if obstacle detected
        if sensor.last_reading and sensor.last_reading < sensor.max_range:
            hit_point = abs_position + Vector2D(
                math.cos(abs_orientation) * sensor.last_reading,
                math.sin(abs_orientation) * sensor.last_reading
            )
            hit_screen_pos = self.world_to_screen(hit_point)
            pygame.draw.circle(surface, Colors.SENSOR_HIT, hit_screen_pos, 4)
    
    def _render_lidar_sensor(self, surface: pygame.Surface, sensor: LidarSensor,
                           robot_pos: Vector2D, robot_angle: float):
        """Render lidar points"""
        if not self.show_lidar_points or not sensor.last_reading:
            return
        
        # Get point cloud
        points = sensor.get_points_cloud(robot_pos, robot_angle)
        
        # Draw lidar points
        for point in points:
            screen_pos = self.world_to_screen(point)
            if (0 <= screen_pos[0] < self.main_area_width and 
                0 <= screen_pos[1] < self.height):
                pygame.draw.circle(surface, Colors.LIDAR_POINTS, screen_pos, 2)
    
    def _render_robot_path_history(self, surface: pygame.Surface, robot: DifferentialDriveRobot):
        """Render robot's path history"""
        if not self.show_path_history or len(robot.path_history) < 2:
            return
        
        # Convert path to screen coordinates
        screen_points = []
        for point in robot.path_history:
            screen_pos = self.world_to_screen(point)
            if (0 <= screen_pos[0] < self.main_area_width and 
                0 <= screen_pos[1] < self.height):
                screen_points.append(screen_pos)
        
        # Draw path line
        if len(screen_points) > 1:
            pygame.draw.lines(surface, Colors.PATH_HISTORY, False, screen_points, 2)
    
    def _render_planned_path(self, surface: pygame.Surface, planned_path: Optional[List[Vector2D]]):
        """Render planned path"""
        if not self.show_planned_path or not planned_path or len(planned_path) < 2:
            return
        
        # Convert path to screen coordinates
        screen_points = []
        for point in planned_path:
            screen_pos = self.world_to_screen(point)
            screen_points.append(screen_pos)
        
        # Draw planned path
        if len(screen_points) > 1:
            pygame.draw.lines(surface, Colors.PLANNED_PATH, False, screen_points, 3)
        
        # Draw waypoints
        for point in screen_points:
            pygame.draw.circle(surface, Colors.PLANNED_PATH, point, 4)
    
    def _render_target(self, surface: pygame.Surface, target_position: Optional[Vector2D]):
        """Render target position"""
        if target_position is None:
            return
        
        target_screen = self.world_to_screen(target_position)
        
        # Draw target as crosshairs
        size = 15
        pygame.draw.line(surface, Colors.TARGET, 
                        (target_screen[0] - size, target_screen[1]),
                        (target_screen[0] + size, target_screen[1]), 3)
        pygame.draw.line(surface, Colors.TARGET,
                        (target_screen[0], target_screen[1] - size),
                        (target_screen[0], target_screen[1] + size), 3)
        
        # Draw target circle
        pygame.draw.circle(surface, Colors.TARGET, target_screen, size, 2)
    
    def _render_ui_panel(self, robot: DifferentialDriveRobot, sensor_array: SensorArray,
                        debug_info: Optional[Dict[str, Any]]):
        """Render UI information panel"""
        # Panel background
        panel_rect = pygame.Rect(self.main_area_width, 0, self.ui_panel_width, self.height)
        pygame.draw.rect(self.screen, Colors.UI_BACKGROUND, panel_rect)
        
        y_offset = 20
        line_height = 25
        
        # Title
        title_text = self.font_large.render("Robot Status", True, Colors.UI_ACCENT)
        self.screen.blit(title_text, (self.main_area_width + 10, y_offset))
        y_offset += 40
        
        # Robot information
        pos = robot.get_position()
        angle_deg = math.degrees(robot.get_orientation())
        
        info_lines = [
            f"Position: ({pos.x:.1f}, {pos.y:.1f})",
            f"Orientation: {angle_deg:.1f}°",
            f"Distance Traveled: {robot.distance_traveled:.1f}",
            "",
            "Wheel Speeds:",
            f"  Left: {robot.left_wheel_speed:.1f} rad/s",
            f"  Right: {robot.right_wheel_speed:.1f} rad/s",
            ""
        ]
        
        # Sensor readings
        info_lines.append("Sensor Readings:")
        readings = sensor_array.get_all_readings()
        for sensor_name, reading in readings.items():
            if isinstance(reading, (int, float)):
                info_lines.append(f"  {sensor_name}: {reading:.1f}")
            elif isinstance(reading, bool):
                info_lines.append(f"  {sensor_name}: {reading}")
        
        info_lines.append("")
        
        # Debug information
        if debug_info:
            info_lines.append("Debug Info:")
            for key, value in debug_info.items():
                if isinstance(value, (int, float)):
                    info_lines.append(f"  {key}: {value:.2f}")
                else:
                    info_lines.append(f"  {key}: {value}")
        
        # Render info lines
        for line in info_lines:
            if line:  # Skip empty lines
                text_surface = self.font_small.render(line, True, Colors.UI_TEXT)
                self.screen.blit(text_surface, (self.main_area_width + 10, y_offset))
            y_offset += line_height
        
        # Control instructions
        y_offset += 20
        instructions = [
            "Controls:",
            "Arrow Keys - Manual Control",
            "Space - Stop Robot",
            "R - Reset Path",
            "S - Toggle Sensors",
            "P - Toggle Path History",
            "L - Toggle Lidar Points",
            "Click - Set Target"
        ]
        
        for instruction in instructions:
            text_surface = self.font_small.render(instruction, True, Colors.UI_TEXT)
            self.screen.blit(text_surface, (self.main_area_width + 10, y_offset))
            y_offset += line_height
    
    def handle_mouse_click(self, mouse_pos: Tuple[int, int]) -> Optional[Vector2D]:
        """Handle mouse click and return world position if in main area"""
        if mouse_pos[0] < self.main_area_width:
            return self.screen_to_world(mouse_pos)
        return None
    
    def toggle_sensor_display(self):
        """Toggle sensor visualization"""
        self.show_sensors = not self.show_sensors
    
    def toggle_path_display(self):
        """Toggle path history display"""
        self.show_path_history = not self.show_path_history
    
    def toggle_lidar_display(self):
        """Toggle lidar points display"""
        self.show_lidar_points = not self.show_lidar_points
    
    def get_fps(self) -> float:
        """Get current FPS"""
        return self.clock.get_fps()
    
    def cleanup(self):
        """Cleanup pygame resources"""
        pygame.quit()