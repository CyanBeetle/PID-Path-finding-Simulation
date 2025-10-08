"""
Basic Robotics Simulation - Simple Demo

This example demonstrates basic robot movement and manual control.
Perfect for getting started and understanding the core concepts.

Run this to see:
- Robot physics and movement
- Manual control with arrow keys
- Sensor visualization
- Basic environment interaction
"""

import pygame
import sys
import os

# Add parent directory to Python path to make src a package
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from src.physics import Vector2D, PhysicsWorld
from src.robot import create_robot, ManualController
from src.controllers import PrecisionMovementController
from src.environment import EnvironmentGenerator
from src.sensors import create_default_sensor_array
from src.visualization import SimulationRenderer


class Simulation:
    """
    Encapsulates the entire simulation, managing its state, components,
    and the main loop.
    """
    def __init__(self):
        self._print_instructions()
        
        # Core components
        self.physics_world = PhysicsWorld()
        self.environment = EnvironmentGenerator.create_random_environment(800, 600)
        
        # Robot and its controllers
        self.robot = create_robot("simple", position=Vector2D(400, 300))
        self.sensor_array = create_default_sensor_array()
        self.manual_controller = ManualController(self.robot)
        self.autonomous_controller = PrecisionMovementController(self.robot, self.sensor_array, self.environment)
        
        # Initial state
        self.control_mode = "manual"
        self.target_position = None
        self.manual_override_timer = 0.0
        
        # Add bodies to physics world
        self.physics_world.add_body(self.robot.body)
        for obstacle in self.environment.obstacles:
            if hasattr(obstacle, 'body') and obstacle.body:
                self.physics_world.add_body(obstacle.body)

        # Rendering
        self.renderer = SimulationRenderer(1200, 800)
        
        # Simulation loop control
        self.running = True
        self.dt = 1.0 / 60.0
        
        # Input handling
        self.keys_pressed = {k: False for k in [pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT]}
        self._setup_key_mappings()

    def _print_instructions(self):
        """Prints the simulation controls and features to the console."""
        print("Basic Robotics Simulator - Refactored Demo")
        print("Controls:")
        print("  Arrow Keys: Manual robot control")
        print("  Left Click: Set autonomous navigation target")
        print("  Space: Emergency brake and switch to manual mode")
        print("  S/P/R: Toggle Sensor Display, Path History, Reset Path")
        print("  G, 1-4: Generate different environment maps")
        print("  ESC: Exit simulation")

    def _setup_key_mappings(self):
        """Set up mappings for environment generation keys."""
        self.env_key_map = {
            pygame.K_g: ("random", {}),
            pygame.K_1: ("obstacle_course", {}),
            pygame.K_2: ("maze", {}),
            pygame.K_3: ("open_field", {}),
        }

    def run(self):
        """Starts and runs the main simulation loop."""
        while self.running:
            self._handle_input()
            self._update_simulation()
            self._render_frame()
        
        self.renderer.cleanup()
        print("Simulation ended.")

    def _handle_input(self):
        """Processes all user input from Pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                self.running = False
            
            # Handle key presses
            if event.type == pygame.KEYDOWN:
                if event.key in self.keys_pressed:
                    self.keys_pressed[event.key] = True
                elif event.key == pygame.K_SPACE:
                    self._activate_manual_mode(emergency_brake=True)
                elif event.key == pygame.K_s:
                    self.renderer.toggle_sensor_display()
                elif event.key == pygame.K_p:
                    self.renderer.toggle_path_display()
                elif event.key == pygame.K_r:
                    self.robot.reset_path()
                elif event.key in self.env_key_map:
                    env_type, kwargs = self.env_key_map[event.key]
                    self._regenerate_environment(env_type, **kwargs)
                elif event.key == pygame.K_4:
                    import time
                    seed = int(time.time()) % 10000
                    self._regenerate_environment("random", seed=seed)
                    print(f"Generated random map with seed: {seed}")

            # Handle key releases
            if event.type == pygame.KEYUP:
                if event.key in self.keys_pressed:
                    self.keys_pressed[event.key] = False
            
            # Handle mouse clicks
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                self._handle_mouse_click(pygame.mouse.get_pos())

    def _handle_mouse_click(self, mouse_pos):
        """Handles logic for setting an autonomous target."""
        clicked_pos = self.renderer.handle_mouse_click(mouse_pos)
        if clicked_pos:
            self.control_mode = "autonomous"
            self.target_position = clicked_pos
            self.autonomous_controller.set_target(self.target_position)
            self.autonomous_controller.enable()
            self.manual_controller.disable()
            self.manual_override_timer = 0.0

    def _update_simulation(self):
        """Updates all simulation components for a single frame."""
        linear_cmd, angular_cmd = self._get_manual_commands()
        self._update_control_mode(linear_cmd, angular_cmd)
        
        # Update the active controller
        if self.control_mode == "manual":
            self.manual_controller.set_commands(linear_cmd, angular_cmd)
            self.manual_controller.update(self.dt)
        else: # autonomous
            self.autonomous_controller.update(self.dt)
            if self.autonomous_controller.is_at_target():
                self._activate_manual_mode()

        # Update physics and sensors
        self.sensor_array.update_all(self.robot.get_position(), self.robot.get_orientation(), self.environment)
        self.robot.update(self.dt, self.physics_world)
        self.physics_world.step(self.dt)
        self.physics_world.handle_environment_collisions(self.environment)
        
        if self.manual_override_timer > 0:
            self.manual_override_timer -= self.dt

    def _get_manual_commands(self) -> (float, float):
        """Returns linear and angular commands based on keys pressed."""
        linear = 0.0
        angular = 0.0
        if self.keys_pressed[pygame.K_UP]: linear += 1.0
        if self.keys_pressed[pygame.K_DOWN]: linear -= 1.0
        if self.keys_pressed[pygame.K_LEFT]: angular += 1.0
        if self.keys_pressed[pygame.K_RIGHT]: angular -= 1.0
        return linear, angular

    def _update_control_mode(self, linear_cmd, angular_cmd):
        """Switches to manual control if input is detected."""
        if (abs(linear_cmd) > 0 or abs(angular_cmd) > 0) and \
           self.control_mode == "autonomous" and self.manual_override_timer <= 0:
            self._activate_manual_mode()
            self.manual_override_timer = 0.5  # Prevent rapid switching

    def _activate_manual_mode(self, emergency_brake=False):
        """Switches the robot to manual control mode."""
        if emergency_brake:
            self.robot.emergency_brake()
        self.control_mode = "manual"
        self.autonomous_controller.disable()
        self.manual_controller.enable()

    def _render_frame(self):
        """Prepares debug info and renders the simulation frame."""
        debug_info = self._prepare_debug_info()
        self.renderer.render_frame(
            robot=self.robot,
            environment=self.environment,
            sensor_array=self.sensor_array,
            target_position=self.target_position if self.control_mode == "autonomous" else None,
            debug_info=debug_info
        )

    def _prepare_debug_info(self) -> dict:
        """Gathers and formats debug information for rendering."""
        robot_pos = self.robot.get_position()
        linear_speed = self.robot.body.velocity.magnitude()
        angular_speed = abs(self.robot.body.angular_velocity)
        
        info = {
            "Control Mode": self.control_mode.title(),
            "Position": f"({robot_pos.x:.1f}, {robot_pos.y:.1f})",
            "Velocity": f"{linear_speed:.2f} m/s",
            "Angular Velocity": f"{angular_speed:.3f} rad/s",
            "FPS": f"{self.renderer.get_fps():.1f}"
        }
        
        if self.control_mode == "autonomous" and self.target_position:
            info.update(self.autonomous_controller.get_debug_info())
        
        return info

    def _regenerate_environment(self, env_type: str, **kwargs):
        """Clears the old environment and generates a new one."""
        for obstacle in self.environment.obstacles:
            if hasattr(obstacle, 'body') and obstacle.body:
                self.physics_world.remove_body(obstacle.body)
        
        self.environment.regenerate_obstacles(env_type, **kwargs)
        
        for obstacle in self.environment.obstacles:
            if hasattr(obstacle, 'body') and obstacle.body:
                self.physics_world.add_body(obstacle.body)
        
        self.robot.body.position = Vector2D(400, 300)
        self.robot.reset_path()
        
        self.autonomous_controller = PrecisionMovementController(self.robot, self.sensor_array, self.environment)
        self._activate_manual_mode(emergency_brake=True)
        
        print(f"Generated new '{env_type}' environment!")

def main():
    """Initializes and runs the simulation."""
    simulation = Simulation()
    simulation.run()


if __name__ == "__main__":
    main()