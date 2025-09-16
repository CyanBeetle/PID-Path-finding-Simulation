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


def main():
    """Run basic simulation demo"""
    print("Basic Robotics Simulator - Enhanced Demo")
    print("Controls:")
    print("  Arrow Keys: Manual robot control (overrides autonomous)")
    print("  Left Click: Set autonomous navigation target")
    print("  Space: Emergency brake (immediate stop) and switch to manual mode")
    print("  S: Toggle sensor display")
    print("  P: Toggle path history")
    print("  R: Reset path history")
    print("  G: Generate new random map")
    print("  1: Generate obstacle course map")
    print("  2: Generate maze map")
    print("  3: Generate open field map")
    print("  4: Generate random map with seed")
    print("  ESC: Exit simulation")
    print()
    print("Features:")
    print("  - Click anywhere to make robot navigate there autonomously")
    print("  - Press any arrow key to take manual control")
    print("  - A* pathfinding provides intelligent obstacle avoidance")
    print()
    
    # Create simulation components
    physics_world = PhysicsWorld()
    environment = EnvironmentGenerator.create_random_environment(800, 600)  # Start with random environment
    
    # Create robot at center of environment
    start_position = Vector2D(400, 300)
    robot = create_robot("simple", position=start_position)
    physics_world.add_body(robot.body)
    
    # Add obstacles from environment to physics world
    for obstacle in environment.obstacles:
        if hasattr(obstacle, 'body') and obstacle.body:
            physics_world.add_body(obstacle.body)
    
    # Create sensor array
    sensor_array = create_default_sensor_array()
    
    # Create controllers
    manual_controller = ManualController(robot)
    autonomous_controller = PrecisionMovementController(robot, sensor_array, environment)
    
    # Control mode state
    control_mode = "manual"  # "manual" or "autonomous"
    target_position = None
    manual_override_timer = 0.0  # Timer to prevent immediate mode switching
    
    # Create renderer
    renderer = SimulationRenderer(1200, 800)
    
    def regenerate_environment(env_type: str, **kwargs):
        """Regenerate the environment with a new layout"""
        nonlocal environment, autonomous_controller, control_mode
        
        # Clear physics world of old obstacles
        for obstacle in environment.obstacles:
            if hasattr(obstacle, 'body') and obstacle.body:
                physics_world.remove_body(obstacle.body)
        
        # Generate new environment
        environment.regenerate_obstacles(env_type, **kwargs)
        
        # Add new obstacles to physics world
        for obstacle in environment.obstacles:
            if hasattr(obstacle, 'body') and obstacle.body:
                physics_world.add_body(obstacle.body)
        
        # Reset robot to center and stop any current navigation
        robot.body.position = Vector2D(400, 300)
        robot.body.velocity = Vector2D(0, 0)
        robot.body.angular_velocity = 0.0
        robot.reset_path()
        
        # Reinitialize autonomous controller with new environment
        autonomous_controller = PrecisionMovementController(robot, sensor_array, environment)
        
        # Switch to manual mode
        control_mode = "manual"
        autonomous_controller.disable()
        manual_controller.enable()
        
        print(f"Generated new {env_type} environment!")
    
    # Simulation loop
    running = True
    dt = 1.0 / 60.0  # 60 FPS
    
    # Input state
    keys_pressed = {
        pygame.K_UP: False,
        pygame.K_DOWN: False,
        pygame.K_LEFT: False,
        pygame.K_RIGHT: False
    }
    
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in keys_pressed:
                    keys_pressed[event.key] = True
                elif event.key == pygame.K_SPACE:
                    robot.emergency_brake()  # Use emergency brake instead of simple stop
                    control_mode = "manual"
                    autonomous_controller.disable()
                    manual_controller.enable()
                elif event.key == pygame.K_s:
                    renderer.toggle_sensor_display()
                elif event.key == pygame.K_p:
                    renderer.toggle_path_display()
                elif event.key == pygame.K_r:
                    robot.reset_path()
                elif event.key == pygame.K_g:
                    # Generate new random map
                    regenerate_environment("random")
                elif event.key == pygame.K_1:
                    # Generate obstacle course map
                    regenerate_environment("obstacle_course")
                elif event.key == pygame.K_2:
                    # Generate maze map
                    regenerate_environment("maze")
                elif event.key == pygame.K_3:
                    # Generate open field map
                    regenerate_environment("open_field")
                elif event.key == pygame.K_4:
                    # Generate random map with specific seed for reproducibility
                    import time
                    seed = int(time.time()) % 10000
                    regenerate_environment("random", seed=seed)
                    print(f"Generated random map with seed: {seed}")
            
            elif event.type == pygame.KEYUP:
                if event.key in keys_pressed:
                    keys_pressed[event.key] = False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    mouse_pos = pygame.mouse.get_pos()
                    clicked_position = renderer.handle_mouse_click(mouse_pos)
                    if clicked_position:
                        # Switch to autonomous mode
                        control_mode = "autonomous"
                        target_position = clicked_position
                        autonomous_controller.set_target(target_position)
                        autonomous_controller.enable()
                        manual_controller.disable()
                        manual_override_timer = 0.0
        
        # Process manual control input
        linear_command = 0.0
        angular_command = 0.0
        
        if keys_pressed[pygame.K_UP]:
            linear_command += 1.0
        if keys_pressed[pygame.K_DOWN]:
            linear_command -= 1.0
        if keys_pressed[pygame.K_LEFT]:
            angular_command += 1.0
        if keys_pressed[pygame.K_RIGHT]:
            angular_command -= 1.0
        
        # Check for manual override
        manual_input_detected = abs(linear_command) > 0 or abs(angular_command) > 0
        
        # Update manual override timer
        if manual_override_timer > 0:
            manual_override_timer -= dt
        
        # Control mode switching logic
        if manual_input_detected and control_mode == "autonomous" and manual_override_timer <= 0:
            # Switch to manual mode when input is detected
            control_mode = "manual"
            autonomous_controller.disable()
            manual_controller.enable()
            manual_override_timer = 0.5  # Prevent immediate switching back for 0.5 seconds
        
        # Update appropriate controller
        if control_mode == "manual":
            manual_controller.set_commands(linear_command, angular_command)
            manual_controller.update(dt)
        elif control_mode == "autonomous":
            autonomous_controller.update(dt)
            # Check if autonomous controller reached target
            if autonomous_controller.is_at_target():
                control_mode = "manual"
                autonomous_controller.disable()
                manual_controller.enable()
        # Update simulation
        sensor_array.update_all(robot.get_position(), robot.get_orientation(), environment)
        robot.update(dt, physics_world)
        physics_world.step(dt)
        # Handle collisions with environment walls
        physics_world.handle_environment_collisions(environment)
        
        # Prepare debug info
        robot_pos = robot.get_position()
        robot_velocity = robot.body.velocity
        linear_speed = robot_velocity.magnitude()
        angular_speed = abs(robot.body.angular_velocity)
        colliding_obstacles = environment.get_colliding_obstacles(robot_pos, robot.body.radius)
        
        debug_info = {
            "Control Mode": control_mode.title(),
            "Linear Command": f"{linear_command:.2f}",
            "Angular Command": f"{angular_command:.2f}",
            "Robot Position": f"({robot_pos.x:.1f}, {robot_pos.y:.1f})",
            "Current Velocity": f"{linear_speed:.2f} m/s",
            "Angular Velocity": f"{angular_speed:.3f} rad/s",
            "Robot Status": "STOPPED" if robot.is_stopped() else "MOVING",
            "Collisions": f"{len(colliding_obstacles)}",
            "FPS": f"{renderer.get_fps():.1f}"
        }
        
        # Add target information if in autonomous mode
        if control_mode == "autonomous" and target_position:
            distance_to_target = robot.distance_to(target_position)
            debug_info["Target"] = f"({target_position.x:.1f}, {target_position.y:.1f})"
            debug_info["Distance to Target"] = f"{distance_to_target:.1f}"
            
            # Add controller-specific debug info
            controller_debug = autonomous_controller.get_debug_info()
            debug_info.update(controller_debug)
        
        # Render frame
        renderer.render_frame(
            robot=robot,
            environment=environment,
            sensor_array=sensor_array,
            target_position=target_position if control_mode == "autonomous" else None,
            debug_info=debug_info
        )
    
    # Cleanup
    renderer.cleanup()
    print("Simulation ended.")


if __name__ == "__main__":
    main()