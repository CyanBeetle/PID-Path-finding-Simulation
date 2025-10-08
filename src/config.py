"""
Centralized Configuration for the Robotics Simulator

This file contains all the tunable parameters for the simulation, allowing for
easy adjustments to the robot's behavior without modifying the core logic.

By centralizing these settings, we can:
- Quickly experiment with different robot performance profiles.
- Define multiple configurations for different scenarios (e.g., speed, precision).
- Improve code readability by separating constants from implementation.
"""

from typing import Dict, Any

# --- Simulation-Wide Settings ---
SIMULATION_SETTINGS: Dict[str, Any] = {
    "time_step": 1.0 / 60.0,  # Simulation update rate (60 FPS)
    "grid_size": 20.0,        # Pathfinding grid resolution
}

# --- Robot Physics & Kinematics ---
ROBOT_PHYSICS: Dict[str, Any] = {
    "radius": 15.0,
    "mass": 10.0,
    "friction_coefficient": 0.2,
    "angular_friction_coefficient": 1.5,
    "max_linear_force": 150.0,
    "max_torque": 50.0,
}

# --- PrecisionMovementController Tuning Parameters ---
# This dictionary holds all the values for the main autonomous controller.
# Grouping them here allows for easy tuning and experimentation.
CONTROLLER_CONFIG: Dict[str, Any] = {
    # --- Target and Waypoint Management ---
    "target_tolerance": 5.0,          # Final distance to target to be considered "arrived"
    "waypoint_tolerance": 10.0,         # Distance to a waypoint to advance to the next one
    "angle_tolerance": 0.05,          # Radians, for final orientation adjustments
    "lookahead_distance": 40.0,       # How far along the path to look for the next target point

    # --- Velocity Control (Distance-based) ---
    "max_velocity": 250.0,            # Maximum speed of the robot
    "min_velocity": 15.0,             # Minimum speed when approaching a target
    "max_distance_for_full_speed": 300.0, # Distance at which robot travels at max_velocity
    "precision_distance": 50.0,       # Distance at which robot begins to slow down for precision

    # --- PID Gains (Position Control) ---
    "position_kp": 3.0,               # Proportional gain for position error
    "position_ki": 0.05,              # Integral gain for accumulated position error
    "position_kd": 0.3,               # Derivative gain for rate of change of position error
    "position_integral_clamp": 50.0,  # Max/min value for the position integral term

    # --- PID Gains (Angular Control) ---
    "angle_kp": 5.0,                  # Proportional gain for angle error
    "angle_ki": 0.1,                  # Integral gain for accumulated angle error
    "angle_kd": 0.2,                  # Derivative gain for rate of change of angle error
    "angle_integral_clamp": 1.0,      # Max/min value for the angle integral term

    # --- Damping and Braking ---
    "velocity_damping_base": 0.05,    # Base damping factor to counter momentum
    "velocity_damping_max": 0.5,      # Max damping factor when close to the target
    "angular_damping": 0.4,           # Damping for rotational movement
    "brake_distance": 15.0,           # Distance at which emergency braking engages
    "brake_force": 400.0,             # Force applied during emergency braking

    # --- Obstacle Avoidance ---
    "obstacle_avoidance_strength": 0.7, # How strongly the robot reacts to obstacles
    "wall_detection_distance": 30.0,    # Base distance to detect walls/obstacles
    "emergency_detection_distance": 15.0, # Closer distance for more drastic maneuvers
}