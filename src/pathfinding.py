"""
Path Planning Algorithms for Basic Robotics Simulator

This module provides path planning algorithms:
- A* (A-star) pathfinding
- Grid-based navigation
- Path smoothing
- Dynamic path replanning

Key concepts for AI-robotics interviews:
- Graph search algorithms
- Heuristic functions
- Grid-based representations
- Path optimization
"""

import math
import heapq
import numpy as np
from typing import List, Tuple, Optional, Set, Dict, Any
from .physics import Vector2D
from .environment import Environment


class GridCell:
    """Represents a cell in the navigation grid"""
    
    def __init__(self, x: int, y: int, is_obstacle: bool = False):
        self.x = x
        self.y = y
        self.is_obstacle = is_obstacle
        
        # A* algorithm properties
        self.g_cost = float('inf')  # Cost from start
        self.h_cost = 0.0          # Heuristic cost to goal
        self.f_cost = float('inf') # Total cost (g + h)
        self.parent = None         # Parent cell for path reconstruction
        self.in_open_set = False
        self.in_closed_set = False
    
    def reset(self):
        """Reset cell for new pathfinding"""
        self.g_cost = float('inf')
        self.h_cost = 0.0
        self.f_cost = float('inf')
        self.parent = None
        self.in_open_set = False
        self.in_closed_set = False
    
    def __lt__(self, other):
        """For heapq comparison"""
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        """Equality comparison"""
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        """Hash for use in sets"""
        return hash((self.x, self.y))


class NavigationGrid:
    """
    Grid-based representation of the environment for pathfinding
    
    Key interview topics:
    - Discretization of continuous space
    - Grid resolution trade-offs
    - Obstacle inflation for robot size
    """
    
    def __init__(self, environment: Environment, cell_size: float = 10.0, 
                 robot_radius: float = 15.0):
        self.environment = environment
        self.cell_size = cell_size
        self.robot_radius = robot_radius
        
        # Grid dimensions
        self.grid_width = int(math.ceil(environment.width / cell_size))
        self.grid_height = int(math.ceil(environment.height / cell_size))
        
        # Create grid
        self.grid: List[List[GridCell]] = []
        self._initialize_grid()
        self._mark_obstacles()
    
    def _initialize_grid(self):
        """Initialize empty grid"""
        self.grid = []
        for y in range(self.grid_height):
            row = []
            for x in range(self.grid_width):
                row.append(GridCell(x, y))
            self.grid.append(row)
    
    def _mark_obstacles(self):
        """Mark obstacle cells in the grid"""
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                world_pos = self.grid_to_world(x, y)
                
                # Check if this cell or nearby cells contain obstacles
                # Use robot radius for obstacle inflation
                if self.environment.is_collision(world_pos, self.robot_radius):
                    self.grid[y][x].is_obstacle = True
    
    def world_to_grid(self, world_pos: Vector2D) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int(world_pos.x / self.cell_size)
        grid_y = int(world_pos.y / self.cell_size)
        
        # Clamp to grid bounds
        grid_x = max(0, min(self.grid_width - 1, grid_x))
        grid_y = max(0, min(self.grid_height - 1, grid_y))
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Vector2D:
        """Convert grid coordinates to world coordinates (cell center)"""
        world_x = (grid_x + 0.5) * self.cell_size
        world_y = (grid_y + 0.5) * self.cell_size
        return Vector2D(world_x, world_y)
    
    def is_valid_cell(self, x: int, y: int) -> bool:
        """Check if grid coordinates are valid and not obstacles"""
        if (0 <= x < self.grid_width and 0 <= y < self.grid_height):
            return not self.grid[y][x].is_obstacle
        return False
    
    def get_neighbors(self, cell: GridCell) -> List[GridCell]:
        """Get valid neighboring cells (8-connected)"""
        neighbors = []
        
        # 8-connected neighbors (includes diagonals)
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
        
        for dx, dy in directions:
            new_x = cell.x + dx
            new_y = cell.y + dy
            
            if self.is_valid_cell(new_x, new_y):
                neighbors.append(self.grid[new_y][new_x])
        
        return neighbors
    
    def get_movement_cost(self, from_cell: GridCell, to_cell: GridCell) -> float:
        """Calculate movement cost between adjacent cells"""
        dx = abs(to_cell.x - from_cell.x)
        dy = abs(to_cell.y - from_cell.y)
        
        # Diagonal movement costs more
        if dx == 1 and dy == 1:
            return math.sqrt(2) * self.cell_size
        else:
            return self.cell_size
    
    def reset_grid(self):
        """Reset all cells for new pathfinding"""
        for row in self.grid:
            for cell in row:
                cell.reset()


class AStarPathfinder:
    """
    A* pathfinding algorithm implementation
    
    Key interview topics:
    - Graph search algorithms
    - Admissible heuristics
    - Open/closed sets
    - Path reconstruction
    """
    
    def __init__(self, grid: NavigationGrid):
        self.grid = grid
    
    def find_path(self, start: Vector2D, goal: Vector2D) -> Optional[List[Vector2D]]:
        """
        Find path from start to goal using A* algorithm
        
        Returns:
            List of world coordinates forming the path, or None if no path found
        """
        # Reset grid for new search
        self.grid.reset_grid()
        
        # Convert world coordinates to grid coordinates
        start_grid = self.grid.world_to_grid(start)
        goal_grid = self.grid.world_to_grid(goal)
        
        # Get start and goal cells
        start_cell = self.grid.grid[start_grid[1]][start_grid[0]]
        goal_cell = self.grid.grid[goal_grid[1]][goal_grid[0]]
        
        # Check if start and goal are valid
        if start_cell.is_obstacle or goal_cell.is_obstacle:
            return None
        
        # Initialize start cell
        start_cell.g_cost = 0
        start_cell.h_cost = self._heuristic(start_cell, goal_cell)
        start_cell.f_cost = start_cell.g_cost + start_cell.h_cost
        
        # Open set (priority queue) and closed set
        open_set = [start_cell]
        heapq.heapify(open_set)
        start_cell.in_open_set = True
        
        # A* main loop
        while open_set:
            # Get cell with lowest f_cost
            current_cell = heapq.heappop(open_set)
            current_cell.in_open_set = False
            current_cell.in_closed_set = True
            
            # Check if we reached the goal
            if current_cell == goal_cell:
                return self._reconstruct_path(start_cell, goal_cell)
            
            # Examine neighbors
            for neighbor in self.grid.get_neighbors(current_cell):
                if neighbor.in_closed_set:
                    continue
                
                # Calculate tentative g_cost
                movement_cost = self.grid.get_movement_cost(current_cell, neighbor)
                tentative_g_cost = current_cell.g_cost + movement_cost
                
                # If this path to neighbor is better than previous
                if tentative_g_cost < neighbor.g_cost:
                    neighbor.parent = current_cell
                    neighbor.g_cost = tentative_g_cost
                    neighbor.h_cost = self._heuristic(neighbor, goal_cell)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    
                    if not neighbor.in_open_set:
                        heapq.heappush(open_set, neighbor)
                        neighbor.in_open_set = True
        
        # No path found
        return None
    
    def _heuristic(self, cell_a: GridCell, cell_b: GridCell) -> float:
        """
        Heuristic function for A* (Manhattan distance with diagonal movement)
        
        Interview topic: Different heuristic functions and their properties
        """
        dx = abs(cell_a.x - cell_b.x)
        dy = abs(cell_a.y - cell_b.y)
        
        # Octile distance (allows diagonal movement)
        if dx > dy:
            return (dx - dy) * self.grid.cell_size + dy * math.sqrt(2) * self.grid.cell_size
        else:
            return (dy - dx) * self.grid.cell_size + dx * math.sqrt(2) * self.grid.cell_size
    
    def _reconstruct_path(self, start_cell: GridCell, goal_cell: GridCell) -> List[Vector2D]:
        """Reconstruct path from goal to start using parent pointers"""
        path = []
        current = goal_cell
        
        while current is not None:
            world_pos = self.grid.grid_to_world(current.x, current.y)
            path.append(world_pos)
            current = current.parent
        
        # Reverse to get path from start to goal
        path.reverse()
        return path


class PathSmoother:
    """
    Smooths paths generated by grid-based planners
    
    Key concepts:
    - Path optimization
    - Line-of-sight checks
    - Trajectory smoothing
    """
    
    def __init__(self, environment: Environment, robot_radius: float = 15.0):
        self.environment = environment
        self.robot_radius = robot_radius
    
    def smooth_path(self, path: List[Vector2D], max_iterations: int = 10) -> List[Vector2D]:
        """
        Smooth path by removing unnecessary waypoints
        
        Uses line-of-sight optimization
        """
        if len(path) <= 2:
            return path
        
        smoothed_path = [path[0]]  # Start with first point
        
        for _ in range(max_iterations):
            improved = False
            i = 0
            
            while i < len(smoothed_path) - 1:
                # Try to connect current point to points further ahead
                for j in range(len(smoothed_path) - 1, i + 1, -1):
                    if self._has_line_of_sight(smoothed_path[i], smoothed_path[j]):
                        # Remove intermediate points
                        smoothed_path = smoothed_path[:i+1] + smoothed_path[j:]
                        improved = True
                        break
                i += 1
            
            if not improved:
                break
        
        return smoothed_path
    
    def _has_line_of_sight(self, start: Vector2D, end: Vector2D) -> bool:
        """Check if there's a clear line of sight between two points"""
        # Sample points along the line
        distance = (end - start).magnitude()
        num_samples = int(distance / 5.0) + 1  # Sample every 5 units
        
        for i in range(num_samples + 1):
            t = i / num_samples if num_samples > 0 else 0
            sample_point = start + (end - start) * t
            
            if self.environment.is_collision(sample_point, self.robot_radius):
                return False
        
        return True


class DynamicPathPlanner:
    """
    Dynamic path planner that can replan when obstacles change
    
    Key concepts:
    - Real-time replanning
    - Incremental search
    - Moving target adaptation
    """
    
    def __init__(self, environment: Environment, cell_size: float = 10.0,
                 robot_radius: float = 15.0):
        self.environment = environment
        self.grid = NavigationGrid(environment, cell_size, robot_radius)
        self.pathfinder = AStarPathfinder(self.grid)
        self.smoother = PathSmoother(environment, robot_radius)
        
        # Current path state
        self.current_path: Optional[List[Vector2D]] = None
        self.current_goal: Optional[Vector2D] = None
        self.path_index = 0
        
        # Replanning parameters
        self.replan_distance_threshold = 50.0  # Replan if goal moves this much
        self.path_check_interval = 30  # Check path validity every N updates
        self.update_counter = 0
    
    def plan_path(self, start: Vector2D, goal: Vector2D) -> Optional[List[Vector2D]]:
        """Plan initial path from start to goal"""
        # Update grid with current environment state
        self.grid._mark_obstacles()
        
        # Find path using A*
        raw_path = self.pathfinder.find_path(start, goal)
        
        if raw_path:
            # Smooth the path
            smoothed_path = self.smoother.smooth_path(raw_path)
            
            # Store current state
            self.current_path = smoothed_path
            self.current_goal = goal
            self.path_index = 0
            
            return smoothed_path
        
        return None
    
    def update_path(self, current_position: Vector2D, 
                   new_goal: Optional[Vector2D] = None) -> Optional[List[Vector2D]]:
        """Update path based on current position and optionally new goal"""
        self.update_counter += 1
        
        # Check if goal has changed significantly
        if new_goal and self.current_goal:
            goal_distance = (new_goal - self.current_goal).magnitude()
            if goal_distance > self.replan_distance_threshold:
                return self.plan_path(current_position, new_goal)
        elif new_goal and not self.current_goal:
            return self.plan_path(current_position, new_goal)
        
        # Periodically check if current path is still valid
        if (self.update_counter % self.path_check_interval == 0 and 
            self.current_path and self.current_goal):
            
            if not self._is_path_valid(current_position):
                # Replan from current position
                return self.plan_path(current_position, self.current_goal)
        
        return self.current_path
    
    def _is_path_valid(self, current_position: Vector2D) -> bool:
        """Check if current path is still valid (no new obstacles)"""
        if not self.current_path:
            return False
        
        # Find current position on path
        current_index = self._find_closest_path_point(current_position)
        
        # Check remaining path for obstacles
        for i in range(current_index, len(self.current_path) - 1):
            start = self.current_path[i]
            end = self.current_path[i + 1]
            
            if not self.smoother._has_line_of_sight(start, end):
                return False
        
        return True
    
    def _find_closest_path_point(self, position: Vector2D) -> int:
        """Find the closest point on the current path"""
        if not self.current_path:
            return 0
        
        min_distance = float('inf')
        closest_index = 0
        
        for i, path_point in enumerate(self.current_path):
            distance = (position - path_point).magnitude()
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        return closest_index
    
    def get_next_waypoint(self, current_position: Vector2D, 
                         lookahead_distance: float = 30.0) -> Optional[Vector2D]:
        """Get next waypoint for path following"""
        if not self.current_path:
            return None
        
        # Find current position on path
        current_index = self._find_closest_path_point(current_position)
        
        # Look ahead for next waypoint
        for i in range(current_index, len(self.current_path)):
            waypoint = self.current_path[i]
            distance = (waypoint - current_position).magnitude()
            
            if distance >= lookahead_distance:
                return waypoint
        
        # Return final waypoint if we're near the end
        return self.current_path[-1] if self.current_path else None


def create_path_planner(environment: Environment, robot_radius: float = 15.0) -> DynamicPathPlanner:
    """Factory function to create a path planner with reasonable defaults"""
    return DynamicPathPlanner(environment, cell_size=8.0, robot_radius=robot_radius)