import numpy as np
import math

# Constants
MAP_SIZE = 470  # in cm
GRID_SIZE = int(MAP_SIZE / 10)  # in cm
GRID_ROWS = int(MAP_SIZE / GRID_SIZE)
GRID_COLS = int(MAP_SIZE / GRID_SIZE)
OBSTACLE_SIZE = (GRID_SIZE, GRID_SIZE)  # in cm
ROBOT_RADIUS = int(GRID_SIZE / 3)  # in cm
GOAL_RADIUS = int(GRID_SIZE / 4)  # in cm
DIRECTIONS = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # Up, Down, Left, Right

# Function to calculate heuristic (Manhattan distance)
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# Function to check if a cell is within the map bounds
def within_bounds(cell):
    return 0 <= cell[0] < GRID_ROWS and 0 <= cell[1] < GRID_COLS

# Function to check if a cell is traversable (not an obstacle)
def is_traversable(cell, obstacles):
    return cell not in obstacles

# Function to find the neighbors of a given cell
def get_neighbors(cell):
    neighbors = [(cell[0] + dy, cell[1] + dx) for dy, dx in DIRECTIONS]
    return [neighbor for neighbor in neighbors if within_bounds(neighbor)]

# A* Path Planning Algorithm
def astar(start, goal, obstacles):
    open_set = {start}
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = min(open_set, key=lambda x: f_score[x])

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        open_set.remove(current)
        for neighbor in get_neighbors(current):
            if not is_traversable(neighbor, obstacles):
                continue
            tentative_g_score = g_score[current] + 1
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in open_set:
                    open_set.add(neighbor)

    return None

# Autonomous Navigation Program
def autonomous_navigation(robot_position, goal_position, obstacle_positions):
    path = astar(robot_position, goal_position, obstacle_positions)
    if path:
        print("Found path:", path)
        # Execute the path by moving the robot
        for position in path:
            print("Moving to position:", position)
            # Update robot_position
            robot_position = position
            # Perform motion to the new position (e.g., publish to robot control topic)
    else:
        print("No path found!")

# Sample usage
def main():
    # Define initial robot position, goal position, and obstacle positions
    robot_position = (4, 5)
    goal_position = (8, 8)
    obstacle_positions = [(1, 1), (0, 9), (4, 1), (4, 7), (8, 3), (8, 6)]

    # Execute autonomous navigation program
    autonomous_navigation(robot_position, goal_position, obstacle_positions)

if __name__ == "__main__":
    main()
