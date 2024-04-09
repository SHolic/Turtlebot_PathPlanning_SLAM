import cv2
import numpy as np
import math
from heapq import heappop, heappush
import time

# Constants
MAP_SIZE = 470  # in cm
GRID_SIZE = int(MAP_SIZE / 10)  # in cm
OBSTACLE_SIZE = (GRID_SIZE, GRID_SIZE)  # in cm
GRID_ROWS = int(MAP_SIZE / GRID_SIZE)
GRID_COLS = int(MAP_SIZE / GRID_SIZE)
# Define the minimum step size for searching
SEARCH_STEP = 1
# Define the possible movement directions (including diagonals)
MOVEMENT_DIRECTIONS = [(1, 0), (-1, 0), (0, 1), (0, -1)] # , (1, 1), (-1, -1), (1, -1), (-1, 1)
# Define positions based on cell indices
obstacle_positions = [(1, 1), (0, 9), (4, 1), (4, 7), (8, 3), (8, 6)]
robot_position = (4, 5)
goal_position = (8, 8)

# Function to draw grid lines on the map
def draw_grid(image):
    color = (100, 100, 100)  # Grey color
    for x in range(0, image.shape[1], GRID_SIZE):
        cv2.line(image, (x, 0), (x, image.shape[0]), color, 1)
    for y in range(0, image.shape[0], GRID_SIZE):
        cv2.line(image, (0, y), (image.shape[1], y), color, 1)

# Function to add obstacles to the map based on cell indices
def add_obstacles(image, obstacles):
    for obstacle in obstacles:
        row, col = obstacle
        x1 = col * GRID_SIZE
        y1 = row * GRID_SIZE
        x2 = x1 + OBSTACLE_SIZE[1]
        y2 = y1 + OBSTACLE_SIZE[0]
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 200), -1)  # Blue color for obstacles

# Function to add robot to the map based on cell index
def add_robot(image, position, direction_degrees=0):
    row, col = position
    center_x = int((col + 0.5) * GRID_SIZE)
    center_y = int((row + 0.5) * GRID_SIZE)
    radius = int(GRID_SIZE / 3)
    cv2.circle(image, (center_x, center_y), radius, (0, 0, 0), -1)  # Black color for robot
    # Draw a white line indicating direction
    line_length = int(radius * 0.8)
    angle_radians = math.radians(direction_degrees)
    end_x = int(center_x + line_length * math.cos(angle_radians))
    end_y = int(center_y - line_length * math.sin(angle_radians))  # Negative because the y-axis is inverted in images
    cv2.line(image, (center_x, center_y), (end_x, end_y), (255, 255, 255), 2)

# Function to add goal position to the map based on cell index
def add_goal(image, position):
    row, col = position
    center_x = int((col + 0.5) * GRID_SIZE)
    center_y = int((row + 0.5) * GRID_SIZE)
    radius = int(GRID_SIZE / 4)
    cv2.circle(image, (center_x, center_y), radius, (0, 255, 0), -1)  # Green color for goal
    
# Calculate the Euclidean distance between two points 
def calculate_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def calculate_direction(current_position, next_position):
    dx = next_position[1] - current_position[1]
    #Adjust the calculation of dy to reflect the inverse growth of the y-axis
    dy = -(next_position[0] - current_position[0])  # 添加负号以反转y轴方向
    angle_radians = math.atan2(dy, dx)
    angle_degrees = math.degrees(angle_radians)
    # Normalize angles to range from 0 to 360 degrees
    if angle_degrees < 0:
        angle_degrees += 360
    return angle_degrees

# Check if a point is inside an obstacle
def is_point_inside_obstacle(point):
    x, y = point
    for obstacle in obstacle_positions:
        x_obs, y_obs = obstacle
        if x_obs == x and y_obs == y:
            return True
    return False

# Check if a point is valid (inside the boundaries and not inside an obstacle)
def is_valid_point(point):
    x, y = point
    return 0 <= x < GRID_ROWS and 0 <= y < GRID_COLS and not is_point_inside_obstacle(point)
    
# Check if a move is valid
def is_valid_move(current, next_point):
    x1, y1 = current
    x2, y2 = next_point
    dx = x2 - x1
    dy = y2 - y1

    if dx != 0 and dy != 0:
        # Check if the move is diagonal
        # Prevent diagonal moves that cut corners by checking if the adjacent cells are obstacles
        if not is_valid_point((x1 + dx, y1)) or not is_valid_point((x1, y1 + dy)):
            return False

    return True

# Find the shortest path using the A* algorithm using 
def find_shortest_path_Astar(start, end):
    #global countStep
    open_list = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: calculate_distance(start, end)}
    steps = 0
    visitedNode = 0
    
    while open_list:
        steps += 1
        current = heappop(open_list)[1]

        if current == end:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1], steps, g_score[end], visitedNode

        for dx, dy in MOVEMENT_DIRECTIONS:
            next_point = current[0] + dx * SEARCH_STEP, current[1] + dy * SEARCH_STEP

            if is_valid_point(next_point) and is_valid_move(current, next_point):
                new_g_score = g_score[current] + calculate_distance(current, next_point)
                if next_point not in g_score or new_g_score < g_score[next_point]:
                    came_from[next_point] = current
                    g_score[next_point] = new_g_score
                    # Calculate the heuristic score
                    h_score = calculate_distance(next_point, end) 
                    f_score[next_point] = new_g_score + h_score
                    heappush(open_list, (f_score[next_point], next_point))

                    # Plot visited nodes as black dotted lines
                    visitedNode+=1

    return None, steps, float('inf'), visitedNode

def update_robot_position(image, new_position, new_direction):
    global robot_position  # Use the global statement to modify global variables within the function
    # Clear the original robot position first
    cv2.circle(image, (int((robot_position[1] + 0.5) * GRID_SIZE), int((robot_position[0] + 0.5) * GRID_SIZE)), int(GRID_SIZE / 3), (200, 200, 200), -1)
    # Update robot position
    robot_position = new_position
    ## Draw new robot position
    add_robot(image, robot_position, direction_degrees=new_direction)

# Main function
def main():
    # Create a grey flooring map
    map_image = np.ones((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8) * 200  # Grey color

    # Draw grid lines
    draw_grid(map_image)

    # Add obstacles to the map
    add_obstacles(map_image, obstacle_positions)

    # Add robot to the map
    add_robot(map_image, robot_position, direction_degrees=90)

    # Add goal position to the map
    add_goal(map_image, goal_position)
    
    # Using A* to find path
    path, steps, distance, visitedNode = find_shortest_path_Astar(robot_position, goal_position)
    
    # visualize path
    if path:
        print(path)
        for i in range(len(path) - 1):
            point1 = path[i]
            point2 = path[i + 1]
            cv2.line(map_image, 
                     (point1[1] * GRID_SIZE + GRID_SIZE // 2, point1[0] * GRID_SIZE + GRID_SIZE // 2),
                     (point2[1] * GRID_SIZE + GRID_SIZE // 2, point2[0] * GRID_SIZE + GRID_SIZE // 2),
                     (255, 0, 0), 2)
            
        for i, point in enumerate(path[:-1]):  # Start traversing from the second point
            next_point = path[i + 1]
            direction_degrees = calculate_direction(point, next_point)
            
            # Change direction
            update_robot_position(map_image, point, direction_degrees)
            cv2.imshow('Gridworld: Turtlebot3 with Obstacles', map_image)
            cv2.waitKey(1)
            time.sleep(1)
            
            # Move to next position
            update_robot_position(map_image, next_point, direction_degrees)  # 确保 update_robot_position 函数不再调用 add_robot，或者调用时不传入方向参数
            cv2.imshow('Gridworld: Turtlebot3 with Obstacles', map_image)
            cv2.waitKey(1)
            time.sleep(1)
            
    # Display the map
    cv2.imshow('Gridworld: Turtlebot3 with Obstacles', map_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
