import cv2
import numpy as np
import math
from heapq import heappop, heappush

obstacle_positions = [(1, 1), (0, 9), (4, 1), (4, 7), (8, 3), (8, 6)]
MOVEMENT_DIRECTIONS = [(1, 0), (-1, 0), (0, 1), (0, -1)]

class DrawGrid:
    def __init__(self, map_size=470, grid_size=47, obstacle_size=(47, 47)):
        self.map_size = map_size
        self.grid_size = grid_size
        self.obstacle_size = obstacle_size
        self.grid_rows = int(map_size / grid_size)
        self.grid_cols = int(map_size / grid_size)

    def draw_grid(self, image):
        color = (100, 100, 100)  
        for x in range(0, image.shape[1], self.grid_size):
            cv2.line(image, (x, 0), (x, image.shape[0]), color, 1)
        for y in range(0, image.shape[0], self.grid_size):
            cv2.line(image, (0, y), (image.shape[1], y), color, 1)

    def add_obstacles(self, image, obstacles):
        for obstacle in obstacles:
            row, col = obstacle
            x1 = col * self.grid_size
            y1 = row * self.grid_size
            x2 = x1 + self.obstacle_size[1]
            y2 = y1 + self.obstacle_size[0]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 200), -1)  

    def add_robot(self, image, position, direction_degrees=0):
        row, col = position
        center_x = int((col + 0.5) * self.grid_size)
        center_y = int((row + 0.5) * self.grid_size)
        radius = int(self.grid_size / 3)
        cv2.circle(image, (center_x, center_y), radius, (0, 0, 0), -1)  

        line_length = int(radius * 0.8)
        angle_radians = math.radians(direction_degrees)
        end_x = int(center_x + line_length * math.cos(angle_radians))
        end_y = int(center_y - line_length * math.sin(angle_radians)) 
        cv2.line(image, (center_x, center_y), (end_x, end_y), (255, 255, 255), 2)

    def add_goal(self, image, position):
        row, col = position
        center_x = int((col + 0.5) * self.grid_size)
        center_y = int((row + 0.5) * self.grid_size)
        radius = int(self.grid_size / 4)
        cv2.circle(image, (center_x, center_y), radius, (0, 255, 0), -1) 

class AStarAlgo:
    def __init__(self, grid_rows, grid_cols):
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols

    def calculate_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def calculate_direction(self, current_position, next_position):
        dx = next_position[1] - current_position[1]
        #Adjust the calculation of dy to reflect the inverse growth of the y-axis
        dy = -(next_position[0] - current_position[0])
        angle_radians = math.atan2(dy, dx)
        angle_degrees = math.degrees(angle_radians)
        # Normalize angles to range from 0 to 360 degrees
        if angle_degrees < 0:
            angle_degrees += 360
        return angle_degrees

    # Check if a point is inside an obstacle
    def is_point_inside_obstacle(self, point):
        x, y = point
        for obstacle in obstacle_positions:
            x_obs, y_obs = obstacle
            if x_obs == x and y_obs == y:
                return True
        return False

    # Check if a point is valid (inside the boundaries and not inside an obstacle)
    def is_valid_point(self, point):
        x, y = point
        return 0 <= x < self.grid_rows and 0 <= y < self.grid_cols and not self.is_point_inside_obstacle(point)
    
    # Check if a move is valid
    def is_valid_move(self, current, next_point):
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
    def find_shortest_path_Astar(self, start, end):
        #global countStep
        open_list = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.calculate_distance(start, end)}
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
                return path[::-1]

            for dx, dy in MOVEMENT_DIRECTIONS:
                next_point = current[0] + dx, current[1] + dy

                if self.is_valid_point(next_point) and self.is_valid_move(current, next_point):
                    new_g_score = g_score[current] + self.calculate_distance(current, next_point)
                    if next_point not in g_score or new_g_score < g_score[next_point]:
                        came_from[next_point] = current
                        g_score[next_point] = new_g_score
                        # Calculate the heuristic score
                        h_score = self.calculate_distance(next_point, end) 
                        f_score[next_point] = new_g_score + h_score
                        heappush(open_list, (f_score[next_point], next_point))

                        # Plot visited nodes as black dotted lines
                        visitedNode+=1

        return None

def generate_arrows(image, path, grid_size):
    for i in range(len(path) - 1):
        current_cell = path[i]
        next_cell = path[i + 1]
        current_x = int((current_cell[1] + 0.5) * grid_size)
        current_y = int((current_cell[0] + 0.5) * grid_size)
        next_x = int((next_cell[1] + 0.5) * grid_size)
        next_y = int((next_cell[0] + 0.5) * grid_size)
        cv2.arrowedLine(image, (current_x, current_y), (next_x, next_y), (0, 0, 0), 2)

def main():
    grid_drawer = DrawGrid()
    a_star = AStarAlgo(grid_drawer.grid_rows, grid_drawer.grid_cols)
    map_image = np.ones((grid_drawer.map_size, grid_drawer.map_size, 3), dtype=np.uint8) * 200  # Grey color

    grid_drawer.draw_grid(map_image)    
    robot_position = (4, 5)
    goal_position = (8, 8)

    grid_drawer.add_obstacles(map_image, obstacle_positions)
    grid_drawer.add_robot(map_image, robot_position)
    grid_drawer.add_goal(map_image, goal_position)

    path = a_star.find_shortest_path_Astar(robot_position, goal_position)

    if path:
        generate_arrows(map_image, path, grid_drawer.grid_size)
    else:
        print("No path found!")
        
    cv2.imshow('Gridworld: Turtlebot3 with Obstacles', map_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

