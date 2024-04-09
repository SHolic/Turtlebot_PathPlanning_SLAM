import cv2
import numpy as np
import math

obstacle_positions = [(1, 1), (0, 9), (4, 1), (4, 7), (8, 3), (8, 6)]

class GridDrawer:
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

class AStar:
    def __init__(self, grid_rows, grid_cols):
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols

    def heuristic(self, node, goal):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def within_bounds(self, cell):
        return 0 <= cell[0] < self.grid_rows and 0 <= cell[1] < self.grid_cols

    def is_traversable(self, cell, obstacles):
        return cell not in obstacles

    def get_neighbors(self, cell):
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  
        neighbors = [(cell[0] + dy, cell[1] + dx) for dy, dx in directions]
        return [neighbor for neighbor in neighbors if self.within_bounds(neighbor)]

    def astar(self, start, goal, obstacles):
        open_set = {start}
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

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
            for neighbor in self.get_neighbors(current):
                if not self.is_traversable(neighbor, obstacles):
                    continue
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)

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
    grid_drawer = GridDrawer()
    a_star = AStar(grid_drawer.grid_rows, grid_drawer.grid_cols)
    map_image = np.ones((grid_drawer.map_size, grid_drawer.map_size, 3), dtype=np.uint8) * 200  # Grey color

    grid_drawer.draw_grid(map_image)    
    robot_position = (4, 5)
    goal_position = (8, 8)

    grid_drawer.add_obstacles(map_image, obstacle_positions)
    grid_drawer.add_robot(map_image, robot_position)
    grid_drawer.add_goal(map_image, goal_position)

    path = a_star.astar(robot_position, goal_position, obstacle_positions)

    if path:
        generate_arrows(map_image, path, grid_drawer.grid_size)
    else:
        print("No path found!")
        
    cv2.imshow('Gridworld: Turtlebot3 with Obstacles', map_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

