import rclpy
from rclpy.node import Node
from turtlebot3_msgs.srv import PlanPath
from turtlebot3_msgs.msg import Point2D
from heapq import heappush, heappop
import numpy as np

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        self.service = self.create_service(
            PlanPath, 
            '/plan_path', 
            self.plan_path_callback)

        self.GRID_ROWS = 10
        self.GRID_COLS = 10
        self.MOVEMENT_DIRECTIONS = [(1, 0), (-1, 0), (0, 1), (0, -1)] # Diagonals: (1, 1), (-1, -1), (1, -1), (-1, 1)
        self.SEARCH_STEP = 1

    # Calculate the Euclidean distance between two points 
    def calculate_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Check if a point is inside an obstacle
    def is_point_inside_obstacle(self, point, obstacles):
        return point in obstacles

    # Check if a point is valid (inside the boundaries and not inside an obstacle)
    def is_valid_point(self, point, obstacles):
        x, y = point
        return 0 <= x < self.GRID_ROWS and 0 <= y < self.GRID_COLS and not self.is_point_inside_obstacle(point, obstacles)
        
    # Check if a move is valid
    def is_valid_move(self, current, next_point, obstacles):
        x1, y1 = current
        x2, y2 = next_point
        dx = x2 - x1
        dy = y2 - y1

        if dx != 0 and dy != 0:
            # Check if the move is diagonal
            # Prevent diagonal moves that cut corners by checking if the adjacent cells are obstacles
            if not self.is_valid_point((x1 + dx, y1), obstacles) or not self.is_valid_point((x1, y1 + dy), obstacles):
                return False

        return True

    # Find the shortest path using the A* algorithm
    def find_shortest_path_Astar(self, start, end, obstacles):
        open_list = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.calculate_distance(start, end)}
        steps = 0
        visitedNodes = 0
        
        while open_list:
            steps += 1
            current = heappop(open_list)[1]

            if current == end:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1], steps, g_score[end], visitedNodes

            for dx, dy in self.MOVEMENT_DIRECTIONS:
                next_point = current[0] + dx * self.SEARCH_STEP, current[1] + dy * self.SEARCH_STEP

                if self.is_valid_point(next_point, obstacles) and self.is_valid_move(current, next_point, obstacles):
                    new_g_score = g_score[current] + self.calculate_distance(current, next_point)
                    if next_point not in g_score or new_g_score < g_score[next_point]:
                        came_from[next_point] = current
                        g_score[next_point] = new_g_score
                        # Calculate the heuristic score
                        h_score = self.calculate_distance(next_point, end) 
                        f_score[next_point] = new_g_score + h_score
                        heappush(open_list, (f_score[next_point], next_point))

                        visitedNodes += 1

        return None, steps, float('inf'), visitedNodes

    def plan_path_callback(self, request, response):
        # Convert the request to a format suitable for the path planning algorithm
        start_position = (request.start.x, request.start.y)
        goal_position = (request.goal.x, request.goal.y)
        obstacle_positions = [(obstacle.x, obstacle.y) for obstacle in request.obstacles]

        # Perform path planning
        path, _, _, _ = self.find_shortest_path_Astar(start_position, goal_position, obstacle_positions)

        if path:
            # Convert the path to a list of Point2D messages and fill it into the response
            for point in path:
                point_msg = Point2D()
                point_msg.x = point[0]
                point_msg.y = point[1]
                response.path.append(point_msg)  # Assuming response.path is an array of Point2D messages

        return response

def main(args=None):
    rclpy.init(args=args)
    path_planning_node = PathPlanningNode()
    rclpy.spin(path_planning_node)
    path_planning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
