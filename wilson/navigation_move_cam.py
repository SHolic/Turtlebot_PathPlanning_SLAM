import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import numpy as np
import path_planning
from path_planning import obstacle_positions, DrawGrid, AStarAlgo, generate_arrows


class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.action = []
        self.position = (0, 0)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update_state)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge() 
        
        self.path = [(4, 5), (5, 5), (5, 6), (5, 7), (6, 7), (7, 7), (8, 7), (8, 8)]

        self.current_action_index = 0        
        self.direction = "up"
        self.path_actions = []
        # Mapping from (current_direction, desired_direction) to (action, angle)
        self.direction_changes = {
            ('up', 'right'): ('turn_right', math.pi / 2),
            ('up', 'left'): ('turn_left', math.pi / 2),
            ('down', 'right'): ('turn_left', math.pi / 2),
            ('down', 'left'): ('turn_right', math.pi / 2),
            ('left', 'up'): ('turn_right', math.pi / 2),
            ('left', 'down'): ('turn_left', math.pi / 2),
            ('right', 'up'): ('turn_left', math.pi / 2),
            ('right', 'down'): ('turn_right', math.pi / 2),
            ('up', 'down'): ('turn_right', math.pi),
            ('down', 'up'): ('turn_right', math.pi),
            ('left', 'right'): ('turn_right', math.pi),
            ('right', 'left'): ('turn_right', math.pi),
        }      
        self.generate_path(self.direction)

    def generate_path(self):
        for i in range(len(self.path) - 1):
            current_pos = self.path[i]
            next_pos = self.path[i + 1]
            dx, dy = next_pos[1] - current_pos[1], next_pos[0] - current_pos[0]
            desired_direction = self.calculate_desired_direction(dx, dy)
            
            if self.direction != desired_direction:
                action, angle = self.direction_changes[(self.direction, desired_direction)]
                self.path_actions.append((action, angle, current_pos, desired_direction))
                self.direction = desired_direction
            
            self.path_actions.append(("forward", self.forward_distance, next_pos, self.direction))
        
        self.path_actions.append(("stop", 0, current_pos, self.direction))

    def calculate_desired_direction(self, dx, dy):
        if dx == 1:
            return 'right'
        elif dx == -1:
            return 'left'
        elif dy == 1:
            return 'down'
        elif dy == -1:
            return 'up'

    def update_state(self):
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.start_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9 
        msg = Twist()

        if self.current_action_index < len(self.path_actions):
            self.action, distance, self.position, self.direction = self.path_actions[self.current_action_index]

            if self.action == "forward":    
                if elapsed_seconds >= distance / 0.2:
                    self.current_action_index += 1
                    self.start_time = self.get_clock().now()
                else:
                    msg.linear.x = 0.2  
            elif self.action in ["turn_left", "turn_right"]:
                if elapsed_seconds >= distance / 0.2:
                    self.current_action_index += 1
                    self.start_time = self.get_clock().now()
                else:
                    if self.action == "turn_left":
                        msg.angular.z = 0.2  
                    else:
                        msg.angular.z = -0.2  

            elif self.action == "stop":
                msg.angular.z = 0.0 
                msg.linear.x = 0.0  

        self.publisher_.publish(msg)  

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                       
            x, y, w, h = 0, 0, 12, 9  
            cropped_image = cv_image[y:y+h, x:x+w]

            hsv_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
            
            if self.action == "forward":         
                lower_red = np.array([0, 100, 100])
                upper_red = np.array([10, 255, 255])
                mask_red = cv2.inRange(hsv_image, lower_red, upper_red)

                if cv2.countNonZero(mask_red) > 0:
                    print(self.position)
                    self.current_action_index = -1 
                    new_obstacle_position = self.get_new_obstacle_position(self.position)
                    if new_obstacle_position not in path_planning.obstacle_positions:
                        path_planning.obstacle_positions.append(new_obstacle_position)
                    
                    self.calculate_new_path()

            cv2.imshow("Camera Image", cv_image)
            
        except Exception as e:
            self.get_logger().error('Failed to convert image: ' + str(e))
            return

        cv2.waitKey(1)
        
    def calculate_new_path(self):
        grid_drawer = DrawGrid()
        a_star = AStarAlgo(grid_drawer.grid_rows, grid_drawer.grid_cols)

        map_image = np.ones((grid_drawer.map_size, grid_drawer.map_size, 3), dtype=np.uint8) * 200  
        grid_drawer.draw_grid(map_image)
        grid_drawer.add_obstacles(map_image, path_planning.obstacle_positions)
        robot_position = self.position
        goal_position = (8, 8)
        grid_drawer.add_robot(map_image, robot_position)
        grid_drawer.add_goal(map_image, goal_position)
        

        self.path = a_star.find_shortest_path_Astar(robot_position, goal_position, obstacle_positions)

        if self.path:
            generate_arrows(map_image, self.path, grid_drawer.grid_size)
        else:
            print("No path found!")

        self.path_actions = []
        self.generate_path(self.direction)
        print(self.path)
        action, angle, *rest = self.path_actions[0]

        # You can modify the angle here
        self.path_actions[0] = (action, angle, *rest)
        print(self.path_actions)
        
        self.current_action_index = 0 
        cv2.imshow('Gridworld: Turtlebot3 with Obstacles', map_image)
    
    
    def get_new_obstacle_position(self, agent_position):
        if self.direction == "up":
            new_obstacle_position = (agent_position[0] - 1, agent_position[1])
        elif self.direction == "down":
            new_obstacle_position = (agent_position[0] + 1, agent_position[1])
        elif self.direction == "left":
            new_obstacle_position = (agent_position[0], agent_position[1] - 1)
        elif self.direction == "right":
            new_obstacle_position = (agent_position[0], agent_position[1] + 1)
        else:
            raise ValueError("Invalid agent direction")

        return new_obstacle_position
    

def main(args=None):
    rclpy.init(args=args)  
    turtlebot_controller = TurtlebotController() 
    rclpy.spin(turtlebot_controller) 
    turtlebot_controller.destroy_node()
    cv2.destroyAllWindows()  
    rclpy.shutdown()  

if __name__ == '__main__':
    main()



