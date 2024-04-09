import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlebot3_msgs.action import MoveRobot
from turtlebot3_msgs.srv import PlanPath, GetMapObstacles
from turtlebot3_msgs.msg import Point2D
import math

class GoalCommanderNode(Node):
    def __init__(self):
        super().__init__('goal_commander_node')
        self.get_map_obstacles_client = self.create_client(GetMapObstacles, '/get_map_obstacles')
        while not self.get_map_obstacles_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/get_map_obstacles service not available, waiting again...')

        self.plan_path_client = self.create_client(PlanPath, '/plan_path')
        while not self.plan_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/plan_path service not available, waiting again...')

        self.action_client = ActionClient(self, MoveRobot, '/move_robot')

        self.robot_position = (4, 5)  # Example initial position
        self.robot_orientation = math.radians(90.0)  # Example initial orientation
        self.goal_position = (8, 8)  # Goal position
        self.move_distance = 0.47

        self.initialize()

    def initialize(self):
        self.obstacle_positions = self.update_obstacle_positions()
        self.path = self.update_path()

    def update_obstacle_positions(self):
        future = self.get_map_obstacles_client.call_async(GetMapObstacles.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return [(obstacle.x, obstacle.y) for obstacle in future.result().obstacles]
        else:
            self.get_logger().error('Failed to call service /get_map_obstacles')
            return []
        
    def update_path(self):
        # Create service request and set start and goal positions
        request = PlanPath.Request()
        request.start.x = self.robot_position[0]
        request.start.y = self.robot_position[1]
        request.goal.x = self.goal_position[0]
        request.goal.y = self.goal_position[1]
        request.obstacles = [Point2D(x=obstacle[0], y=obstacle[1]) for obstacle in self.obstacle_positions]

        # Call the service asynchronously
        future = self.plan_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return [(p.x, p.y) for p in future.result().path]
        else:
            self.get_logger().error('Failed to call service /plan_path')
            return []
        
    def execute_path(self):
        while self.path:
            print("path: ", self.path)
            # Variable to record if path needs to be replanned
            replan_path = False

            # Iterating using index to continue from current position after path update
            i = 1
            while i < len(self.path):
                current_goal = self.path[i]
                print("robot_position: ", self.robot_position)
                print("current_goal: ", current_goal)

                # Check if path complete
                if current_goal == self.goal_position:
                    self.get_logger().info('Path Complete')
                    break
                    
                self.plan_and_move(current_goal)
                tmp_obstacles = self.update_obstacle_positions()

                # Check if obstacles have changed
                if self.obstacle_positions != tmp_obstacles:
                    self.obstacle_positions = tmp_obstacles
                    self.path = self.update_path()
                    replan_path = True
                    break  # Exit current loop, start new path execution

                i += 1  # Move to the next goal in the path

            # If no path needs to be replanned, exit loop
            if not replan_path:
                break

    def plan_and_move(self, goal):
        rotate_angle = self.calculate_rotation(self.robot_position, goal)
        move_distance = self.move_distance
        
        # Send goal and wait for result
        self.send_goal_and_wait(rotate_angle, move_distance)
        
        # Update current position and orientation
        self.robot_position = goal
        self.robot_orientation += rotate_angle

    def calculate_rotation(self, current_position, next_position):
        dx = next_position[1] - current_position[1]
        dy = -(next_position[0] - current_position[0]) # Reverse y-axis direction
        target_angle_radians = math.atan2(dy, dx)
        # Calculate the angle difference between target direction and robot's current orientation
        angle_diff_radians = target_angle_radians - self.robot_orientation
        # Normalize angle to [-π, π] range
        angle_diff_radians = (angle_diff_radians + math.pi) % (2 * math.pi) - math.pi
        return angle_diff_radians

    def send_goal_and_wait(self, rotate_angle, move_distance):
        goal_msg = MoveRobot.Goal()
        goal_msg.rotate_angle = rotate_angle
        goal_msg.move_distance = move_distance
        print("rotate_angle: ", rotate_angle)
        print("move_distance: ", move_distance)
        
        self.get_logger().info('Sending goal and waiting for result...')
        
        # Wait for the Action server
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server available.')
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Sending goal...')
        rclpy.spin_until_future_complete(self, send_goal_future)  # Wait for goal to be sent
        self.get_logger().info('Send complete.')
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)  # Wait for goal completion
        
        result = get_result_future.result().result
        if result.success:
            self.get_logger().info('Goal completed successfully')
        else:
            self.get_logger().info('Goal failed')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback received - Rotated: {feedback.rotated_angle}, Moved: {feedback.moved_distance}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalCommanderNode()
    node.execute_path()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
