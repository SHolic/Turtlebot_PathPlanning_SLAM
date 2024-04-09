import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf_transformations
import math
from turtlebot3_msgs.action import MoveRobot
import time

class MotionControlNode(Node):
    def __init__(self):
        super().__init__('motion_control_node')
        self.action_server = ActionServer(
            self,
            MoveRobot,
            '/move_robot',
            self.execute_callback)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        self.initial_orientation_q = None
        self.initial_position = None
        self.current_orientation_q = None
        self.current_position = None

    def execute_callback(self, goal_handle):
        feedback_msg = MoveRobot.Feedback()
        goal = goal_handle.request
        rotate_angle = goal.rotate_angle
        move_distance = goal.move_distance
        
        while (self.initial_orientation_q is None or self.initial_position is None) and rclpy.ok():
            self.get_logger().info('Waiting for initial odometry data...')
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Initial odometry data received.')
        
        # Rotation logic
        success = self.rotate(rotate_angle, feedback_msg, goal_handle)
        if not success:
            return MoveRobot.Result(success=False)
        
        # Movement logic
        success = self.move(move_distance, feedback_msg, goal_handle)
        if not success:
            return MoveRobot.Result(success=False)
        
        goal_handle.succeed()
        return MoveRobot.Result(success=True)

    def rotate(self, target_angle, feedback_msg, goal_handle):
        start_orientation = tf_transformations.euler_from_quaternion([
            self.initial_orientation_q.x,
            self.initial_orientation_q.y,
            self.initial_orientation_q.z,
            self.initial_orientation_q.w
        ])[2]  # Yaw
        target_orientation = start_orientation + target_angle

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            current_orientation = tf_transformations.euler_from_quaternion([
                self.current_orientation_q.x,
                self.current_orientation_q.y,
                self.current_orientation_q.z,
                self.current_orientation_q.w
            ])[2]  # Yaw

            angle_diff = target_orientation - current_orientation

            if abs(angle_diff) < 0.01:  # Threshold to consider rotation complete
                break

            # Publish feedback
            feedback_msg.rotated_angle = current_orientation - start_orientation
            goal_handle.publish_feedback(feedback_msg)

            # Continue rotating
            velocity_msg = Twist()
            velocity_msg.angular.z = 0.3 if angle_diff > 0 else -0.3
            self.cmd_vel_publisher.publish(velocity_msg)

        # Stop rotation
        velocity_msg = Twist()
        self.cmd_vel_publisher.publish(velocity_msg)
        return True

    def move(self, target_distance, feedback_msg, goal_handle):
        start_position = self.initial_position
        traveled_distance = 0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            current_position = self.current_position
            traveled_distance = math.sqrt(
                (current_position.x - start_position.x) ** 2 +
                (current_position.y - start_position.y) ** 2)

            if traveled_distance >= target_distance:
                break

            # Publish feedback
            feedback_msg.moved_distance = traveled_distance
            goal_handle.publish_feedback(feedback_msg)

            # Continue moving
            velocity_msg = Twist()
            velocity_msg.linear.x = 0.2
            self.cmd_vel_publisher.publish(velocity_msg)

        # Stop movement
        velocity_msg = Twist()
        self.cmd_vel_publisher.publish(velocity_msg)
        return True

    def odom_callback(self, msg):
        self.current_orientation_q = msg.pose.pose.orientation
        self.current_position = msg.pose.pose.position
        if self.initial_orientation_q is None or self.initial_position is None:
            self.initial_orientation_q = self.current_orientation_q
            self.initial_position = self.current_position

def main(args=None):
    rclpy.init(args=args)
    node = MotionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
