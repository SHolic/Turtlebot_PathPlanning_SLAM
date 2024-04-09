import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose, Quaternion
import cv2
import numpy as np
import math
import tf_transformations

class CameraImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        self.obstacle_pub = self.create_publisher(
            PoseArray, 
            '/camera_obstacle_distances', 
            10)

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Detect obstacles
        obstacle_positions = self.detect_obstacles(cv_image)

        # Prepare PoseArray message
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_link"  # Coordinate frame of the camera

        for pose in obstacle_positions:  
            pose_array.poses.append(pose)

        self.obstacle_pub.publish(pose_array)
        # self.get_logger().info('Obstacles found in camera')


    def detect_obstacles(self, cv_image):
        # Convert BGR image to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define HSV range for red
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obstacle_positions = []
        # For each contour, draw bounding rectangle and calculate bottom center coordinates
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

            if w * h > 100:
                # Calculate bottom center coordinates
                bottom_center_x = x + w / 2
                bottom_center_y = y + h
                distance, angle = self.estimate_distance_and_angle(bottom_center_x, bottom_center_y)
                pose = self.distance_and_angle_to_pose(distance, angle)
                obstacle_positions.append(pose)
                cv2.circle(cv_image, (int(bottom_center_x), int(bottom_center_y)), 5, (0, 0, 255), -1)
                # self.get_logger().info(f"Bottom center coordinates: ({bottom_center_x}, {bottom_center_y})")
                # self.get_logger().info(f"distance: {distance}, angle: {angle}")

        # Show result image
        cv2.imshow("Detection Result", cv_image)
        cv2.waitKey(1)  # Wait a bit for the window to update

        return obstacle_positions
    
    def estimate_distance_and_angle(self, object_center_pixel_x, object_center_pixel_y):
        image_width_pixels = 640     # Image width (pixels)
        image_height_pixels = 480    # Image height (pixels)
        fov_horizontal_radians = 1.085595  # Horizontal field of view (radians)
        fov_vertical_radians = 1.085595    # Assuming vertical FOV is the same as horizontal
        camera_height_above_ground = 0.093  # Camera height from the ground (meters)    
        
        # Calculate horizontal pixel difference from image center to obstacle bottom center
        pixel_difference_horizontal = object_center_pixel_x - image_width_pixels / 2
        # Calculate horizontal angle per pixel
        angle_per_pixel_horizontal = fov_horizontal_radians / image_width_pixels
        # Calculate horizontal angle from image center to obstacle center
        angle_horizontal = pixel_difference_horizontal * angle_per_pixel_horizontal
        
        # Calculate vertical pixel difference from image center to obstacle bottom center
        pixel_difference_vertical = object_center_pixel_y - image_height_pixels / 2
        # Calculate vertical angle per pixel
        angle_per_pixel_vertical = fov_vertical_radians / image_height_pixels
        # Calculate vertical angle from bottom of the obstacle to camera optical axis
        angle_vertical = pixel_difference_vertical * angle_per_pixel_vertical
        
        # Estimate distance to the bottom of the obstacle using camera height and vertical angle difference
        distance_to_object = camera_height_above_ground / np.tan(angle_vertical)

        # Correct distance to reflect the actual position of the obstacle center
        distance_correction = 0.2836700936  # Correction value for obstacle side length
        distance_to_object_center = distance_to_object + distance_correction
        
        # Return distance (meters) and horizontal angle (degrees)
        return distance_to_object_center, np.degrees(angle_horizontal)

    def distance_and_angle_to_pose(self, distance, angle_degrees):
        # Convert angle from degrees to radians
        angle_radians = math.radians(angle_degrees)

        # Calculate obstacle's x and y coordinates in the camera coordinate system
        x = distance * math.cos(angle_radians)  # Forward distance
        y = distance * math.sin(angle_radians)  # Lateral distance

        # Create Pose message
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0  # Assuming obstacle and camera are on the same plane

        # Set orientation (here, simply used angle for Z-axis rotation)
        # Note: Real applications may need more complex transformations to correctly represent orientation in 3D space
        orientation_quaternion = self.angle_to_quaternion(angle_degrees)
        pose.orientation = orientation_quaternion

        return pose
    
    def angle_to_quaternion(self, angle_degrees):
        # Convert angle from degrees to radians
        angle_radians = math.radians(angle_degrees)
        
        # Use tf transformation library to convert Euler angles to quaternion
        # Assuming rotation only around Z-axis, with 0 rotation around X and Y axes
        quaternion = tf_transformations.quaternion_from_euler(0, 0, angle_radians)
        
        # Convert quaternion to ROS message format
        orientation = Quaternion()
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        
        return orientation

def main(args=None):
    rclpy.init(args=args)
    node = CameraImageProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
