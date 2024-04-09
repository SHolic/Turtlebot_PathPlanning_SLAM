import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from turtlebot3_msgs.srv import GetMapObstacles
from turtlebot3_msgs.msg import Point2D
import numpy as np
import cv2

class MapUpdateNode(Node):
    def __init__(self):
        super().__init__('map_update_node')
        # Create a subscriber to the '/obstacle_positions' topic
        self.subscription = self.create_subscription(
            PoseArray, 
            '/camera_obstacle_distances', 
            self.obstacle_callback, 
            10)
        self.subscription  # prevent unused variable warning

        self.service = self.create_service(
            GetMapObstacles,
            '/get_map_obstacles',
            self.get_map_obstacles_callback)

        self.publisher_ = self.create_publisher(
            OccupancyGrid, 
            '/updated_map', 
            10)
        
        self.timer = self.create_timer(1.0, self.publish_updated_map)
        self.grid_size = (10, 10)
        self.occupancy_grid = np.zeros(self.grid_size, dtype=int)
        # Initial obstacle positions
        self.obstacle_positions = [(1, 1), (0, 9), (4, 1), (4, 7), (8, 3), (8, 6)]

        self.init_map()

    def init_map(self):
        # Initialize the map, marking obstacles
        for pos in self.obstacle_positions:
            self.occupancy_grid[pos] = 1

    def obstacle_callback(self, msg):
        # Update obstacle positions on the map
        # Note: Now you need to convert the obstacle positions from coordinates relative to "camera_link" to map coordinates
        # for pose in msg.poses:
        #     # The conversion here will depend on your specific setup and the robot's current position on the map
        #     # Assuming robot_position is the robot's grid coordinates on the map
        #     x = int(pose.position.x) + self.robot_position[0]
        #     y = int(pose.position.y) + self.robot_position[1]
        #     if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
        #         self.occupancy_grid[y, x] = 1

        # Redisplay the map after updating the obstacle positions
        self.show_map()
        # Publish the updated map
        self.publish_updated_map()
        # self.get_logger().info('Obstacles updated in map')

    def get_map_obstacles_callback(self, request, response):
        # Add the current list of obstacle positions to the response
        for pos in self.obstacle_positions:
            obstacle = Point2D()
            obstacle.x = pos[0]
            obstacle.y = pos[1]
            response.obstacles.append(obstacle)
        return response

    def publish_updated_map(self):
        # Create an OccupancyGrid message and publish it
        grid_msg = OccupancyGrid()
        grid_msg.header = Header(frame_id='map')
        grid_msg.info.resolution = 0.47  # Actual size of each grid square (meters)
        grid_msg.info.width = 10
        grid_msg.info.height = 10
        grid_msg.data = self.occupancy_grid.flatten().tolist()
        self.publisher_.publish(grid_msg)

    def show_map(self):
        # Convert map data into a grayscale image where 255 is white and 0 is black
        # Note: OpenCV uses BGR format, but we only need grayscale values here
        height, width = self.occupancy_grid.shape
        image = np.zeros((height, width, 3), dtype=np.uint8)

        # Set obstacles to black and free space to white
        image[self.occupancy_grid == 1] = [0, 0, 0]  # Obstacle
        image[self.occupancy_grid == 0] = [255, 255, 255]  # Free space

        # Scale the image to 500x500 pixels for better visibility
        scaled_image = cv2.resize(image, (500, 500), interpolation=cv2.INTER_NEAREST)

        # Display the image using OpenCV
        cv2.imshow("Occupancy Grid Map", scaled_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    map_update_node = MapUpdateNode()
    rclpy.spin(map_update_node)
    map_update_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
