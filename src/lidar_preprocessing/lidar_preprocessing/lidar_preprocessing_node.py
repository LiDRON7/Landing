# This imports the ROS2 Python Client Library. It lets python talk to ROS2.
import rclpy

# This imports the Node class.
from rclpy.node import Node

# This imports the PointCloud2 message type, which is used to represent 3D point cloud data.
from sensor_msgs.msg import PointCloud2

from .pipeline import run_preprocessing_pipeline


class LidarPreprocessingNode(Node):
    def __init__(self):
        # Initialize the node with the name 'lidar_preprocessing_node'
        super().__init__('lidar_preprocessing_node')

        self.input_topic = '/lidar/points/points'  # The topic to subscribe to for incoming point cloud data
        self.output_topic = '/lidar/points/filtered'  # The topic to publish the processed point cloud data

        self.ground_topic = '/lidar/points/ground'
        self.obstacles_topic = '/lidar/points/obstacles'

        self.declare_parameter('roi.x_min', -10.0)
        self.declare_parameter('roi.x_max', 10.0)
        self.declare_parameter('roi.y_min', -10.0)
        self.declare_parameter('roi.y_max', 10.0)
        self.declare_parameter('roi.z_min', -2.0)
        self.declare_parameter('roi.z_max', 5.0)

        # Listen to the input topic and call the pointcloud_callback function whenever a new message is received
        # 10 is the queue size, determines how many messages to buffer if the processing is slower than the incoming data rate
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10
        )

        self.ground_publisher = self.create_publisher(
            PointCloud2,
            self.ground_topic,
            10
        )

        self.obstacles_publisher = self.create_publisher(
            PointCloud2,
            self.obstacles_topic,
            10
        )

        # Log the topics we are subscribing to and publishing to
        self.get_logger().info(f'Subscribed to {self.input_topic}')
        self.get_logger().info(f'Publishing to {self.output_topic}')

        self.get_logger().info(f'Publishing ground to {self.ground_topic}')
        self.get_logger().info(f'Publishing obstacles to {self.obstacles_topic}')

    def pointcloud_callback(self, msg: PointCloud2):
        # Calculate points before filtering
        # PointCloud2 is 2D (width x height). If it's "unorganized", height is 1.
        num_points_before = msg.width * msg.height

        roi = {
            'x_min': float(self.get_parameter('roi.x_min').value),
            'x_max': float(self.get_parameter('roi.x_max').value),
            'y_min': float(self.get_parameter('roi.y_min').value),
            'y_max': float(self.get_parameter('roi.y_max').value),
            'z_min': float(self.get_parameter('roi.z_min').value),
            'z_max': float(self.get_parameter('roi.z_max').value),
        }

        # Run the preprocessing pipeline on the incoming point cloud message
        filtered_msg, ground_msg, obstacles_msg = run_preprocessing_pipeline(msg, roi=roi)

        # Calculate points after filtering
        num_points_after = filtered_msg.width * filtered_msg.height

        # Calculate removed points and reduction percentage
        removed_points = num_points_before - num_points_after
        if num_points_before > 0:
            reduction = (1 - (num_points_after / num_points_before)) * 100
        else:
            reduction = 0.0

        # Print filter statistics
        self.get_logger().info(
            f'Points in: {num_points_before} | '
            f'Points out: {num_points_after} | '
            f'Removed: {removed_points} | '
            f'Reduced by: {reduction:.2f}%'
        )

        # Publish the processed point cloud message to the output topic
        self.publisher.publish(filtered_msg)
        self.ground_publisher.publish(ground_msg)
        self.obstacles_publisher.publish(obstacles_msg)

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library

    node = LidarPreprocessingNode()  # Create an instance of the LidarPreprocessingNode

    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass  # Allow shutdown on Ctrl+C
    finally:
        node.destroy_node()  # Clean up the node
        if rclpy.ok():
            rclpy.shutdown()  # Shutdown the ROS2 client library


if __name__ == '__main__':
    main()