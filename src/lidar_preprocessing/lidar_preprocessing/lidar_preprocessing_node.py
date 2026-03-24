
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

        
        # Listen to the input topic and call the pointcloud_callback function whenever a new message is received
        # 10 is the queue size, determines how many messages to buffer if the processing is slower than the incoming data rate
        self.subscription = self.create_subscription(PointCloud2, self.input_topic, self.pointcloud_callback, 10) # Subscriber

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10) # Publisher        

        # Log the topics we are subscribing to and publishing to
        self.get_logger().info(f'Subscribed to {self.input_topic}')
        self.get_logger().info(f'Publishing to {self.output_topic}')

    
    def pointcloud_callback(self, msg: PointCloud2):
        # Calculate points before filtering
        # PointCloud2 is 2D (width x height). If it's "unorganized", height is 1.
        num_points_before = msg.width * msg.height

        # Run the preprocessing pipeline
        filtered_msg = run_preprocessing_pipeline(msg)

        # Calculate points after filtering
        num_points_after = filtered_msg.width * filtered_msg.height

        # Calculate reduction percentage
        if num_points_before > 0:
            reduction = (1 - (num_points_after / num_points_before)) * 100
        else:
            reduction = 0

        # Print 
        self.get_logger().info(
            f'--- Filter Stats ---\n'
            f'Original: {num_points_before} points\n'
            f'Filtered: {num_points_after} points\n'
            f'Reduced by: {reduction:.2f}%\n'
            f'--------------------'
        )

        # Publish the processed point cloud message
        self.publisher.publish(filtered_msg)




def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library

    node = LidarPreprocessingNode()  # Create an instance of the LidarPreprocessingNode

    try:
        rclpy.spin(node) # Keep the node running
    except KeyboardInterrupt:
        pass  # Allow shutdown on Ctrl+C

    node.destroy_node() # Clean up the node
    rclpy.shutdown() # Shutdown the ROS2 client library

if __name__=='__main__':
    main()

