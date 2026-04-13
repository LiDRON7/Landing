
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

        self.declare_parameter('roi.x_min', -10.0)
        self.declare_parameter('roi.x_max', 10.0)
        self.declare_parameter('roi.y_min', -10.0)
        self.declare_parameter('roi.y_max', 10.0)
        self.declare_parameter('roi.z_min', -2.0)
        self.declare_parameter('roi.z_max', 5.0)

        
        # Listen to the input topic and call the pointcloud_callback function whenever a new message is received
        # 10 is the queue size, determines how many messages to buffer if the processing is slower than the incoming data rate
        self.subscription = self.create_subscription(PointCloud2, self.input_topic, self.pointcloud_callback, 10) # Subscriber

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10) # Publisher        

        # Log the topics we are subscribing to and publishing to
        self.get_logger().info(f'Subscribed to {self.input_topic}')
        self.get_logger().info(f'Publishing to {self.output_topic}')

    
    def pointcloud_callback(self, msg: PointCloud2):
        # This function is called whenever a new PointCloud2 message is received on the input topic
        self.get_logger().info('Received point cloud message')

        roi = {
            'x_min': float(self.get_parameter('roi.x_min').value),
            'x_max': float(self.get_parameter('roi.x_max').value),
            'y_min': float(self.get_parameter('roi.y_min').value),
            'y_max': float(self.get_parameter('roi.y_max').value),
            'z_min': float(self.get_parameter('roi.z_min').value),
            'z_max': float(self.get_parameter('roi.z_max').value),
        }

        # Run the preprocessing pipeline on the incoming point cloud message
        filtered_msg = run_preprocessing_pipeline(msg, roi=roi)

        # Publish the processed point cloud message to the output topic
        self.publisher.publish(filtered_msg)
        self.get_logger().info('Published filtered point cloud message')




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

