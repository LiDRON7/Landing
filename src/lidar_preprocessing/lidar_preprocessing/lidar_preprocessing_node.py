# This imports the ROS2 Python Client Library. It lets python talk to ROS2.
import rclpy

# This imports the Node class.
from rclpy.node import Node

# This imports the PointCloud2 message type, which is used to represent 3D point cloud data.
from sensor_msgs.msg import PointCloud2

from .pipeline import run_preprocessing_pipeline_with_stats, run_preprocessing_pipeline


class LidarPreprocessingNode(Node):
    def __init__(self):
        # Initialize the node with the name 'lidar_preprocessing_node'
        super().__init__('lidar_preprocessing_node')

        self.input_topic = '/lidar/points/points'  # The topic to subscribe to for incoming point cloud data
        self.output_topic = '/lidar/points/filtered'  # The topic to publish the processed point cloud data

        self.declare_parameter('roi.x_min', -2.0)
        self.declare_parameter('roi.x_max', 2.0)
        self.declare_parameter('roi.y_min', -2.0)
        self.declare_parameter('roi.y_max', 2.0)
        self.declare_parameter('roi.z_min', -1.0)
        self.declare_parameter('roi.z_max', 2.0)
        self.declare_parameter('filters.enable_nan', False)
        self.declare_parameter('filters.enable_voxel', False)
        self.declare_parameter('filters.enable_roi', False)
        self.declare_parameter('filters.enable_outlier', True)
        self.declare_parameter('voxel.size', 0.1)
        self.declare_parameter('outlier.mean_k', 50)
        self.declare_parameter('outlier.threshold', 3.0)

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
        roi = {
            'x_min': float(self.get_parameter('roi.x_min').value),
            'x_max': float(self.get_parameter('roi.x_max').value),
            'y_min': float(self.get_parameter('roi.y_min').value),
            'y_max': float(self.get_parameter('roi.y_max').value),
            'z_min': float(self.get_parameter('roi.z_min').value),
            'z_max': float(self.get_parameter('roi.z_max').value),
        }


        enable_nan = bool(self.get_parameter('filters.enable_nan').value)
        enable_voxel = bool(self.get_parameter('filters.enable_voxel').value)
        enable_roi = bool(self.get_parameter('filters.enable_roi').value)
        enable_outlier = bool(self.get_parameter('filters.enable_outlier').value)

        voxel_size = float(self.get_parameter('voxel.size').value)
        outlier_mean_k = int(self.get_parameter('outlier.mean_k').value)
        outlier_threshold = float(self.get_parameter('outlier.threshold').value)

        # Run the preprocessing pipeline and collect stage-by-stage point counts.
        filtered_msg, stage_counts = run_preprocessing_pipeline_with_stats(
            msg,
            roi=roi,
            voxel_size=voxel_size,
            enable_nan_filter=enable_nan,
            enable_voxel_filter=enable_voxel,
            enable_roi_filter=enable_roi,
            enable_outlier_filter=enable_outlier,
            outlier_mean_k=outlier_mean_k,
            outlier_threshold=outlier_threshold,
        )

        # Run the preprocessing pipeline on the incoming point cloud message
        filtered_msg, ground_msg, obstacles_msg = run_preprocessing_pipeline(msg, roi=roi)

        # Calculate points after filtering
        num_points_before = stage_counts['raw_input']
        num_points_after = stage_counts['after_statistical_outlier']

        # Calculate removed points and reduction percentage
        removed_points = num_points_before - num_points_after
        if num_points_before > 0:
            reduction = (1 - (num_points_after / num_points_before)) * 100
        else:
            reduction = 0.0

        # Print baseline stage counts required for sensitivity analysis.
        self.get_logger().info(
            'Stage counts | '
            f'Raw input: {stage_counts["raw_input"]} | '
            f'After NaN removal: {stage_counts["after_nan_removal"]} | '
            f'After Voxel Grid Downsampling: {stage_counts["after_voxel_grid_downsampling"]} | '
            f'After ROI Filtering: {stage_counts["after_roi_filtering"]} | '
            f'After Statistical Outlier: {stage_counts["after_statistical_outlier"]}'
        )

        # Print aggregate statistics.
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