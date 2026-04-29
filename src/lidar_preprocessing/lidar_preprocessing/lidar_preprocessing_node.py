# This imports the ROS2 Python Client Library. It lets python talk to ROS2.
import rclpy

# This imports the Node class.
from rclpy.node import Node

# This imports the PointCloud2 message type, which is used to represent 3D point cloud data.
from sensor_msgs.msg import PointCloud2

# Ensure your pipeline.py has the 'run_preprocessing_pipeline_with_stats' function updated 
# to return: filtered_msg, ground_msg, obstacles_msg, stage_counts
from .pipeline import run_preprocessing_pipeline_with_stats


class LidarPreprocessingNode(Node):
    def __init__(self):
        # Initialize the node with the name 'lidar_preprocessing_node'
        super().__init__('lidar_preprocessing_node')

        self.input_topic = '/lidar/points/points'
        self.output_topic = '/lidar/points/filtered'
        self.ground_topic = '/lidar/points/ground'
        self.obstacles_topic = '/lidar/points/obstacles'

        # ROI Parameters (Using the wider range from your ransac_unit_test branch)
        self.declare_parameter('roi.x_min', -10.0)
        self.declare_parameter('roi.x_max', 10.0)
        self.declare_parameter('roi.y_min', -10.0)
        self.declare_parameter('roi.y_max', 10.0)
        self.declare_parameter('roi.z_min', -2.0)
        self.declare_parameter('roi.z_max', 5.0)

        # Filter Toggles from Main
        self.declare_parameter('filters.enable_nan', True)
        self.declare_parameter('filters.enable_voxel', True)
        self.declare_parameter('filters.enable_roi', True)
        self.declare_parameter('filters.enable_outlier', True)

        # Algorithm Parameters
        self.declare_parameter('voxel.size', 0.1)
        self.declare_parameter('outlier.mean_k', 50)
        self.declare_parameter('outlier.threshold', 3.0)
        self.declare_parameter('ransac.dist_threshold', 0.2)
        self.declare_parameter('ransac.num_iterations', 100)

        # Subscriptions and Publishers
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.ground_publisher = self.create_publisher(PointCloud2, self.ground_topic, 10)
        self.obstacles_publisher = self.create_publisher(PointCloud2, self.obstacles_topic, 10)

        # Logging initialization
        self.get_logger().info(f'Node started. Subscribed to {self.input_topic}')

    def pointcloud_callback(self, msg: PointCloud2):
        # 1. Extract Parameters
        roi = {
            'x_min': float(self.get_parameter('roi.x_min').value),
            'x_max': float(self.get_parameter('roi.x_max').value),
            'y_min': float(self.get_parameter('roi.y_min').value),
            'y_max': float(self.get_parameter('roi.y_max').value),
            'z_min': float(self.get_parameter('roi.z_min').value),
            'z_max': float(self.get_parameter('roi.z_max').value),
        }

        ransac_params = {
            'dist_threshold': float(self.get_parameter('ransac.dist_threshold').value),
            'num_iterations': int(self.get_parameter('ransac.num_iterations').value),
        }

        # 2. Run the preprocessing pipeline
        # Note: We pass both the toggles and the RANSAC settings here
        filtered_msg, ground_msg, obstacles_msg, stage_counts = run_preprocessing_pipeline_with_stats(
            msg,
            roi=roi,
            ransac_params=ransac_params,
            voxel_size=float(self.get_parameter('voxel.size').value),
            enable_nan_filter=bool(self.get_parameter('filters.enable_nan').value),
            enable_voxel_filter=bool(self.get_parameter('filters.enable_voxel').value),
            enable_roi_filter=bool(self.get_parameter('filters.enable_roi').value),
            enable_outlier_filter=bool(self.get_parameter('filters.enable_outlier').value),
            outlier_mean_k=int(self.get_parameter('outlier.mean_k').value),
            outlier_threshold=float(self.get_parameter('outlier.threshold').value),
        )

        # 3. Calculate Statistics
        num_points_before = stage_counts['raw_input']
        num_points_after = stage_counts['after_statistical_outlier']
        removed_points = num_points_before - num_points_after
        reduction = (1 - (num_points_after / num_points_before)) * 100 if num_points_before > 0 else 0.0

        # 4. Log detailed stage counts
        self.get_logger().info(
            f'Stats | In: {num_points_before} | Out: {num_points_after} | '
            f'Reduced: {reduction:.2f}%'
        )

        # 5. Publish all three clouds
        self.publisher.publish(filtered_msg)
        self.ground_publisher.publish(ground_msg)
        self.obstacles_publisher.publish(obstacles_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPreprocessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()