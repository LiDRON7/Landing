import open3d as o3d
import numpy as np
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

# Helper function
def pcd_to_numpy(pcd_msg):
    """
    Args:
        pcd_msg: The input sensor_msgs/msg/PointCloud2 message.
    
    Output:
        A np.float64 array of shape (N, 3), where N is the number of points.
    """
    points_numpy = point_cloud2.read_points_numpy(
        pcd_msg, field_names=("x", "y", "z")
    )
    # read_points_numpy returns a float32 array (N, 3) directly
    return points_numpy.astype(np.float64)

def numpy_to_pcd(points, header):
    """
    Converts a numpy array to a ROS2 PointCloud2 message.
    
    Args:
        points: A NumPy array of shape (N, 3) containing point coordinates.
        header: The std_msgs/msg/Header to be attached to the message.
        
    Output:
        A sensor_msgs/msg/PointCloud2 message containing the x, y, z fields.
    """
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(header, fields, points)

def ransac_ground_segmentation(pcd_msg, dist_threshold, num_iterations):
    """
    Segments ground and obstacles using Open3D RANSAC.
    
    Ags:
        pcd_msg: The input sensor_msgs/msg/PointCloud2 message.
        dist_threshold: Max distance a point can be from the plane to be an inlier.
        num_iterations: Number of random samples to find the best-fitting plane.
        
    Output:
        ground_msg: PointCloud2 message containing only ground plane points.
        obstacle_msg: PointCloud2 message containing all remaining non-ground points.
    """
    # Convert message to numpy
    points = pcd_to_numpy(pcd_msg)
    
    if points.shape[0] == 0:
        return None, None

    # Create Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    
    pcd.points = o3d.utility.Vector3dVector(points)

    # -----------RANSAC Plane Segmentation--------------
    # Identifies the dominant plane (ground) in the 3D data
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=dist_threshold,
        ransac_n=3,
        num_iterations=num_iterations
    )

    # Extract the two clouds
    ground_cloud = pcd.select_by_index(inliers)
    obstacle_cloud = pcd.select_by_index(inliers, invert=True)

    # Convert back to numpy
    ground_points = np.asarray(ground_cloud.points)
    obstacle_points = np.asarray(obstacle_cloud.points)

    # Convert to ROS2 messages
    ground_msg = numpy_to_pcd(ground_points, pcd_msg.header)
    obstacle_msg = numpy_to_pcd(obstacle_points, pcd_msg.header)

    return ground_msg, obstacle_msg