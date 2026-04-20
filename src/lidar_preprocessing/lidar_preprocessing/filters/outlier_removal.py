# Imports
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

def statistical_outlier_removal(point_cloud : PointCloud2, meanK=30, threshold=2) -> PointCloud2:
    """
    Removes points that are too far away from its neighbors.
    Time Complexity: O(n^2)

    Args:
        point_cloud (PointCloud2): The cloud of points which will be filtered.
        meank (int): number of neighbors to analyze
        threshold (float): standard deviation threshold, how strict the filter is.
            - Small: removes more points
            - Large: removes less points

    Returns:
        PointCloud2: The filtered pointcloud.    

    Note: This python version is fine as a prototype, but is O(n^2) because it builds all pairwise distances. 
          For a dense LiDAR point cloud, it can get slow. Reason why native PCL/C++ is better long term.
    """

    field_names = ("x", "y", "z")

    # Reads all (x, y, z) points from the PointCloud2 message as a Numpy array of shape (n, 3) 
    points = point_cloud2.read_points_numpy(point_cloud, field_names, skip_nans = True).astype(np.float32)

    # Keep only points where x, y, z are all finite
    points = points[np.all(np.isfinite(points), axis=1)]

    if len(points) == 0:
        return point_cloud

    if len(points) <= meanK:
        return point_cloud
    
     # Array to store average distance to the k nearest neighbors for each point
    average_distances = np.empty(len(points), dtype=np.float32)

    for i in range(len(points)):
        current_point = points[i]

        # Compute distance from current point to every other point
        # np.linalg.norm(...,axis=1) computes the length of each vector, which is the Euclidean distance.
        distances = np.linalg.norm(points - current_point, axis=1)

        # Ignore distance from the point to itself
        distances[i] = np.inf

        # Get the k nearest distances without fully sorting
        nearest_distances = np.partition(distances, meanK)[:meanK]

        # Get the distances to the k nearest neighbors
        average_distances[i] = np.mean(nearest_distances)

    # Compute global mean and standard deviation
    global_mean = np.mean(average_distances)
    global_std = np.std(average_distances)

    # Points above this threshold are considrered outliers
    new_threshold = global_mean + (threshold * global_std)

    # Keep only points whose average distance is below the threshold
    mask = average_distances <= new_threshold
    filtered_cloud = points[mask]

    # Convert filtered points back into a PointCloud2 message
    filtered_msg = point_cloud2.create_cloud_xyz32(point_cloud.header, filtered_cloud.tolist())

    return filtered_msg

