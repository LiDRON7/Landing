# Imports
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import pcl  #wrapper for PCL in Python


    
"""
    Voxel grid downsampling is a technique used to reduce the number of points in a point cloud while preserving the overall structure.
    The idea is to divide the space into a 3D grid of voxels (cubic cells) and replace all points that fall within the same voxel with a single representative point, 
    typically the centroid of the points in that voxel. This process effectively reduces the density of the point cloud while maintaining its geometric features.
"""
def voxel_grid_downsampling(point_cloud : PointCloud2, voxel_size : float) -> PointCloud2:
    # Implementation for voxel grid downsampling
    points_np = np.array(list(pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)
    
    if points_np.size == 0:
        return point_cloud  # Return the original point cloud if it's empty
    
    #convert the numpy array to a PCL PointCloud format
    cloud = pcl.PointCloud()
    cloud.from_array(points_np)
    
    #apply voxel grid downsampling
    vg = cloud.make_voxel_grid_filter()
    vg.set_leaf_size(voxel_size, voxel_size, voxel_size)
    cloud_filtered = vg.filter()
    
    # Convert back to ROS PointCloud2
    final_points = cloud_filtered.to_array()
    filtered_msg = pc2.create_cloud_xyz32(point_cloud.header, final_points)
    
    return filtered_msg



