from sensor_msgs.msg import PointCloud2
from .filters.passthrough import passthrough_filter
from .filters.voxel_grid_downsampling import voxel_grid_downsampling
from .filters.outlier_removal import outlier_removal
from .filters.ground_segmentation import ransac_ground_segmentation

def run_preprocessing_pipeline(msg: PointCloud2) -> PointCloud2:
    # This function will run the entire preprocessing pipeline for the LiDAR data.

    # example:
    filtered_msg = voxel_grid_downsampling(msg, 0.05)
    # filtered_msg = passthrough_filter(filtered_msg)

    return filtered_msg