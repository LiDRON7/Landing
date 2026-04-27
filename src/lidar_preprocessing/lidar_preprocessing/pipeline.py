from sensor_msgs.msg import PointCloud2
from .filters.nan_infinite_filter import nan_infinite_filter
from .filters.passthrough import passthrough_filter
from .filters.voxel_grid_downsampling import voxel_grid_downsampling
from .filters.outlier_removal import statistical_outlier_removal
from .filters.ground_segmentation import ransac_ground_segmentation

def run_preprocessing_pipeline(
    msg: PointCloud2,
    roi: dict[str, float] | None = None,
) -> PointCloud2:
    # This function will run the entire preprocessing pipeline for the LiDAR data.

    # Start with the incoming message and apply filters in sequence.
    filtered_msg = nan_infinite_filter(msg)

    if roi is not None:
        filtered_msg = passthrough_filter(
            filtered_msg,
            x_min=roi["x_min"],
            x_max=roi["x_max"],
            y_min=roi["y_min"],
            y_max=roi["y_max"],
            z_min=roi["z_min"],
            z_max=roi["z_max"],
        )

    filtered_msg = statistical_outlier_removal(msg)

    # NOTE: Julian when implementing RANSAC delete this and change it to recieve from the function 
    # Example: ground_msg, obstacles_msg = ransac_ground_segmentation(filtered_msg)
    ground_msg = filtered_msg
    obstacles_msg = filtered_msg

    return filtered_msg, ground_msg, obstacles_msg