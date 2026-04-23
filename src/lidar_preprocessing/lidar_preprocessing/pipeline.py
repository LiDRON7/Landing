from sensor_msgs.msg import PointCloud2
from .filters.nan_infinite_filter import nan_infinite_filter
from .filters.passthrough import passthrough_filter
from .filters.voxel_grid_downsampling import voxel_grid_downsampling
from .filters.outlier_removal import statistical_outlier_removal
from .filters.ground_segmentation import ransac_ground_segmentation


def _count_points(point_cloud: PointCloud2) -> int:
    return int(point_cloud.width * point_cloud.height)


def run_preprocessing_pipeline_with_stats(
    msg: PointCloud2,
    roi: dict[str, float] | None = None,
    voxel_size: float = 0.1,
    enable_nan_filter: bool = True,
    enable_voxel_filter: bool = True,
    enable_roi_filter: bool = True,
    enable_outlier_filter: bool = True,
    outlier_mean_k: int = 30,
    outlier_threshold: float = 2.0,
) -> tuple[PointCloud2, dict[str, int]]:
    # This function runs the preprocessing pipeline and returns point counts at each stage.

    # Start with the incoming message and apply filters in sequence.
    filtered_msg = msg
    stage_counts: dict[str, int] = {
        "raw_input": _count_points(filtered_msg),
    }

    if enable_nan_filter:
        filtered_msg = nan_infinite_filter(filtered_msg)
    stage_counts["after_nan_removal"] = _count_points(filtered_msg)

    if enable_voxel_filter:
        filtered_msg = voxel_grid_downsampling(filtered_msg, voxel_size)
    stage_counts["after_voxel_grid_downsampling"] = _count_points(filtered_msg)

    if enable_roi_filter and roi is not None:
        filtered_msg = passthrough_filter(
            filtered_msg,
            x_min=roi["x_min"],
            x_max=roi["x_max"],
            y_min=roi["y_min"],
            y_max=roi["y_max"],
            z_min=roi["z_min"],
            z_max=roi["z_max"],
        )
    stage_counts["after_roi_filtering"] = _count_points(filtered_msg)

    if enable_outlier_filter:
        filtered_msg = statistical_outlier_removal(
            filtered_msg,
            meanK=outlier_mean_k,
            threshold=outlier_threshold,
        )
    stage_counts["after_statistical_outlier"] = _count_points(filtered_msg)

    return filtered_msg, stage_counts


def run_preprocessing_pipeline(
    msg: PointCloud2,
    roi: dict[str, float] | None = None,
    voxel_size: float = 0.1,
    enable_nan_filter: bool = True,
    enable_voxel_filter: bool = True,
    enable_roi_filter: bool = True,
    enable_outlier_filter: bool = True,
    outlier_mean_k: int = 30,
    outlier_threshold: float = 2.0,
) -> PointCloud2:
    # Backward-compatible API that returns only the filtered cloud.
    filtered_msg, _ = run_preprocessing_pipeline_with_stats(
        msg,
        roi=roi,
        voxel_size=voxel_size,
        enable_nan_filter=enable_nan_filter,
        enable_voxel_filter=enable_voxel_filter,
        enable_roi_filter=enable_roi_filter,
        enable_outlier_filter=enable_outlier_filter,
        outlier_mean_k=outlier_mean_k,
        outlier_threshold=outlier_threshold,
    )

    return filtered_msg