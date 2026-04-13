import math
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def nan_infinite_filter(msg: PointCloud2) -> PointCloud2:
    """Filter out NaN and infinite points from the point cloud.
    
    This filter removes points that contain NaN or infinite values in their
    x, y, or z coordinates, returning only valid points.
    
    Args:
        msg: Input PointCloud2 message
        
    Returns:
        Filtered PointCloud2 message with only valid points
    """
    field_names = [field.name for field in msg.fields]
    
    if "x" not in field_names or "y" not in field_names or "z" not in field_names:
        # Pass through unchanged if x/y/z fields missing
        return msg
    
    x_idx = field_names.index("x")
    y_idx = field_names.index("y")
    z_idx = field_names.index("z")
    
    valid_points = []
    dropped = 0
    
    for point in point_cloud2.read_points(msg, field_names=field_names, skip_nans=False):
        if math.isfinite(point[x_idx]) and math.isfinite(point[y_idx]) and math.isfinite(point[z_idx]):
            valid_points.append(point)
        else:
            dropped += 1
    
    filtered_msg = point_cloud2.create_cloud(msg.header, msg.fields, valid_points)
    filtered_msg.is_dense = True
    
    return filtered_msg
