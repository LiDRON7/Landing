from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def passthrough_filter(
    msg: PointCloud2,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> PointCloud2:
    """Keep points inside an axis-aligned ROI using inclusive bounds."""
    field_names = [field.name for field in msg.fields]

    if "x" not in field_names or "y" not in field_names or "z" not in field_names:
        # Pass through unchanged if x/y/z fields are missing.
        return msg

    x_idx = field_names.index("x")
    y_idx = field_names.index("y")
    z_idx = field_names.index("z")

    points = list(point_cloud2.read_points(msg, field_names=field_names, skip_nans=False))

    if not points:
        filtered_msg = point_cloud2.create_cloud(msg.header, msg.fields, [])
        filtered_msg.is_dense = False
        return filtered_msg

    kept_points = []
    for point in points:
        x = point[x_idx]
        y = point[y_idx]
        z = point[z_idx]

        if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max:
            kept_points.append(point)

    filtered_msg = point_cloud2.create_cloud(msg.header, msg.fields, kept_points)
    filtered_msg.is_dense = False

    return filtered_msg