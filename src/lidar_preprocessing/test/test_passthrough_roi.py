import importlib
import pathlib
import sys
import types


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))


class _FakeField:
    def __init__(self, name: str):
        self.name = name


class _FakePointCloud2:
    def __init__(self, header, fields, points):
        self.header = header
        self.fields = fields
        self.points = list(points)
        self.is_dense = True


def _install_ros_stubs() -> None:
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.PointCloud2 = _FakePointCloud2
    sensor_msgs.msg = sensor_msgs_msg

    point_cloud2 = types.ModuleType("point_cloud2")

    def read_points(msg, field_names, skip_nans=False):
        del skip_nans
        all_names = [field.name for field in msg.fields]
        indexes = [all_names.index(name) for name in field_names]
        for point in msg.points:
            yield tuple(point[index] for index in indexes)

    def create_cloud(header, fields, points):
        return _FakePointCloud2(header, fields, points)

    point_cloud2.read_points = read_points
    point_cloud2.create_cloud = create_cloud

    sensor_msgs_py = types.ModuleType("sensor_msgs_py")
    sensor_msgs_py.point_cloud2 = point_cloud2

    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
    sys.modules["sensor_msgs_py"] = sensor_msgs_py
    sys.modules["sensor_msgs_py.point_cloud2"] = point_cloud2


def test_passthrough_filter_includes_boundary_points_and_excludes_outside_points():
    _install_ros_stubs()
    module = importlib.import_module("lidar_preprocessing.filters.passthrough")

    msg = _FakePointCloud2(
        header={"frame_id": "map"},
        fields=[_FakeField("x"), _FakeField("y"), _FakeField("z")],
        points=[
            (-1.0, -1.0, -1.0),
            (1.0, 1.0, 1.0),
            (0.0, 0.0, 0.0),
            (1.0001, 0.0, 0.0),
            (0.0, -1.0001, 0.0),
            (0.0, 0.0, -1.0001),
        ],
    )

    filtered = module.passthrough_filter(
        msg,
        x_min=-1.0,
        x_max=1.0,
        y_min=-1.0,
        y_max=1.0,
        z_min=-1.0,
        z_max=1.0,
    )

    assert filtered.points == [
        (-1.0, -1.0, -1.0),
        (1.0, 1.0, 1.0),
        (0.0, 0.0, 0.0),
    ]


def test_passthrough_filter_returns_original_when_xyz_fields_missing():
    _install_ros_stubs()
    module = importlib.import_module("lidar_preprocessing.filters.passthrough")

    msg = _FakePointCloud2(
        header={"frame_id": "map"},
        fields=[_FakeField("intensity")],
        points=[(42.0,), (100.0,)],
    )

    filtered = module.passthrough_filter(
        msg,
        x_min=-1.0,
        x_max=1.0,
        y_min=-1.0,
        y_max=1.0,
        z_min=-1.0,
        z_max=1.0,
    )

    assert filtered is msg
