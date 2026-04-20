import open3d as o3d

def test_open3d_version():
    """
    Test to ensure open3d is installed and accessible.
    This test will print the version of the open3d library.
    """
    print(f"Successfully imported open3d version: {o3d.__version__}")
    assert o3d.__version__ is not None

if __name__ == '__main__':
    test_open3d_version()