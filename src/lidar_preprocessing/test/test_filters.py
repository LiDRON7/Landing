import unittest
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2 
from std_msgs.msg import Header

from lidar_preprocessing.filters.voxel_grid_downsampling import voxel_grid_downsampling
from lidar_preprocessing.filters.passthrough import passthrough_filter


"""
This file tests all the filters using Python's unittest library.

In these test cases, we use static points. This means that Gazebo does not
need to be manipulated or running for the tests to work.

To run this test:
1. Follow the setup instructions provided in the WorkSetup PDF.
2. After completing the setup, open a terminal.
3. Navigate to the folder named "test".
4. Run the following command:

    python3 test_filters.py

This will execute all the filter tests defined in this file.
"""



class TestPreprocessingFilters(unittest.TestCase):

    def create_point_cloud(self, points):
        """Creates a PointCloud2 message using official ROS 2 tools."""
        header = Header()
        header.frame_id = 'test_frame'
        # points should be a Nx3 numpy array
        return pc2.create_cloud_xyz32(header, points.astype(np.float32))

    def test_voxel_grid_downsampling(self):

        print("Voxel_Grid_Downsampling dummy test")

        points = np.array([
            [0.0, 0.0, 0.0],
            [0.01, 0.01, 0.01],
            [0.5, 0.5, 0.5],
            [0.51, 0.51, 0.51]
        ], dtype=np.float32)
        pc2_msg = self.create_point_cloud(points)

        # Apply filtering
        leaf_size = 0.1
        downsampled_msg = voxel_grid_downsampling(pc2_msg, leaf_size)
        
        # Convert back to numpy to check length
        # read_points returns a generator of (x, y, z) tuples
        gen = pc2.read_points(downsampled_msg, skip_nans=True)
        downsampled_points = np.array(list(gen))

        self.assertEqual(len(downsampled_points), 2)
        print("Voxel_Grid_Downsampling Test Passed")
    
    def test_voxel_grid_downsampling_edge_cases(self):
        print("Voxel_Grid_Downsampling Edge Cases Test")

        # Test with an empty point cloud
        points_empty = np.array([], dtype=np.float32).reshape(0, 3)
        pc2_msg_empty = self.create_point_cloud(points_empty)
        downsampled_msg_empty = voxel_grid_downsampling(pc2_msg_empty, 0.1)
        gen_empty = pc2.read_points(downsampled_msg_empty, skip_nans=True)
        downsampled_points_empty = np.array(list(gen_empty))
        self.assertEqual(len(downsampled_points_empty), 0)
        print("  - Empty cloud test passed")

        # Test with all points in the same voxel
        points_same_voxel = np.array([
            [0.0, 0.0, 0.0],
            [0.01, 0.02, 0.03],
            [0.04, 0.05, 0.06]
        ], dtype=np.float32)
        pc2_msg_same_voxel = self.create_point_cloud(points_same_voxel)
        downsampled_msg_same_voxel = voxel_grid_downsampling(pc2_msg_same_voxel, 0.1)
        gen_same_voxel = pc2.read_points(downsampled_msg_same_voxel, skip_nans=True)
        downsampled_points_same_voxel = np.array(list(gen_same_voxel))
        self.assertEqual(len(downsampled_points_same_voxel), 1)
        print("  - Same voxel test passed")

        # Test with points far apart, should not be downsampled
        points_far_apart = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [2.0, 2.0, 2.0]
        ], dtype=np.float32)
        pc2_msg_far_apart = self.create_point_cloud(points_far_apart)
        downsampled_msg_far_apart = voxel_grid_downsampling(pc2_msg_far_apart, 0.1)
        gen_far_apart = pc2.read_points(downsampled_msg_far_apart, skip_nans=True)
        downsampled_points_far_apart = np.array(list(gen_far_apart))
        self.assertEqual(len(downsampled_points_far_apart), 3)
        print("  - Far apart points test passed")
        
        print("Voxel_Grid_Downsampling Edge Cases Test Passed")
    
    def test_passthrough_filter(self):
        print("Passthrough Filter Test")

        points = np.array([
            [0.5, 0.5, 0.5],
            [1.5, 1.5, 1.5],  # Outside x range
            [-0.5, 0.5, 0.5], # Outside x range
            [0.5, 2.5, 0.5],  # Outside y range
            [0.5, -1.5, 0.5], # Outside y range
            [0.5, 0.5, 3.5],  # Outside z range
            [0.5, 0.5, -2.5]  # Outside z range
        ], dtype=np.float32)
        pc2_msg = self.create_point_cloud(points)

        filtered_msg = passthrough_filter(pc2_msg, 0.0, 1.0, 0.0, 2.0, 0.0, 3.0)
        
        gen = pc2.read_points(filtered_msg, skip_nans=True)
        filtered_points_structured = np.array(list(gen))
        filtered_points = filtered_points_structured.view(np.float32).reshape(-1, 3)

        self.assertEqual(len(filtered_points), 1)
        self.assertTrue(np.allclose(filtered_points[0], [0.5, 0.5, 0.5]))
        print("Passthrough Filter Test Passed")

if __name__ == '__main__':
    unittest.main()