import unittest
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2 
from std_msgs.msg import Header

from lidar_preprocessing.filters.voxel_grid_downsampling import voxel_grid_downsampling
from lidar_preprocessing.filters.passthrough import passthrough_filter
from lidar_preprocessing.filters.nan_infinite_filter import nan_infinite_filter
from lidar_preprocessing.filters.ground_segmentation import ransac_ground_segmentation
from lidar_preprocessing.filters.outlier_removal import statistical_outlier_removal


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
        """
        Helper method to wrap a NumPy array into a ROS 2 PointCloud2 message.

        Args:
            points (np.ndarray): An Nx3 array of float32 coordinates.

        Returns:
            sensor_msgs.msg.PointCloud2: A message ready for filter processing.
        """
        header = Header()
        header.frame_id = 'test_frame'
        # Convert Nx3 numpy array to the ROS 2 PointCloud2 format
        return pc2.create_cloud_xyz32(header, points.astype(np.float32))

    def test_voxel_grid_downsampling(self):
        """
        Validates that the voxel grid filter correctly reduces point density.
        
        Logic: Points within the same 'leaf size' (0.1m) should be merged into 
        one single centroid point.
        """
        print("Voxel_Grid_Downsampling dummy test")

        # Two points in the [0,0,0] voxel, two in the [0.5, 0.5, 0.5] voxel
        points = np.array([
            [0.0, 0.0, 0.0],
            [0.01, 0.01, 0.01],
            [0.5, 0.5, 0.5],
            [0.51, 0.51, 0.51]
        ], dtype=np.float32)
        pc2_msg = self.create_point_cloud(points)

        # Apply filtering with a 0.1m cube leaf size
        leaf_size = 0.1
        downsampled_msg = voxel_grid_downsampling(pc2_msg, leaf_size)
        
        # Extract points from the resulting ROS message
        gen = pc2.read_points(downsampled_msg, skip_nans=True)
        downsampled_points = np.array(list(gen))

        # We expect 2 points (one for each voxel occupied)
        self.assertEqual(len(downsampled_points), 2)
        print("Voxel_Grid_Downsampling Test Passed")
    
    def test_voxel_grid_downsampling_edge_cases(self):
        """
        Checks behavior against empty clouds, identical voxels, and sparse data.
        """
        print("Voxel_Grid_Downsampling Edge Cases Test")

        # 1. EMPTY CLOUD: Should return 0 points without crashing
        points_empty = np.array([], dtype=np.float32).reshape(0, 3)
        pc2_msg_empty = self.create_point_cloud(points_empty)
        downsampled_msg_empty = voxel_grid_downsampling(pc2_msg_empty, 0.1)
        gen_empty = pc2.read_points(downsampled_msg_empty, skip_nans=True)
        downsampled_points_empty = np.array(list(gen_empty))
        self.assertEqual(len(downsampled_points_empty), 0)
        print("  - Empty cloud test passed")

        # 2. SAME VOXEL: Multiple points very close together should become 1
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

        # 3. FAR APART: Points in separate voxels should all be preserved
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
        """
        Validates the cropping logic of the passthrough filter.
        
        Points outside the specified [min, max] ranges for X, Y, or Z 
        should be discarded.
        """
        print("Passthrough Filter Test")

        points = np.array([
            [0.5, 0.5, 0.5],  # Inside all ranges
            [1.5, 1.5, 1.5],  # Outside X max (1.0)
            [-0.5, 0.5, 0.5], # Outside X min (0.0)
            [0.5, 2.5, 0.5],  # Outside Y max (2.0)
            [0.5, -1.5, 0.5], # Outside Y min (0.0)
            [0.5, 0.5, 3.5],  # Outside Z max (3.0)
            [0.5, 0.5, -2.5]  # Outside Z min (0.0)
        ], dtype=np.float32)
        pc2_msg = self.create_point_cloud(points)

        # Apply filter with bounds: X[0,1], Y[0,2], Z[0,3]
        filtered_msg = passthrough_filter(pc2_msg, 0.0, 1.0, 0.0, 2.0, 0.0, 3.0)
        
        gen = pc2.read_points(filtered_msg, skip_nans=True)
        filtered_points_structured = np.array(list(gen))
        # Flatten structured array back to standard coordinate format
        filtered_points = filtered_points_structured.view(np.float32).reshape(-1, 3)

        # Only the first point should remain
        self.assertEqual(len(filtered_points), 1)
        self.assertTrue(np.allclose(filtered_points[0], [0.5, 0.5, 0.5]))
        print("Passthrough Filter Test Passed")
    
    def test_nan_infinite_filter(self):
        """
        Ensures that invalid numerical values (NaN, Inf) are removed.
        
        This prevents downstream algorithms (like SLAM or clustering) from 
        crashing due to floating-point errors.
        """
        print("NaN/Infinite Filter Test")

        points = np.array([
            [0.0, 0.0, 0.0],    # Valid
            [1.0, 1.0, np.nan], # Not a Number
            [2.0, np.inf, 2.0], # Positive Infinity
            [-np.inf, 3.0, 3.0],# Negative Infinity
            [4.0, 4.0, 4.0]     # Valid
        ], dtype=np.float32)
        pc2_msg = self.create_point_cloud(points)

        filtered_msg = nan_infinite_filter(pc2_msg)
        
        gen = pc2.read_points(filtered_msg, skip_nans=False)
        filtered_points_structured = np.array(list(gen))
        filtered_points = filtered_points_structured.view(np.float32).reshape(-1, 3)

        # Expect only the 1st and 5th points to survive
        self.assertEqual(len(filtered_points), 2)
        self.assertTrue(np.allclose(filtered_points, [[0.0, 0.0, 0.0], [4.0, 4.0, 4.0]]))
        print("NaN/Infinite Filter Test Passed")

    def test_ransac_ground_segmentation_empty_cloud(self):
        """
        Verifies that `ransac_ground_segmentation` returns (None, None)
        for an empty input cloud.
        """
        print("RANSAC Ground Segmentation Empty Cloud Test")
        points_empty = np.array([], dtype=np.float32).reshape(0, 3)
        pc2_msg_empty = self.create_point_cloud(points_empty)

        ground_msg, obstacle_msg = ransac_ground_segmentation(
            pc2_msg_empty, dist_threshold=0.1, num_iterations=10
        )

        self.assertIsNone(ground_msg)
        self.assertIsNone(obstacle_msg)
        print("  - Empty cloud test passed")

    def test_ransac_ground_segmentation_plane_vs_obstacles(self):
        """
        Creates a simple cloud where several points lie on z=0 (ground)
        and a couple of points are elevated (obstacles). Asserts that
        RANSAC separates them into ground and obstacle messages.
        """
        print("RANSAC Ground Segmentation Plane vs Obstacles Test")

        plane_points = [
            [0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.5, 0.5, 0.0],
        ]
        obstacle_points = [
            [0.2, 0.2, 1.0],
            [0.8, 0.3, 1.0],
        ]

        all_points = np.array(plane_points + obstacle_points, dtype=np.float32)
        pc2_msg = self.create_point_cloud(all_points)

        ground_msg, obstacle_msg = ransac_ground_segmentation(
            pc2_msg, dist_threshold=0.01, num_iterations=100
        )

        # Extract points from the resulting ROS messages
        gen_ground = pc2.read_points(ground_msg, skip_nans=True)
        gen_obst = pc2.read_points(obstacle_msg, skip_nans=True)

        ground_list = list(gen_ground)
        try:
            ground_pts = np.array(ground_list, dtype=np.float32)
        except Exception:
            ground_pts = np.array([tuple(p) for p in ground_list], dtype=np.float32)

        obst_list = list(gen_obst)
        try:
            obst_pts = np.array(obst_list, dtype=np.float32)
        except Exception:
            obst_pts = np.array([tuple(p) for p in obst_list], dtype=np.float32)

        # Expect the six plane points and two obstacle points
        self.assertEqual(len(ground_pts), 6)
        self.assertEqual(len(obst_pts), 2)
        # Check z-values roughly match expected plane/obstacle heights
        self.assertTrue(np.allclose(ground_pts[:, 2], 0.0, atol=1e-3))
        self.assertTrue(np.allclose(obst_pts[:, 2], 1.0, atol=1e-3))
        print("RANSAC Ground Segmentation Plane vs Obstacles Test Passed")

    # Note: RANSAC ground segmentation tests are defined above; duplicate
    # definitions were removed to avoid running the same tests twice.

class TestStatisticalFilter(unittest.TestCase):

    def create_point_cloud(self, points):
            header = Header()
            header.frame_id = 'test_frame'
            return pc2.create_cloud_xyz32(header, points.astype(np.float32))

    def test_statistical_outlier_filter(self):
        # Checks that SOR removes isolated noise points while keeping clustered points.

        print("Statistical Outlier Filter Test")

        points = np.array([
            [0.0, 0.0, 0.0],
            [0.01, 0.0, 0.0],
            [0.0, 0.01, 0.0],
            [0.01, 0.01, 0.0],
            [5.0, 5.0, 5.0]   # Outlier
        ], dtype=np.float32)

        pc2_msg = self.create_point_cloud(points)

        filtered_msg = statistical_outlier_removal(pc2_msg, meanK=3, threshold=1)

        gen = pc2.read_points(filtered_msg, skip_nans=False)
        filtered_points_structured = np.array(list(gen))
        filtered_points = filtered_points_structured.view(np.float32).reshape(-1, 3)

        # The far point should be removed
        self.assertEqual(len(filtered_points), 4)

        # Make sure the outlier is not present
        self.assertFalse(
            np.any(np.all(np.isclose(filtered_points, [5.0, 5.0, 5.0]), axis=1))
        )

    def test_statistical_outlier_filter_empty_cloud(self):
        points = np.array([], dtype=np.float32).reshape(0, 3)
        pc2_msg = self.create_point_cloud(points)

        filtered_msg = statistical_outlier_removal(pc2_msg, meanK=30, threshold=2)

        gen = pc2.read_points(filtered_msg, skip_nans=True)
        filtered_points = np.array(list(gen))

        self.assertEqual(len(filtered_points), 0)

    def test_statistical_outlier_filter_no_outliers(self):
        points = np.array([
            [0.0, 0.0, 0.0],
            [0.01, 0.0, 0.0],
            [0.0, 0.01, 0.0],
            [0.01, 0.01, 0.0]
        ], dtype=np.float32)

        pc2_msg = self.create_point_cloud(points)

        filtered_msg = statistical_outlier_removal(pc2_msg, meanK=2, threshold=2)

        gen = pc2.read_points(filtered_msg, skip_nans=True)
        filtered_points_structured = np.array(list(gen))
        filtered_points = filtered_points_structured.view(np.float32).reshape(-1, 3)

        self.assertEqual(len(filtered_points), 4)

if __name__ == '__main__':
    # Entry point for running tests via 'python3 test_filters.py'
    unittest.main()