import struct
import math
from sensor_msgs.msg import PointCloud2

def voxel_grid_downsampling(point_cloud: PointCloud2, voxel_size: float) -> PointCloud2:
    # 1. Basic check
    if not point_cloud.data:
        return point_cloud

    # Extract points from the byte array
    # PointCloud2 data is usually packed as (x, y, z) + other fields.
    point_step = point_cloud.point_step
    data = point_cloud.data
    voxels = {}

    for i in range(0, len(data), point_step):
        # Unpack 3 floats (x, y, z) - 'fff' means 3 floats (4 bytes each)
        try:
            x, y, z = struct.unpack_from('fff', data, i)
            
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            # -----------------------------------------------

        except struct.error:
            continue

        # 3. Find the Voxel Index
        vx = int(math.floor(x / voxel_size + 1e-9))
        vy = int(math.floor(y / voxel_size + 1e-9))
        vz = int(math.floor(z / voxel_size + 1e-9))
        voxel_key = (vx, vy, vz)

        # Store points in the voxel to calculate the average later
        if voxel_key not in voxels:
            voxels[voxel_key] = [0.0, 0.0, 0.0, 0] # sum_x, sum_y, sum_z, count
        
        voxels[voxel_key][0] += x
        voxels[voxel_key][1] += y
        voxels[voxel_key][2] += z
        voxels[voxel_key][3] += 1

    # Calculate Centroids and build new byte array
    new_data = bytearray()
    for key in voxels:
        sum_x, sum_y, sum_z, count = voxels[key]
        # Average the points in this voxel
        avg_x = sum_x / count
        avg_y = sum_y / count
        avg_z = sum_z / count
        
        # Pack the 3 floats back into bytes
        new_data.extend(struct.pack('fff', avg_x, avg_y, avg_z))
        
        # Padding to maintain the original point_step structure
        padding = point_step - 12
        if padding > 0:
            new_data.extend(b'\x00' * padding)

    # 6. Create the output message
    downsampled_cloud = PointCloud2()
    downsampled_cloud.header = point_cloud.header
    downsampled_cloud.height = 1
    downsampled_cloud.width = len(voxels)
    downsampled_cloud.fields = point_cloud.fields
    downsampled_cloud.is_bigendian = point_cloud.is_bigendian
    downsampled_cloud.point_step = point_cloud.point_step
    downsampled_cloud.row_step = downsampled_cloud.width * point_cloud.point_step
    downsampled_cloud.is_dense = point_cloud.is_dense
    downsampled_cloud.data = bytes(new_data)

    return downsampled_cloud
