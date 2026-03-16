# LiDRON Landing Pipeline

Here you will find a brief explanation of this diagram. We will not go into technical details about what **ROS2**, **PCL**, or the message structure used by ROS2 to read LiDAR point clouds (**PointCloud2**) are.

---

## ROS2 Publisher Node

This node is responsible for receiving the data from the LiDAR sensor and transforming it into a message format that ROS2 can interpret as a point cloud.

**Publishes on topic:**  
`/lidar_point`

---

## PointCloud Filter Node

This node receives the information from the `/lidar_point` topic, which contains the LiDAR point cloud data. It then applies **PCL filters** to improve the quality and usability of the point cloud data.

### Filters Used
- VoxelGrid  
- PassThrough  
- Statistical Outlier Removal  
- RANSAC  

**Publishes on topic:**  
`/lidar_points_filtered`

**Subscriber:**  
`/lidar_point`

---

## Landing Algorithm Node

This node executes the landing algorithm. It receives processed point cloud data from the `/lidar_points_filtered` topic.

**Publishes on topics:**  
N/A  

**Subscriber:**  
`/lidar_points_filtered`