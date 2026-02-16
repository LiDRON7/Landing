# LiDAR Simulation Setup (PX4 SITL + Gazebo Classic + ROS2 Foxy)
This setup describes how to configure and validate a LiDAR simulation.

## Prerequisites
- Docker installed
- Docker Desktop opened

## Step 1: Create a Persistent Container (Recommended)

Open the **Ubuntu** terminal and go to the PX4 gazebo docker folder:
```bash
cd gazebo-docker
```

**Do NOT** use --rm when running the container.
The --rm flag deletes the container when you exit, which erases installed software (ROS2) and all model modifications.

Create the container **once**:
```bash
docker run -it --name px4_lidar_dev \-e DISPLAY=$DISPLAY \-v /tmp/.X11-unix:/tmp/.X11-unix \px4-gazebo bash
```

This command creates a persistent container instance from the px4-gazebo image. It keeps all installed packages, ROS configuration, and edited simulation files.

## Step 2: How to re-enter the Container in Later Sessions

After the container has been created, **never run** `docker run` again.

To view all containers:
``` bash
docker ps -a
```

If the container status says **Exited** reopen the same environment:

``` bash
docker start px4_lidar_dev
```

Enter the container:
``` bash
docker exec -it px4_lidar_dev bash
```

## Step 3: Install ROS2 Foxy (Inside Container)
PX4 and Gazebo simulate the drone and the LiDAR, but the data stays inside the simulator.
ROS2 is installed so the LiDAR readings can be shared as a ROS topic.

Since the container uses Ubuntu 20.04, the compatible ROS2 version is **Foxy**.

### 3.1 Update system and install required tools
These utilities are needed to securely add a new software repository.

Update the package list:

```bash
apt update
```

Install required tools:
``` bash
apt install -y curl gnupg lsb-release
```

Create the keyring directory:
``` bash
mkdir -p /usr/share/keyrings
```

### 3.2 Add the official ROS2 repository
Download the ROS2 security key and register the ROS2 package source so Ubuntu can trust and download ROS2 software.

Download the ROS2 security key:

``` bash
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Register the ROS2 package source:

``` bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
```

### 3.3 Refresh package list

After adding the repository, update the package database so ROS2 packages become available.

```bash
apt update
```

### 3.4 Install ROS2 and Gazebo-ROS integration

Install the ROS2 core system, visualization tool, and the Gazebo bridge that allows simulated sensors to publish ROS topics.

``` bash
apt install -y ros-foxy-ros-base ros-foxy-rviz2 ros-foxy-gazebo-ros-pkgs
```

### 3.5 Enable ROS2 automatically

Add ROS2 to the shell startup file:
``` bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

Activate ROS2 in the current terminal:
``` bash
source /opt/ros/foxy/setup.bash
```

After this step, ROS2 commands will work in every new terminal session.

## Step 4: Attach the LiDAR to the drone (Iris)

The Iris drone model is generated from a template file.
We will edit this template to add a simulated 3D LiDAR sensor and connect it to ROS2.

### 4.1 Install nano in the container
``` bash
apt update && apt install -y nano
```

### 4.2 Open the model file
``` bash
nano /px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja
```
This file defines the drone’s physical structure and all its sensors.

## 4.3 Find where to insert the LiDAR

Scroll to the end of the file, you will see:
``` code 
 </plugin>
 PASTE HERE
</model>
```

The LiDAR block must be placed between </plugin> and </model> section.

### 4.4 Add the LiDAR sensor
Paste the following block:

``` code
<!-- ===================== 3D LIDAR (Landing) ===================== -->
<link name="lidar_link">
  <pose>0 0 0.12 0 0 0</pose>

  <inertial>
    <mass>0.05</mass>
    <inertia>
      <ixx>0.0001</ixx>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyy>0.0001</iyy>
      <iyz>0</iyz>
      <izz>0.0001</izz>
    </inertia>
  </inertial>

  <sensor name="landing_lidar" type="gpu_ray">
    <pose>0 0 0 0 0 0</pose>
    <update_rate>10</update_rate>
    <visualize>false</visualize>

    <ray>
      <scan>
        <horizontal>
          <samples>1024</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.261799</min_angle>
          <max_angle>0.261799</max_angle>
        </vertical>
      </scan>

      <range>
        <min>0.3</min>
        <max>60.0</max>
        <resolution>0.01</resolution>
      </range>

      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>
      </noise>
    </ray>

    <!-- Gazebo -> ROS2 bridge -->
    <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/iris</namespace>
        <remapping>~/out:=/lidar/scan</remapping>
      </ros>
      <frame_name>lidar_link</frame_name>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>

  </sensor>
</link>

<joint name="lidar_joint" type="fixed">
  <parent>base_link</parent>
  <child>lidar_link</child>
</joint>
<!-- =============================================================== -->
```

You can edit the sensor configuration in this block for your preference.

### 4.5 Save and exit
After pasting:
``` code
CTRL + O
ENTER
CTRL + X
```

## Step 4: Run the Simulation (PX4 SITL + Gazebo Classic Iris)
From inside the container:

```bash 
make px4_sitl gazebo-classic_iris
```

**Do NOT** close the simulation.

## Step 5: Validate ROS Topic + Rate
Wait until Gazebo fully loads and the drone appears.

Open another Ubuntu terminal:

Enter the same container:
``` bash
docker exec -it px4_lidar_dev bash
```
``` bash
source /opt/ros/foxy/setup.bash
```

Confirm topics:
``` bash
ros2 topic list
```

Expected Output:
``` code
/lidar/scan
/clock
/rosout
/parameter_events
```

Confirm publish rate (should be near 10 Hz):

``` bash
ros2 topic hz /lidar/scan
```

The average rate should be approximately 9–10 Hz.
This confirms the LiDAR sensor is publishing correctly and meets the update rate requirement.

## LiDAR Sensor Configuration (Description)

The simulated LiDAR is implemented using the Gazebo `gpu_ray` sensor.

This sensor performs ray-casting in the environment and measures the distance between the drone and surrounding surfaces. The data is exported to ROS2 as a `sensor_msgs/LaserScan` topic.

### Sensor Parameters
- Update rate = 10Hz (Landing does not require high-speed scanning but must be consistent)
- Horizontal Field of View = 360° (Allows detection of landing area in all directions)
- Vertical channels = 16 (Provides vertical surface detection)
- Range Min = 0.3m (Prevents self-detection of drone body)
- Range Max = 60m (Sufficient altitude detection for landing)
- Resolution = 0.01m (Fine terrain height estimation)
- Noise = Gaussian(σ = 0.02) (Simulates real LiDAR measurement error)

This configuration approximates a low-resolution 3D LiDAR suitable for landing surface detection.

