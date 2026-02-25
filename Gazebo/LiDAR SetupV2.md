
# PX4 Gazebo — rplidar (landing-optimized)


---

## Prerequisites

You should have completed the Docker and Gazebo installation guide for your operating system. https://github.com/LiDRON7/Aterrizaje

## 1) Run the PX4 Gazebo container

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  px4-gazebo bash
```

## 2) Start SITL using the rplidar model

From `/px4` inside the container:

```bash
cd /px4
make px4_sitl gazebo-classic_iris_rplidar
```

If you see blue rays in Gazebo, the sensor is publishing.

## 3) Locate the model file

```bash
cd /px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/rplidar
ls
```

Main file: `model.sdf`

## 4) Desired sensor parameters

We want to modify the LiDAR sensor parameters to better fit our simulation needs.
- Range: 0.1 → 30 m
- Vertical beams: 16
- Range resolution: 0.01 m
- Noise: Gaussian (stddev 0.01)
- Update rate: 10 Hz

## 5) Create `model_new.sdf`

Create `model_new.sdf` in the `rplidar` model folder. Use the XML below. This XML will change the parameter of the sensor.

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="rplidar">
    <link name="link">
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.19</mass>
        <inertia>
          <ixx>4.15e-6</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>2.407e-6</iyy>
          <iyz>0</iyz>
          <izz>2.407e-6</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.02 0.05 0.05</size>
          </box>
        </geometry>
      </visual>
      <sensor name="laser" type="ray">
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>0.01</resolution>
              <min_angle>-3.1415</min_angle>
              <max_angle>3.1415</max_angle>
            </horizontal>
            <vertical>
              <samples>16</samples>
              <min_angle>-0.2</min_angle>
              <max_angle>0.2</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="laser" filename="libRayPlugin.so" />
        <plugin name="gazebo_ros_head_rplidar_controller" filename="libgazebo_ros_laser.so">
          <topicName>laser/scan</topicName>
          <frameName>rplidar_link</frameName>
        </plugin>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <visualize>true</visualize>
      </sensor>
    </link>
  </model>
</sdf>
```

Write it safely (prevent variable expansion):

```bash
cat > model_new.sdf <<'EOL'
<PASTE THE XML BLOCK FROM ABOVE HERE>
EOL
```

## 6) Backup and swap the files

This sequence of commands backs up the original `model.sdf` file, replaces it with a new version `model_new.sdf`, and then lists the details of both the new and backup files to verify the change.

```bash
mv model.sdf model_backup.sdf
mv model_new.sdf model.sdf
ls -la model.sdf model_backup.sdf
```

## 7) Verify the LiDAR output

It is recommended to close Gazebo/terminal and reopen it after making changes. If you reopen it without closing, a new world instance will be created, which may not include the recent updates.

Open Gazebo again.

```bash
make px4_sitl gazebo-classic_iris_rplidar
```


List topics:

```bash
gz topic -l
```

You should see something like this:
```bash
/gazebo/default/iris_rplidar/laser/scan
```
This commands lists all active ROS/Gazebo topics to verify that the LiDAR sensor is correctly publishing its data.

## Sensor Detection Verification

We spawn a test box in the Gazebo world and then check the LiDAR sensor topic to confirm that the sensor is correctly detecting objects and publishing distance measurements

Spawn Box:
```bash
gz model --spawn-file=/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/small_box/model.sdf --model-name box1 -x 2 -y 0 -z 0
```
Now, lets check if the sensor its detecting the box.

```bash
gz topic -e /gazebo/default/iris_rplidar/rplidar/link/laser/scan
```

You should see something like this:
```bash
.....
ranges: 0.9
ranges: 1.1
ranges: 0.8
.....
```
If you've made it this far, it means the lidar sensor is working as it should.