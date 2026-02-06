# PX4 Gazebo Simulation Setup for macOS

This guide walks you through setting up and running PX4 SITL (Software-In-The-Loop) simulation with Gazebo Classic on macOS using Docker and VNC for GUI access.

## Step 1: Install Docker Desktop

1. Download Docker Desktop from [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop)
2. Install and launch Docker Desktop
3. Configure resources for optimal performance:
   - Open **Docker Desktop** → **Settings** → **Resources**
   - Set **Memory** to at least **6 GB** (8 GB recommended)
   - Set **CPUs** to at least **2**
   - Click **Apply & Restart**

## Step 2: Clone the Landing Repository

Clone the repository and navigate to the Gazebo directory:

```bash
git clone https://github.com/LiDRON7/Landing.git
cd Landing/Gazebo
```

The Dockerfile is located at `Landing/Gazebo/Dockerfile`.

## Step 3: Build the Docker Image

From the `Landing/Gazebo` directory, build the Docker image:

```bash
docker build -t px4-gazebo .
```

**Note:** This will take 15-30 minutes depending on your system. The build downloads PX4, Gazebo, and compiles everything with VNC support.

## Step 4: Run the Container with VNC Enabled

Start the container with VNC enabled:

```bash
docker run -it \
  -e ENABLE_VNC=true \
  -p 5900:5900 \
  --memory="6g" \
  --memory-swap="8g" \
  px4-gazebo
```

**What this does:**

- `-e ENABLE_VNC=true` - Enables the VNC server for GUI access
- `-p 5900:5900` - Exposes VNC port to your Mac
- `--memory="6g"` - Allocates 6GB RAM to prevent build failures

You should see output indicating:

```
Starting VNC server...
VNC server started on port 5900 (password: px4vnc)
The VNC desktop is:      <container_id>:0
PORT=5900
```

## Step 5: Connect to VNC

You have three options to connect to the GUI:

### RealVNC Viewer

1. Download from [https://www.realvnc.com/en/connect/download/viewer/](https://www.realvnc.com/en/connect/download/viewer/)
2. Install and open RealVNC Viewer
3. Enter: `localhost:5900`
4. Click **Connect**
5. Password: `px4vnc`

**You should now see a desktop** - this is the Fluxbox window manager running inside Docker. It will appear as a black or gray screen initially.

## Step 6: Launch Gazebo Simulation

You have two ways to start the simulation:

### From Inside VNC

1. In the VNC window, **right-click** on the desktop
2. Navigate to **Applications** → **Shells** → **bash**
3. In the terminal that opens, type:

```bash
cd /px4
make px4_sitl_default gazebo
```

## Step 7: Verify the Simulation is Running

After a few seconds, you should see:

**In the VNC window:**

- Gazebo GUI opens with a 3D world
- An iris quadcopter spawns in the center
- You can rotate the camera and see the drone

**In the terminal:**

```
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s
INFO  [commander] Ready for takeoff!
```

**Note:** You may see some warnings like `Error advertising topic [/asphalt_plane/joint_cmd]` - these are harmless and can be ignored.

## Using the Simulation

### Basic Flight Commands

In the terminal where PX4 is running (you'll see `pxh>` prompt):

```bash
# Take off to 2.5m altitude
commander takeoff

# Check vehicle status
commander status

# Land the drone
commander land

# Disarm motors
commander disarm
```

### Gazebo Camera Controls

- **Left-click + drag**: Rotate camera view
- **Right-click + drag**: Pan camera
- **Scroll wheel**: Zoom in/out
- **Ctrl + Left-click**: Select objects in the world

### Adjusting Simulation Speed

In Gazebo, at the bottom of the window, you can adjust the real-time factor to speed up or slow down the simulation.

## Stopping and Restarting

### Stop Gazebo Simulation

In the terminal running Gazebo, press `Ctrl + C`

### Exit the Container

Type `exit` in the container's bash prompt

### Stop the Container

```bash
docker stop <container_id>
```

### Restart the Same Container Later

```bash
# Start the stopped container
docker start <container_id>

# Attach to it
docker attach <container_id>

# Or execute commands directly
docker exec -it <container_id> bash -c "cd /px4 && make px4_sitl_default gazebo"
```

### Remove Container Completely

```bash
docker rm <container_id>
```

## Troubleshooting

### VNC Shows Only Black Screen

**This is normal!** The black/gray screen is the empty Fluxbox desktop. Launch Gazebo as described in Step 6.

### "Container killed" or Out of Memory During Build

**Solution:**

1. Increase Docker memory to 8 GB in Docker Desktop settings
2. Rebuild the image

### VNC Connection Refused

**Check if container is running:**

```bash
docker ps
```

**Restart the container:**

```bash
docker restart <container_id>
```

### Gazebo Window Doesn't Appear

**Ensure you're in the correct directory:**

```bash
docker exec -it <container_id> bash
cd /px4
pwd  # Should show /px4
make px4_sitl_default gazebo
```

### Low Frame Rate in Gazebo

Software rendering is slower than GPU acceleration. This is expected on macOS with Docker. Tips:

- Close other applications to free up resources
- Use simpler Gazebo worlds
- Reduce real-time factor in Gazebo

### Can't Find Container ID

```bash
# List all running containers
docker ps

# List all containers (including stopped)
docker ps -a
```

## Quick Command Reference

```bash
# Build image
docker build -t px4-gazebo .

# Run with VNC
docker run -it -e ENABLE_VNC=true -p 5900:5900 --memory="6g" px4-gazebo

# List containers
docker ps

# Execute command in running container
docker exec -it <container_id> bash -c "cd /px4 && make px4_sitl_default gazebo"

# Stop container
docker stop <container_id>

# Remove container
docker rm <container_id>

# Remove all stopped containers
docker container prune

# Remove image
docker rmi px4-gazebo
```

## Resources

- **PX4 Documentation:** [https://docs.px4.io/](https://docs.px4.io/)
- **PX4 User Guide:** [https://docs.px4.io/main/en/](https://docs.px4.io/main/en/)
- **Gazebo Classic:** [https://classic.gazebosim.org/](https://classic.gazebosim.org/)
- **PX4 GitHub:** [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)

## Notes

- The VNC password is hardcoded as `px4vnc` in the Dockerfile
- The simulation runs in software rendering mode (no GPU acceleration)
- All data in the container is lost when removed - mount volumes if you need persistence
- This setup is specifically for macOS - Windows and Linux users should use native X11 forwarding
