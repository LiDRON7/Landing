# PX4 Gazebo Simulation with Docker and VNC on macOS

This guide walks you through setting up and running PX4 SITL (Software-In-The-Loop) simulation with Gazebo Classic in a Docker container on macOS, with GUI access via VNC.

## Step 1: Install Docker Desktop

1. Download Docker Desktop from [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop)
2. Install and launch Docker Desktop
3. Configure resources:
   - Open **Docker Desktop** → **Settings** → **Resources**
   - Set **Memory** to at least **6 GB** (8 GB recommended)
   - Set **CPUs** to at least **2**
   - Click **Apply & Restart**

## Step 2: Clone the Landing Repository

Clone the repository containing the Dockerfile:

```bash
git clone  Landing

cd Landing/Gazebo
```

The Dockerfile is located at `Landing/Gazebo/Dockerfile`.

## Step 3: Build the Docker Image

From the `Landing/Gazebo` directory, build the Docker image:

```bash
docker build -t px4-gazebo .
```

**Note:** This will take 15-30 minutes depending on your system. The build process downloads and compiles PX4 and all its dependencies.

## Step 4: Run the Container

Start the container with VNC port exposed:

```bash
docker run -it \
  -p 5900:5900 \
  --memory="6g" \
  --memory-swap="8g" \
  px4-gazebo
```

You should see output indicating that:

- Xvfb (virtual display) is running
- Fluxbox (window manager) is running
- x11vnc server is listening on port 5900

Look for a message like:

```
The VNC desktop is:      <container_id>:0
PORT=5900
```

## Step 5: Connect to VNC

### Using RealVNC Viewer (Recommended)

1. Download from: [https://www.realvnc.com/en/connect/download/viewer/](https://www.realvnc.com/en/connect/download/viewer/)
2. Install and open RealVNC Viewer
3. Enter VNC Server address: `localhost:5900`
4. Click **Connect**
5. Enter password: `px4vnc`

You should see a black or gray desktop - this is the Fluxbox window manager.

## Step 6: Launch Gazebo Simulation

### From Inside VNC

1. Right-click on the VNC desktop
2. Navigate to **Applications** → **Shells** → **bash**
3. In the terminal that opens, run:

```bash
make px4_sitl_default gazebo
```

## Step 7: Verify the Simulation

After running the command, you should see:

1. Gazebo GUI window appears in the VNC viewer
2. A quadcopter (iris) model spawns in the simulation
3. Terminal output shows PX4 is running with messages like:
   ```
   INFO  [mavlink] mode: Normal, data rate: 4000000 B/s
   INFO  [commander] Ready for takeoff!
   ```

## Using the Simulation

### Basic PX4 Commands

In the terminal where PX4 is running, you can use the `pxh>` console:

```bash
# Arm the drone
commander takeoff

# Check status
commander status

# Land the drone
commander land
```

### Gazebo Controls

- **Left-click + drag**: Rotate view
- **Right-click + drag**: Pan view
- **Scroll wheel**: Zoom in/out
- **Ctrl + Left-click**: Select objects

## Troubleshooting

### Container Runs Out of Memory During Build

**Symptoms:**

```
c++: fatal error: Killed signal terminated program cc1plus
```

**Solution:** Increase Docker memory allocation in Docker Desktop settings to 8 GB.

### VNC Shows Black Screen

**Normal behavior:** The black/gray screen is the Fluxbox desktop. Launch applications as described in Step 6.

### Gazebo Doesn't Start

**Check:** Ensure you're in the `/px4` directory:

```bash
cd /px4
make px4_sitl_default gazebo
```

### "Cannot connect to display" Error

**Solution:** Restart the container:

```bash
docker stop <container_id>
docker start <container_id>
docker attach <container_id>
```

### Low Frame Rate in Gazebo

This is normal with software rendering. For better performance:

- Close other applications
- Reduce Gazebo's real-time factor
- Use a simpler world/model

## Stopping and Restarting

### Stop the Simulation

Press `Ctrl + C` in the terminal running Gazebo.

### Exit the Container

Type `exit` in the container terminal.

### Stop the Container

```bash
docker stop <container_id>
```

### Restart an Existing Container

```bash
docker start <container_id>
docker attach <container_id>
```

### Remove the Container

```bash
docker rm <container_id>
```

## Advanced Usage

### Running Headless (No GUI)

If you only need to test flight controllers without visualization:

```bash
HEADLESS=1 make px4_sitl_default gazebo
```

### Using Different PX4 Versions

Change the git clone command in the Dockerfile:

```dockerfile
RUN git clone --branch v1.15.0 https://github.com/PX4/PX4-Autopilot.git /px4
```

### Connecting QGroundControl

QGroundControl can connect to the simulated drone:

1. Download QGroundControl for macOS
2. It should auto-detect the PX4 simulation on UDP port 14550
3. If not, add a manual connection to `localhost:14550`

## Useful Commands Reference

```bash
# Build the image
docker build -t px4-gazebo .

# Run container
docker run -it -p 5900:5900 --memory="6g" px4-gazebo

# List running containers
docker ps

# Execute command in running container
docker exec -it <container_id> <command>

# View container logs
docker logs <container_id>

# Stop container
docker stop <container_id>

# Remove container
docker rm <container_id>

# Remove image
docker rmi px4-gazebo
```

## Resources

- **PX4 Documentation:** [https://docs.px4.io/](https://docs.px4.io/)
- **Gazebo Documentation:** [https://gazebosim.org/](https://gazebosim.org/)
- **PX4 GitHub:** [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- **QGroundControl:** [https://qgroundcontrol.com/](https://qgroundcontrol.com/)
