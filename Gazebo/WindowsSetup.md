# PX4 Gazebo Simulation Setup for Windows

This guide shows you how to run PX4 SITL (Software-In-The-Loop) simulation with Gazebo Classic on Windows using Docker Desktop with WSLg (Windows Subsystem for Linux with GUI support).

## Prerequisites

- Windows 10 version 2004+ (Build 19041+) or Windows 11
- WSL 2 installed
- At least 16 GB RAM (12 GB minimum)

## Step 1: Install WSL 2

Open **PowerShell as Administrator** and run:

```powershell
# Install WSL 2 with Ubuntu
wsl --install -d Ubuntu-20.04

# After installation, restart your computer
```

After reboot, Ubuntu will open automatically. Set up your username and password.

## Step 2: Update WSL

In PowerShell (as Administrator):

```powershell
# Update WSL to the latest version (required for WSLg GUI support)
wsl --update

# Verify WSL version
wsl --version
```

You should see WSLg version listed.

## Step 3: Install Docker Desktop

1. Download Docker Desktop from [https://www.docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop)
2. Install Docker Desktop
3. During installation, ensure **"Use WSL 2 instead of Hyper-V"** is checked
4. After installation, open **Docker Desktop**
5. Go to **Settings** → **General** → Enable **"Use the WSL 2 based engine"**
6. Go to **Settings** → **Resources** → **WSL Integration**
   - Enable integration with your Ubuntu-20.04 distro
7. Go to **Settings** → **Resources** → **Advanced**
   - Set **Memory** to at least **8 GB** (12 GB recommended)
   - Set **CPUs** to at least **2**
8. Click **Apply & Restart**

## Step 4: Set Up Ubuntu in WSL

Open **Ubuntu** from the Start menu (or run `wsl` in PowerShell):

```bash
# Update package list
sudo apt-get update
sudo apt-get upgrade -y

# Install required tools
sudo apt-get install -y git
```

## Step 5: Clone the Repository

In your Ubuntu WSL terminal:

```bash
# Navigate to home directory
cd ~

# Clone the repository
git clone <repository-url> Landing
cd Landing/Gazebo
```

## Step 6: Build the Docker Image

From the `Landing/Gazebo` directory:

```bash
docker build -t px4-gazebo .
```

**Expected build time:** 20-40 minutes depending on your system.

> **Note:** This runs inside WSL, using Docker Desktop's backend.

## Step 7: Run the Container with WSLg

WSLg provides native GUI support, so no VNC needed!

```bash
docker run -it \
  --env DISPLAY=$DISPLAY \
  --env WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  --env PULSE_SERVER=$PULSE_SERVER \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume /mnt/wslg:/mnt/wslg:rw \
  --memory="8g" \
  px4-gazebo
```

**What each flag does:**

- `--env DISPLAY=$DISPLAY` - Forwards display to WSLg
- `--volume /mnt/wslg:/mnt/wslg:rw` - Mounts WSLg sockets
- `--memory="8g"` - Limits container to 8GB RAM

You should see a bash prompt:

```
root@<container_id>:/px4#
```

## Step 8: Launch Gazebo

From inside the container:

```bash
cd /px4
make px4_sitl_default gazebo
```

**Gazebo will launch in a native Windows window!** WSLg handles the GUI automatically.

## Step 9: Verify Simulation

After 10-20 seconds:

**Gazebo window shows:**

- 3D world environment
- Iris quadcopter model

**Terminal shows:**

```
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s
INFO  [commander] Ready for takeoff!
```

## Using the Simulation

### Basic Flight Commands

In the terminal with `pxh>` prompt:

```bash
# Take off
commander takeoff

# Check status
commander status

# Land
commander land

# Disarm
commander disarm
```

### Gazebo Controls

- **Left-click + drag** - Rotate camera
- **Right-click + drag** - Pan camera
- **Scroll wheel** - Zoom
- **Ctrl + Left-click** - Select objects

## Stopping and Restarting

### Stop Gazebo

Press `Ctrl + C` in the WSL terminal running Gazebo

### Exit Container

```bash
exit
```

### Stop Container

```bash
docker stop <container_id>
```

### Find Container ID

```bash
docker ps        # Running containers
docker ps -a     # All containers
```

### Restart Existing Container

```bash
# Start container
docker start <container_id>

# Attach to it
docker attach <container_id>

# Or run Gazebo directly
docker exec -it <container_id> bash -c "cd /px4 && make px4_sitl_default gazebo"
```

## Troubleshooting

### WSLg Not Working / No GUI Appears

**Check WSLg is installed:**

```bash
# In WSL terminal
echo $DISPLAY
ls /mnt/wslg
```

If empty, update WSL:

```powershell
# In PowerShell as Administrator
wsl --update
wsl --shutdown
```

Then restart WSL.

### Build Fails with "Killed" Error

**Cause:** Out of memory

**Solution:**

1. Close other applications
2. Increase Docker memory in Docker Desktop settings to 12 GB
3. Add swap in WSL:

```bash
# In WSL
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### Docker Command Not Found in WSL

**Solution:**

```bash
# Ensure Docker Desktop is running
# In WSL, verify integration:
docker --version

# If not found, restart Docker Desktop and enable WSL integration:
# Docker Desktop → Settings → Resources → WSL Integration → Enable Ubuntu-20.04
```

### Gazebo Window is Blank/Black

**Try software rendering:**

```bash
docker run -it \
  --env DISPLAY=$DISPLAY \
  --env LIBGL_ALWAYS_SOFTWARE=1 \
  --volume /mnt/wslg:/mnt/wslg:rw \
  px4-gazebo
```

### Slow Performance

**Tips:**

1. Store files in WSL filesystem (`~/Landing`), not Windows filesystem (`/mnt/c/...`)
2. Close unnecessary Windows applications
3. Increase Docker resources in Docker Desktop settings
4. Disable Windows Defender real-time scanning for WSL directories (at your own risk)

### "Cannot connect to Docker daemon" Error

**Solution:**

1. Make sure Docker Desktop is running
2. In Docker Desktop settings, enable WSL integration for Ubuntu-20.04
3. Restart WSL:

```powershell
wsl --shutdown
```

Then open Ubuntu again.

## QGroundControl for Windows

Download and install QGroundControl for Windows:

1. Download from [https://qgroundcontrol.com/](https://qgroundcontrol.com/)
2. Install and launch
3. It should auto-detect the simulation on UDP port 14550

> **Note:** You may need to allow QGroundControl through Windows Firewall.

## Accessing Files Between WSL and Windows

Your WSL files are accessible from Windows File Explorer:

```
\\wsl$\Ubuntu-20.04\home\<your-username>\Landing
```

You can also access Windows files from WSL:

```bash
cd /mnt/c/Users/<your-windows-username>/
```

> **Performance Tip:** Keep Docker files in WSL filesystem (`~`), not in `/mnt/c/`

## Alternative: Use VNC (If WSLg Doesn't Work)

If WSLg isn't working, you can use VNC like macOS:

```bash
docker run -it \
  -e ENABLE_VNC=true \
  -p 5900:5900 \
  --memory="8g" \
  px4-gazebo
```

Then download a VNC viewer for Windows:

- [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/)
- [TightVNC](https://www.tightvnc.com/)

Connect to `localhost:5900` with password `px4vnc`.

## Quick Reference

```bash
# Build image (in WSL)
docker build -t px4-gazebo .

# Run container with WSLg
docker run -it \
  --env DISPLAY=$DISPLAY \
  --volume /mnt/wslg:/mnt/wslg:rw \
  --memory="8g" \
  px4-gazebo

# Launch Gazebo (inside container)
cd /px4 && make px4_sitl_default gazebo

# List containers
docker ps

# Stop container
docker stop <container_id>

# Remove container
docker rm <container_id>

# Clean up
docker container prune
```

## PowerShell Commands

```powershell
# Update WSL
wsl --update

# Shutdown WSL (to apply updates)
wsl --shutdown

# List WSL distros
wsl --list --verbose

# Set default WSL version
wsl --set-default-version 2

# Check WSL version
wsl --version
```

## Resources

- **PX4 Documentation:** [https://docs.px4.io/](https://docs.px4.io/)
- **WSL Documentation:** [https://docs.microsoft.com/en-us/windows/wsl/](https://docs.microsoft.com/en-us/windows/wsl/)
- **WSLg Guide:** [https://github.com/microsoft/wslg](https://github.com/microsoft/wslg)
- **Docker Desktop for Windows:** [https://docs.docker.com/desktop/windows/wsl/](https://docs.docker.com/desktop/windows/wsl/)
- **QGroundControl:** [https://qgroundcontrol.com/](https://qgroundcontrol.com/)
