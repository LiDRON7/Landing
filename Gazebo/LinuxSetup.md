# PX4 Gazebo Simulation Setup for Linux

This guide shows you how to run PX4 SITL (Software-In-The-Loop) simulation with Gazebo Classic on Linux using Docker with native X11 forwarding.

## Step 1: Install Docker

### Ubuntu/Debian

```bash
# Update package list
sudo apt-get update

# Install dependencies
sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker's official GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Set up stable repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io

# Add your user to docker group (to run without sudo)
sudo usermod -aG docker $USER

# Log out and log back in for group changes to take effect
```
### Ubuntu/Debian

```bash
sudo systemctl start docker
```
### Fedora/RHEL

```bash
sudo dnf -y install dnf-plugins-core
sudo dnf config-manager --add-repo https://download.docker.com/linux/fedora/docker-ce.repo
sudo dnf install -y docker-ce docker-ce-cli containerd.io
sudo systemctl start docker
sudo systemctl enable docker
sudo usermod -aG docker $USER
```

### Arch Linux

```bash
sudo pacman -S docker
sudo systemctl start docker
sudo systemctl enable docker
sudo usermod -aG docker $USER
```

**After installation:** Log out and log back in for group changes to take effect.

## Step 2: Clone the Repository

```bash
git clone <repository-url> Landing
cd Landing/Gazebo
```

## Step 3: Build the Docker Image

From the `Landing/Gazebo` directory:

```bash
docker build -t px4-gazebo .
```

**If not building** make sure you built docker
**Expected build time:** 20-40 minutes depending on your system.

## Step 4: Enable X11 Forwarding

Allow Docker to access your X server:

```bash
xhost +local:docker
```

> **Note:** This grants Docker containers access to your display.
## Step 5: Run the Container

Start the container with X11 forwarding:

```bash
docker run -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --memory="16g" \
  px4-gazebo
```

**What each flag does:**

- `--env DISPLAY=$DISPLAY` - Forwards your display to the container
- `--env QT_X11_NO_MITSHM=1` - Fixes Qt rendering issues
- `--volume /tmp/.X11-unix:/tmp/.X11-unix:rw` - Mounts X11 socket
- `--memory="16g"` - Limits container to 8GB RAM
You should get a bash prompt inside the container:

```
root@<container_id>:/px4#
```

## Step 6: Launch Gazebo

From inside the container:

```bash
cd /px4
make px4_sitl_default gazebo
```
**IMPORTANT** If this step fails, its probably because you put either too much ram or too little when starting the container, go back to step 5 and adjust the memory ammount

## Step 7: Verify Simulation

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

Press `Ctrl + C` in the terminal running Gazebo

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


## Troubleshooting

### "cannot open display" Error

**Solution:**

```bash
# Re-enable X11 forwarding
xhost +local:docker

# Make sure DISPLAY is set
echo $DISPLAY

# Run container with correct DISPLAY
docker run -it \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  px4-gazebo
```

### Build Fails with "Killed" Error

**Cause:** Out of memory during compilation

**Solution:** Increase available RAM or add swap:

```bash
# Check available memory
free -h

# If needed, add swap space
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### Gazebo Crashes or Black Screen

**Solution:** Update graphics drivers and try software rendering:

```bash
docker run -it \
  --env DISPLAY=$DISPLAY \
  --env LIBGL_ALWAYS_SOFTWARE=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  px4-gazebo
```

### Permission Denied Errors

**Solution:** Make sure you're in the docker group:

```bash
# Add yourself to docker group
sudo usermod -aG docker $USER

# Log out and log back in, then test
docker run hello-world
```

### GPU Acceleration Not Working

For NVIDIA GPUs, use nvidia-docker:

```bash
# Install nvidia-docker2
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# Run with GPU support
docker run -it \
  --gpus all \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  px4-gazebo
```


## Resources

- **PX4 Documentation:** [https://docs.px4.io/](https://docs.px4.io/)
- **PX4 Development Guide:** [https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- **Gazebo Classic:** [https://classic.gazebosim.org/](https://classic.gazebosim.org/)
- **QGroundControl:** [https://qgroundcontrol.com/](https://qgroundcontrol.com/)
