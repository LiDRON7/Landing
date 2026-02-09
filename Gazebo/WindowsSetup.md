# Installation Guide To Run Gazebo+PX4 In A Docker Container for Windows

## Overview

This installation guide aims to create a shared and reproducible workspace where all team members can run simulations using the software developed across different divisions. This environment allows teams to test, integrate, and validate their work with other divisions’ software in a consistent setup.

The system is built using **Docker** in combination with **Windows Subsystem for Linux (WSL2)** running **Ubuntu** with a graphical interface. The Ubuntu terminal is used to configure and run Docker, which launches the **Gazebo + PX4** simulation environment.

## Step 1: Install WSL2 (Windows Subsystem for Linux)

Open **PowerShell as Administrator** and run:

```powershell
wsl --install
```

This command installs the WSL2 kernel and a default Linux distribution. When prompted, select **Ubuntu** as the distribution. You will be asked to restart your system once the installation completes.

After restarting, open the **Ubuntu terminal** for the first time. You will be prompted to create a Linux username and password, which will be used within the Ubuntu environment.

To launch the Ubuntu terminal, open **Command Prompt** (or Windows Terminal) and select **Ubuntu** from the dropdown menu (▾) next to the + button.

### Verify WSL2 installation

Run the following command in **PowerShell**:

```powershell
wsl -l -v
```

Expected output:

```text
  NAME      STATE   VERSION
* Ubuntu    Running 2
```

If Ubuntu shows **VERSION 1**, run

```powershell
wsl --set-version Ubuntu 2
```

### Optional: Manual Ubuntu Installation

If Ubuntu does not install automatically when running `wsl --install`, you can install it manually using:

```powershell
wsl --install -d Ubuntu
```

### Optional: Verify WSL2 and WSLg Installation

After installing WSL2 and Ubuntu, you can verify the versions of the installed WSL components by running the following command in **PowerShell**:

```powershell
wsl --version
```

This command displays the installed versions of WSL, the Linux kernel, and WSLg (Windows Subsystem for Linux GUI support).

WSLg is included by default with WSL2 and allows Linux GUI applications—such as Gazebo + PX4—to run without additional configuration.

To verify that GUI support is working, run the following command in the **Ubuntu terminal**:

```bash
echo $DISPLAY
```

You should see output similar to:

```
:0
```

## Step 2: Install and Configure Docker Desktop for WSL2

1. Download Docker Desktop either from the Microsoft Store or from the official Docker website (Windows amd64):
   https://www.docker.com/products/docker-desktop/

2. Once downloaded, open **Docker Desktop**.

3. In Docker Desktop, navigate to:
   **Settings → General**

4. Ensure the following option is enabled:
   - **Use the WSL 2 based engine**

   > Note: This option may already be enabled by default on first launch.

5. Click **Apply & Restart** if any changes were made.

## Step 3: Enable Docker WSL Integration

1. In Docker Desktop, go to:
   **Settings → Resources → Scroll up → WSL Integration**

2. Enable:
   - **Enable integration with my default WSL distro**
   - **Ubuntu**

3. Click **Apply & Restart**

To verify Docker inside WSL, run the following command in the **Ubuntu terminal**:

```bash
docker --version
```

You should see output similar to:

```text
Docker version 29.2.0, build 0b9d198
```

Now run:

```bash
docker run hello-world
```

It should print a success message and demonstrates that Docker is working correctly.

```text
Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (amd64)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/
```

## Step 4: Create the PX4 + Gazebo Docker Project Directory

In this step, you will create a directory that will store the Docker configuration files required to build and run the PX4 + Gazebo simulation.

Open the **Ubuntu terminal** and create a new directory called gazebo-docker:

```bash
mkdir gazebo-docker
```

## Step 5: Obtain the Dockerfile in GitHub

Download the Dockerfile from the project repository:

1. Navigate to the **Landing** repository.

2. Open the file named `Dockerfile`.

3. Click on **Raw** in the right corner.

4. Right-click anywhere on the page and select **Save As**.

5. Save the file with the name **Dockerfile**

6. Go to your **Downloads** folder and rename your Dockerfile to have no file extension (.txt).

> ⚠️ Ensure the file is saved exactly as `Dockerfile` and **not** `Dockerfile.txt`.

## Step 6: Move the Dockerfile into the Project Directory

If the Dockerfile was downloaded to your Windows **Downloads** folder, move it into the project directory using the Ubuntu terminal.

Navigate to the Windows Downloads folder:

```bash
cd /mnt/c/Users/your-username/Downloads
```

> Note: You can also cd files separately.

Move the Dockerfile into the gazebo-docker directory:

```bash
mv Dockerfile ~/gazebo-docker/.
```

Change directory to Home:

```bash
cd ~
```

Change directory to gazebo-docker:

```bash
cd gazebo-docker
```

Verify that the Dockerfile was moved successfully:

```bash
ls
```

Expected output:

```text
Dockerfile
```

## Step 7: Building and Running the Gazebo + PX4 Container

Once all previous steps are complete and the Dockerfile is inside the `gazebo-docker` directory, you are ready to build the Docker image for the first time.

> ⚠️ This step may take a significant amount of time (20–40 minutes) depending on your system specifications, as all dependencies will be installed and PX4 will be built.

Open the **Ubuntu terminal**, navigate to the `gazebo-docker` directory, and run:

```bash
docker build -t px4-gazebo .
```

## Run the PX4 + Gazebo Docker Container

After the image has been built successfully, run the container using the following command:

```bash
docker run -it --rm \-e DISPLAY=$DISPLAY \-v /tmp/.X11-unix:/tmp/.X11-unix \px4-gazebo bash
```

## Step 8: Start PX4 with Gazebo

After the Docker container is running, you will be inside the container’s shell.

Navigate to the PX4 directory:

```bash
cd /px4
```

Start the PX4 simulation with Gazebo:

```bash
make px4_sitl_default gazebo
```

During the first run, this step may take several minutes as PX4 finishes building and setting up the simulation environment.

Once the build completes, PX4 startup logs will begin appearing in the terminal, indicating that the simulator has started successfully.

## Stopping the Simulation

To stop the PX4 simulation, press:

```text
Ctrl + C
```

## To exit container

```bash
exit
```
