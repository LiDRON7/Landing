FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# installing dependencies
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    gnupg2 \
    software-properties-common \
    x11-apps \
    build-essential \
    curl \
    git \
    sudo \
    vim \
    tzdata \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# setting up gazebo repo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y gazebo11 libgazebo11-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# installing px4 dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-jinja2 \
    python3-numpy \
    python3-empy \
    python3-toml \
    python3-yaml \
    python-is-python3 \
    protobuf-compiler \
    libeigen3-dev \
    genromfs \
    libxml2-dev \
    libxslt1-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install VNC and virtual display (for macOS)
RUN apt-get update && apt-get install -y \
    xvfb \
    x11vnc \
    fluxbox \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# upgrading python tools
RUN pip3 install "pip<24" setuptools wheel

# cloning px4 source code from stable
RUN git clone --branch v1.14.3 --recursive https://github.com/PX4/PX4-Autopilot.git /px4

# installing px4 simulation dependencies
RUN /px4/Tools/setup/ubuntu.sh --no-nuttx

# building px4 sitl with gazebo classic with limited parallelism to prevent OOM
WORKDIR /px4
RUN MAKEFLAGS="-j1" make px4_sitl_default

# Setup VNC (only used when ENABLE_VNC=true)
ENV DISPLAY=:99
EXPOSE 5900

# Create flexible startup script
RUN echo '#!/bin/bash\n\
if [ "$ENABLE_VNC" = "true" ]; then\n\
    echo "Starting VNC server..."\n\
    Xvfb :99 -screen 0 1280x1024x24 &\n\
    sleep 2\n\
    fluxbox &\n\
    x11vnc -display :99 -forever -shared -rfbport 5900 -passwd px4vnc &\n\
    echo "VNC server started on port 5900 (password: px4vnc)"\n\
fi\n\
exec "$@"' > /start.sh && chmod +x /start.sh

ENTRYPOINT ["/start.sh"]
CMD ["bash"]
