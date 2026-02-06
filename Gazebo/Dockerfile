FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

#installing dependencies 
RUN apt update && apt install -y \
	wget \
	lsb-release \
	gnupg2 \
	software-properties-common \
	x11-apps \
	build-essential \
	curl \
	&& apt clean

#setting up gazebo repo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt update \
    && apt install -y gazebo11 \
    && apt clean

#installing px4 dependencies 
RUN apt update && apt install -y \
	sudo \
	python3 \
	python3-pip \
	python3-jinja2 \
	python3-numpy \
	python3-empy \
	python3-toml \
	python3-yaml \
	python-is-python3 \
	git \
	vim \
	tzdata \
	protobuf-compiler \
	libeigen3-dev \
	genromfs \
	libxml2-dev \
	libxslt1-dev \
	python3-dev \
	&& apt clean

# Install VNC and virtual display
RUN apt update && apt install -y \
	xvfb \
	x11vnc \
	fluxbox \
	mesa-utils \
	libgl1-mesa-dri \
	libgl1-mesa-glx \
	&& apt clean

#cloning px4 source code from stable 
RUN git clone --branch v1.14.3 https://github.com/PX4/PX4-Autopilot.git /px4

#installing px4 simulation dependencies
RUN /px4/Tools/setup/ubuntu.sh --no-nuttx

#building px4 sitl with gazebo classic
WORKDIR /px4
RUN make px4_sitl_default

# Setup VNC
ENV DISPLAY=:99
EXPOSE 5900

# Create startup script
RUN echo '#!/bin/bash\n\
Xvfb :99 -screen 0 1280x1024x24 &\n\
sleep 2\n\
fluxbox &\n\
x11vnc -display :99 -forever -shared -rfbport 5900 -passwd px4vnc &\n\
exec "$@"' > /start.sh && chmod +x /start.sh

ENTRYPOINT ["/start.sh"]
CMD ["bash"]
