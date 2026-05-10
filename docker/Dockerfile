FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    ccache \
    gdb \
    lsb-release \
    software-properties-common \
    git \
    curl \
    wget \
    tmux \
    xz-utils \
    zip \
    unzip \
    file \
    rsync \
    ca-certificates \
    python3-dev \
    python3-empy \
    python3-jinja2 \
    python3-jsonschema \
    python3-pyserial \
    python3-toml \
    python3-numpy \
    python3-yaml \
    python3-kconfiglib \
    python3-packaging \
    python3-pip \
    python3-opencv \
    python3-colcon-common-extensions \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-topic-tools \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

RUN pip3 install --no-cache-dir mavsdk

WORKDIR /work

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/start_aio_stack.sh /usr/local/bin/start_aio_stack.sh
RUN chmod +x /entrypoint.sh
RUN chmod +x /usr/local/bin/start_aio_stack.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
