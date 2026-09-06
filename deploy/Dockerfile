# syntax=docker/dockerfile:1
FROM ros:humble-ros-base-jammy AS common

ARG TARGETARCH
ARG UNITREE_SDK2_REVISION=65e19c625b76b697f94e0b953e296ecc0ce7b4a1
ENV DEBIAN_FRONTEND=noninteractive
ENV CONDA_DIR=/opt/conda
ENV PATH=/opt/conda/bin:${PATH}

RUN apt-get update && apt-get install -y --no-install-recommends \
      bash build-essential ca-certificates cmake curl git iproute2 iputils-ping \
      openssh-client sshpass tar usbutils zsh \
      libboost-all-dev libeigen3-dev libfmt-dev libspdlog-dev libyaml-cpp-dev \
      python3-colcon-common-extensions python3-rosdep \
      ros-humble-ament-cmake-pytest \
      ros-humble-controller-manager \
      ros-humble-control-msgs \
      ros-humble-gripper-controllers \
      ros-humble-joint-state-broadcaster \
      ros-humble-joint-state-publisher-gui \
      ros-humble-joint-trajectory-controller \
      ros-humble-moveit \
      ros-humble-realsense2-camera \
      ros-humble-robot-state-publisher \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-rviz2 \
      ros-humble-tf-transformations \
      ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN case "${TARGETARCH}" in \
      amd64) miniforge_arch=x86_64 ;; \
      arm64) miniforge_arch=aarch64 ;; \
      *) echo "Unsupported Docker architecture: ${TARGETARCH}" >&2; exit 2 ;; \
    esac \
    && curl -fsSL -o /tmp/miniforge.sh \
      "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-${miniforge_arch}.sh" \
    && bash /tmp/miniforge.sh -b -p "${CONDA_DIR}" \
    && rm /tmp/miniforge.sh

COPY deploy/conda-environment.yml /tmp/d1-conda-environment-amd64.yml
COPY deploy/conda-environment-arm64.yml /tmp/d1-conda-environment-arm64.yml
RUN if [ "${TARGETARCH}" = arm64 ]; then \
      cp /tmp/d1-conda-environment-arm64.yml /tmp/d1-conda-environment.yml; \
    else \
      cp /tmp/d1-conda-environment-amd64.yml /tmp/d1-conda-environment.yml; \
    fi \
    && conda env create -f /tmp/d1-conda-environment.yml \
    && conda clean -afy \
    && rm /tmp/d1-conda-environment*.yml

# Pin the host-side Unitree transport ABI. The SDK build installs its matching
# Cyclone DDS C/C++ libraries beside libunitree_sdk2 under /usr/local.
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git /tmp/unitree_sdk2 \
    && git -C /tmp/unitree_sdk2 checkout "${UNITREE_SDK2_REVISION}" \
    && cmake -S /tmp/unitree_sdk2 -B /tmp/unitree_sdk2/build \
      -DBUILD_EXAMPLES=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/unitree_sdk2/build -j"$(nproc)" \
    && cmake --install /tmp/unitree_sdk2/build \
    && ldconfig \
    && rm -rf /tmp/unitree_sdk2

COPY deploy/docker-entrypoint.sh /usr/local/bin/d1-docker-entrypoint
RUN chmod 0755 /usr/local/bin/d1-docker-entrypoint

ENV D1_WORKSPACE=/workspace
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/d1-docker-entrypoint"]
CMD ["bash"]

FROM common AS amd64
ARG TARGETARCH
RUN test "${TARGETARCH}" = amd64
RUN conda run -n trash_collection python -m pip install --no-cache-dir \
      cyclonedds==11.0.1 \
      mujoco==3.11.0 \
      pycollada==0.9.3 \
      trimesh==5.0.0

FROM common AS arm64
ARG TARGETARCH
RUN test "${TARGETARCH}" = arm64
