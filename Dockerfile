FROM ros:noetic-ros-base-focal
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-move-base ros-noetic-global-planner ros-noetic-teb-local-planner ros-noetic-dwa-local-planner \
      ros-noetic-move-base-msgs ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-tf ros-noetic-rostest \
      ros-noetic-gmapping ros-noetic-realsense2-camera \
      ros-noetic-robot-localization ros-noetic-pointcloud-to-laserscan \
      ros-noetic-ntrip-client ros-noetic-nmea-navsat-driver python3-serial \
      python3-nose python3-yaml python3-numpy \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
CMD ["bash"]
