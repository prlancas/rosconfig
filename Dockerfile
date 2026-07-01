# Minimal headless ROS 2 image for the Droidal SLAM stack.
#
# Based on ros:jazzy-ros-base (NOT desktop-full) because nothing here is
# graphical - no RViz, no GUIs. This drops the image from several GB to a
# fraction of that. slam-toolbox (what slam.sh used to apt-install at runtime)
# is baked in so the container is ready to run on first boot.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-slam-toolbox \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf-transformations \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        python3-serial \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

ENV DROIDAL_DIR=/opt/droidal
WORKDIR ${DROIDAL_DIR}

# Robot scripts + SLAM params (the things you used to run by hand in /mnt).
COPY mnt/ ${DROIDAL_DIR}/
COPY launch/ ${DROIDAL_DIR}/launch/
COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh && chmod +x ${DROIDAL_DIR}/*.py

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/opt/droidal/launch/bringup.launch.py"]
