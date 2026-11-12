FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install ROS 1 Noetic packages, OpenCV, and native AprilTag libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-catkin-tools \
    python3-opencv \
    python3-numpy \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-image-transport-plugins \
    ros-noetic-compressed-image-transport \
    ros-noetic-rosbridge-suite \
    ros-noetic-web-video-server \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-apriltag \
    ros-noetic-apriltag-ros \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create Catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}

# Copy our ROS package
COPY src/ros1_apriltag ${CATKIN_WS}/src/ros1_apriltag
RUN chmod +x ${CATKIN_WS}/src/ros1_apriltag/scripts/*.py

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Automatically source ROS setup in every interactive bash session (for `docker exec -it ... bash`)
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/catkin_ws/devel/setup.bash ]; then source /root/catkin_ws/devel/setup.bash; fi" >> /root/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc

# Ensure ROS binaries and Python packages are in PATH and PYTHONPATH system-wide
ENV PATH="/root/catkin_ws/devel/bin:/opt/ros/noetic/bin:${PATH}"
ENV PYTHONPATH="/root/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH}"

# Expose ports:
# 9090: rosbridge websocket
# 8080: web_video_server (view detections in browser)
EXPOSE 9090 8080

ENTRYPOINT ["/entrypoint.sh"]
# --wait: wait for the host's ROS master instead of starting our own roscore
# (with host networking, a roscore started here would take port 11311 from the host's).
CMD ["roslaunch", "--wait", "ros1_apriltag", "apriltag_pipeline.launch"]
