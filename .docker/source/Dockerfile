ARG ROS_DISTRO="foxy"
FROM ros:${ROS_DISTRO}
ENV OVERLAY_WS /root/overlay_ws
WORKDIR $OVERLAY_WS/src

# Copy source files
COPY . consai_msg_publisher

# Install depnedencies
ARG FROOTSPI_MSGS_BRANCH="main"
RUN apt-get update && rosdep update && \
    git clone -b $FROOTSPI_MSGS_BRANCH https://github.com/SSL-Roots/consai_frootspi_msgs.git && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

# Build packages
ARG COLCON_OPTION=""
RUN cd $OVERLAY_WS && \
        . /opt/ros/${ROS_DISTRO}/setup.sh && \
        colcon build --symlink-install $COLCON_OPTION

# Edit entrypoint to source overlay setup file
WORKDIR $OVERLAY_WS
RUN sed --in-place --expression \
    '$i if [ -e $OVERLAY_WS/install/setup.bash ]; then\n\tsource "$OVERLAY_WS/install/setup.bash" \nfi' \
    /ros_entrypoint.sh
