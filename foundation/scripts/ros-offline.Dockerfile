# This Ubuntu 24.04 image is TEST-ONLY for the Debian 12 Pupper Pi.
# ARM64/ROS Jazzy matching does not imply libc/libstdc++ ABI compatibility.
# Build deployment binaries on the Pi or in a separately verified Pi-matching sysroot.
# Rebuild only when the recorded local image is unavailable or an intentional
# dependency update is being evaluated. Record resulting image ID/package list.
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential python3-colcon-common-extensions \
    ros-jazzy-controller-interface ros-jazzy-controller-manager-msgs \
    ros-jazzy-hardware-interface ros-jazzy-pluginlib ros-jazzy-realtime-tools \
    ros-jazzy-rclcpp-lifecycle ros-jazzy-rcl-interfaces \
    ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs ros-jazzy-std-msgs \
    ros-jazzy-tf2 ros-jazzy-generate-parameter-library \
    ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
    ros-jazzy-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*
