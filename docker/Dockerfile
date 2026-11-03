FROM ros:jazzy
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      ros-jazzy-ros-gz ros-jazzy-nav2-minimal-tb3-sim \
      ros-jazzy-py-trees ros-jazzy-py-trees-ros \
      ros-jazzy-foxglove-bridge \
      python3-pytest \
    && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
# Each container runs its own simulation: only discover ROS nodes inside it,
# so two simulations on the same machine never see each other's topics.
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
