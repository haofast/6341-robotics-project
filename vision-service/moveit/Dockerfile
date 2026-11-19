FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-moveit \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /project
