FROM ros:lyrical-ros-base-resolute

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-lyrical-ros-gz \
        ros-lyrical-rviz2 \
        ros-lyrical-teleop-twist-keyboard \
        python3-colcon-common-extensions \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work/robot_ws

CMD ["bash"]
