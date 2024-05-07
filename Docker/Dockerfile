FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    apt-utils \
    nano \
    vim \
    ros-noetic-joy \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    libncurses5-dev \
    libncursesw5-dev \
    libeigen3-dev

RUN apt-get install -y libopencv-dev

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/noetic/setup.bash

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]