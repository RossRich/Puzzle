FROM px4io/px4-dev-ros-noetic:latest
LABEL Name=puzzle Version=0.0.1
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

RUN apt update \
    && apt install software-properties-common -y

RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

RUN apt update \
    && apt install ros-noetic-rviz -y \
    && apt install ros-noetic-rqt-common-plugins -y \
    && apt install ros-noetic-rqt-robot-plugins -y \
    && apt-get install librealsense2-dkms -y \
    && apt-get install librealsense2-utils -y \
    && apt-get install librealsense2-dev -y
    # && apt install libopencv-dev python3-opencv -y

RUN git config --global user.email "RossRich@bk.ru" && git config --global user.name "Ross Rich"