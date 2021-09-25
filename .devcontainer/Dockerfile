FROM px4io/px4-dev-ros-noetic:latest
LABEL Name=puzzle Version=0.0.1
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# RUN apt update \
#     && apt install ros-noetic-rviz -y \
#     && apt install ros-noetic-rqt-common-plugins -y \
#     && apt install ros-noetic-rqt-robot-plugins -y

RUN git config --global user.email "RossRich@bk.ru" && git config --global user.name "Ross Rich"