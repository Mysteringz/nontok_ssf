# ROS 2 Humble with everything the Nontok workspace needs, for building and running the
# simulated robot on a laptop. The Jetson installs the same packages natively.
#
#   docker build -t nontok-dev docker
#   docker run --rm -it -v "$PWD":/nontok -w /nontok/ros2_ws nontok-dev
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-robot-localization \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        ros-humble-foxglove-bridge \
        ros-humble-cv-bridge \
        ros-humble-tf-transformations \
        python3-pip python3-pytest python3-serial python3-yaml python3-numpy python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# pupil-apriltags has no ready-made package for every platform; without it the detector falls
# back to OpenCV's AprilTag support.
# numpy stays below 2: the system OpenCV and ROS packages are built against numpy 1.
RUN pip3 install --no-cache-dir 'numpy<2' onnxruntime \
 && (pip3 install --no-cache-dir --only-binary=:all: 'numpy<2' pupil-apriltags || echo 'pupil-apriltags not installed')

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /nontok/ros2_ws/install/setup.bash ] && source /nontok/ros2_ws/install/setup.bash' >> /root/.bashrc
