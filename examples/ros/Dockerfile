FROM ros:humble-ros-base

RUN apt update && apt install -y python3-pip
RUN pip install websockets
RUN pip install pydantic
RUN pip install pytest
RUN pip install pytest-asyncio

COPY actions /ws/src
WORKDIR /ws

RUN rosdep install --from-paths src --ignore-src -r -y --rosdistro=humble
RUN . /opt/ros/humble/setup.sh; colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY py_wrapper.sh /py_wrapper.sh
RUN chmod +x /py_wrapper.sh

RUN echo "source /ws/install/setup.bash" >> /root/.bashrc
