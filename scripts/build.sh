#!/bin/bash
cd  ~/crazyflie/crazyflie-ros/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON