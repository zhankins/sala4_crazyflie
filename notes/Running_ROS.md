Adapted from: https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/

Running ROS in simulation begins in step 2 of the above guide, with the following changes for our workspace, which is set up at `~/crazyflie/crazyflie-ros/`

Instead of running:
```bash
source ~/crazyflie_mapping_demo/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
```
We run
```bash
source ~/crazyflie/crazyflie-ros/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/crazyflie/crazyflie-ros/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
```

In step 3, to run ROS on the live drone, we do not need to run the `gedit` step, as it is already configured for our drone.

Instead of running the following:
```bash
source ~/crazyflie_mapping_demo/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_real.launch.py
```
We use our root workspace directory:
```bash
source ~/crazyflie/crazyflie-ros/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/crazyflie/crazyflie-ros/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_real.launch.py
```

