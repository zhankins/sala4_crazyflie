# sala4_crazyflie

Crazyflie autonomous flight development for SALA4 ULI Project at USC.

This repository contains two ROS2 packages: `sala4_bringup` and `sala4`, which are responsible for preparing the drone for takeoff, and actual commands for the drone respectively. This repository also contains several utility scripts for various functionality, explained below.

This Readme is structured in the steps necessary to get the Crazyflie up and running.

## Building

To build the packages contained within this repository, there is a helpful build script located at `scripts/build.sh`. You must rebuild the ROS workspace after creating or renaming any python file in either the `sala4_bringup` or `sala4` packages. To build, run:
`source scripts/build.sh`, which automatically navigates to our root ROS workspace and back to this folder.

## Prelaunch

After building and before running any ros2 launch commands, you must first run the prelaunch script at `scripts/prelaunch_ros.sh`. This is a simple script which runs necessary ros2 bash files and adds the necessary gazebo resource path. **This must be run in every terminal before running any ros2 commands.**
To run:
`source scripts/prelaunch_ros.sh`

## ROS2 - sala4_bringup

The `sala4_bringup` package is responsible for setting up our Crazyflie environment for both simulation and real-life purposes. It currently contains two launch files, `crazyflie_real.launch.py` and `crazyflie_simulation.launch.py`, for real-life and simulation use respectively. They are essentially identical, but the real launch file includes velocity and arming nodes, which are necessary for initializing the velocity twist topic for drone control and for enabling the motors on the brushless drone.

To run the real-life launch file, run:
`ros2 launch sala4_bringup crazyflie_real.launch.py`

To run the simulation launch file, run:
`ros2 launch sala4_bringup crazyflie_simulation.launch.py`

## ROS2 - sala4

After the drone has been initialized via the launch files, we can run any of the scripts inside of the sala4 package, at `./sala4/sala4` by using the following command:
`ros2 run sala4 [script_name].py`
For example, to run `mapper_multiranger.py`:
`ros2 run sala4 mapper_multiranger.py`
