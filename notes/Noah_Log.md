# Oct. 23
- ROS Workspace is completely set up, including Gazebo Harmonic and Crazyswarm ROS libraries.
- Successfully ran simulation in RViz and Gazebo, for simple wall mapping
- Attempted to run ROS with the live drone, but encountered an issue where the drone's roll appears to drift heavily
  - Observing using the `cfclient` plotter, it looks like the roll increases at a steady rate and then resets after a certain amount of seconds, or after a certain increment of 9 degrees.
  - From [this github issue](https://github.com/orgs/bitcraze/discussions/1160), it appears that this could be an issue with the flow deck's mounting
    - Testing the crazyflie with the flow deck and turning it off and on to recalibrate produced the same ramping Pose in the plotter each time
    - Testing the crazyflie withOUT the flow deck, and turning it off and on again to recalibrate produced the same ramping pose each time, but a different ramping pose from the plot with the flow deck. 
  - This indicates to me that there could be some conflicting pose estimation between the decks, which is evidenced by [this article](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/) which leads us to believe that we need to change to the Kalman estimator in code
  - Another issue was that there was a bad array access within the example code, which didnt seem to be fixed after changing the code. Need to investigate that further