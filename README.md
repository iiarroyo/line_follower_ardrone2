# LineFollowingDrone
Line following drone python script. Using a Parrot A.R. Drone 2.0 with ROS melodic. 
Packages:
- tum_simulator_melodic
- ardrone_autonomy


## instalation

## running
    roslaunch ardrone_autonomy ardrone.launch
    roslaunch cvg_sim_gazebo itesm.launch

    rosrun cvg_sim_gazebo movement.py
## troubleshooting
### Camera not working

    rosservice call /ardrone/togglecam 