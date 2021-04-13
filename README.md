# LineFollowingDrone

Line following drone python script. Using a Parrot A.R. Drone 2.0 with ROS melodic.
Packages:

- tum\_simulator\_melodic
- ardrone_autonomy

## Instalation

1. ROS melodic and Gazebo 9
2. tum\_simulator\_melodic and ardrone_autonomy

> installation: [https://github.com/surajmahangade/tum\_simulator\_melodic](https://github.com/surajmahangade/tum_simulator_melodic)

3.  ardrone firmware downgrade

> The ardrone_autonomy driver for the Parrot ARDrone works best with firmware version 2.3.3
> tutorials: <https://www.youtube.com/watch?v=2KwzoBF1YRs> and <https://robot-ing.blogspot.com/2014/03/downgrading-ardrone-20-firmware-to-233.html>
> plf file download: <https://www.durrans.com/ardrone/firmware/2.3.3/> or
> <https://www.espaciodrone.com/todos-los-firmware-del-ar-drone-1-y-2/>

4. cv_bridge

> Used to process sensor_msgs Image as numpy array

```
$ sudo apt-get install ros-melodic-cv-bridge 
```

## running

```
$ roslaunch ardrone_autonomy ardrone.launch
$ roslaunch cvg_sim_gazebo itesm.launch

$ rosrun cvg_sim_gazebo camera.py 
```

## troubleshooting

### Switch to bottom camera

Only needed with ardrone_autonomy package

```
$ rosservice call /ardrone/togglecam 
```

## Notes

### Video

<https://tecmx-my.sharepoint.com/:v:/g/personal/a01706190_itesm_mx/EZZi0TNfx7FPlt5mlw0vXIEB9tKh-ev_dsFtKUYzx2tvAA?e=O89nt2> 

### OpenCV mask

Open CV mask picks desired color range and shows everything else as black. As "Imagen" window shows
![](/_resources/447f92aef2d04e23b9679112db9257e9.jpg)
OGRE script texture doesnt show up
![](/_resources/026a6fe96594466f94087da228afb1fb.jpg)
