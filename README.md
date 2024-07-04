# Differential Drive Robot Gazebo Simulation
A ROS 2 package for simulating and controlling a differential drive robot. This repository includes everything needed to get started with differential drive robots in ROS 2, from URDF modeling and Gazebo simulation to navigation and control.

- [Differential Drive Kinematics](#differential_drive_kinematics)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [References](#references)

### Differential Drive Kinematics
The main concept of differential drive is to rotate around the ICC (_Instantaneous Center of Curvature_) point with the left and right wheel speed.
<div id="header" align="center">
  <img src="https://raw.githubusercontent.com/gurselturkeri/ros2_diff_drive_robot/main/docs/diff_drive_github.png" width="400"/>
  
  <img src="https://raw.githubusercontent.com/gurselturkeri/ros2_diff_drive_robot/main/docs/rviz_robot.gif" width="400"/>
 </div>

- **ICC**: Instantaneous Center of Curvature.
- **ω**: Rotation about the ICC.
- **L**: Distance between the centers of the two wheels.
- **Vr**: Right wheel velocity along the ground.
- **Vl**: Left wheel velocity along the ground.
- **R**: Distance from ICC to the midpoint between the wheels.


$$
\omega (R + L/2) = Vr \\
$$

$$
\omega (R - L/2) = Vl \\
$$

$$
R = \frac{L}{2} \frac{Vl + Vr}{Vr + Vl} \quad \\
\text{and} \\
\quad \omega = \frac{Vr - Vl}{L}
$$

- If **Vl=Vr**, robot move forward linear motion in a straght line. (R=∞)
- If **Vl=-Vr**, robot rotate its around. (R=0)
- If **Vl>Vr**, robot rotates right otherwise rotates left. 


### Mapping with Nav2
Used Nav2 package to generate `.pgm` format map.
<div id="header" align="center">
  <img src="https://raw.githubusercontent.com/gurselturkeri/ros2_diff_drive_robot/main/docs/mapping.gif" width="500"/>
  
 </div>

Add “use_sim_time:=True” to use the Gazebo time. If using the real robot, skip this argument.
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
Same as previous code.
```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
So we need move our robot to discover map.
```
python3 ~/ros2_ws/src/diff_robot/control/keyboard_control.py
```
Once you get a good enough looking map, you can save it.
```
ros2 run nav2_map_server map_saver_cli -f my_map
```

### .pgm format to occupancy grid map

```
map_server map_server --ros-args -p yaml_filename:=~/ros2_ws/src/diff_robot/map/my_map.yaml
```

```
ros2 run nav2_util lifecycle_bringup map_server
```





## A* Algorithm 
## Path Planning

## Dependencies
## Installation

## References
[Dudek, G., &#38; Jenkin, M. (2010). Computational Principles of Mobile Robotics (2nd ed.). Cambridge: Cambridge University Press.](https://doi.org/10.1017/CBO9780511780929) 