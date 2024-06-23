# Differential Drive Robot Gazebo Simulation
A ROS 2 package for simulating and controlling a differential drive robot. This repository includes everything needed to get started with differential drive robots in ROS 2, from URDF modeling and Gazebo simulation to navigation and control.

- [Differential Drive Kinematics](#differential_drive_kinematics)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [References](#references)

## Differential Drive Kinematics
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


## Dependencies
## Installation

## References
[Dudek, G., &#38; Jenkin, M. (2010). Computational Principles of Mobile Robotics (2nd ed.). Cambridge: Cambridge University Press.](https://doi.org/10.1017/CBO9780511780929) 