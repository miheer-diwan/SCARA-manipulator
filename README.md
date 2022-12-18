# SCARA-manipulator

In this project, a SCARA manipulator is designed using URDF and position and velocity controllers are implemented to move the robot in the desired way.

### Tools used
- ROS2
- PID controller
- Gazebo

### Methodology

- A URDF model of the SCARA robot was developed and spawned in Gazebo environment.

- The Forward Kinematics and Inverse Kinematics of the robot was calculated and a PID controller was used to control the end-effector position.

- Similarly, the Jacobian and Inverse Jacobian of the robot was calculated and a PID controller was used to control the end-effector velocity.

### Result

The robot moving along +Y direction can be seen in the below GIF.

![Alt Text](https://github.com/mayankbansal82/SCARA-manipulator/blob/main/images/output.gif)

