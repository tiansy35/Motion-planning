# Motion planning

Code developed while doing course assignments and projects for the motion planning problem of different robotic systems.

## 1. Manipulator

- RRT
  - The RRT algorithm was implemented for a 5-DOF [manipulator](http://www.lynxmotion.com/c-130-al5d.aspx). We only considered the first 4 joints (the last joint does not affect the end-effector position) so the planning was done in 4D space. 
  - Execute `runsim.m` to run tests with different maps and start/goal locations and get the output paths. Here is a [video](https://drive.google.com/file/d/1eNW-7C3mYjRm3z8w2kxhpAuniF7FMIkW/view?usp=sharing) that shows the test we did on real manipulators.

- Artificial Potential Field (APF)
  - The APF method is implemented for the same robot manipulator. We used the statics equation to compute the torques on each joint from the "virtual forces" determined by the potential field functions. We then normalized the torques and took a fixed magnitude step of joint variable updates.
  - Since the APF planner/controller must run in real time, the `runsim.m` code will report an error as the simulation module is not uploaded here. Check out the [video](https://drive.google.com/file/d/1bQIEfGlHMuwrcpp-ZlB_OUZCivDLNGMd/view?usp=sharing) showing a test where we drove the manipulator away with a moving obstacle (the ball). The manipulator is equipped with this [LIDAR](https://www.sparkfun.com/products/15776) so it can sense the obstacle and avoid it in real time (which is a big advantage of the APF method), although due to physical limitations, it moved a bit slowly.
