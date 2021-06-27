# Motion planning

Code developed while doing course assignments and projects for the motion planning problem of different robotic systems.

## 1. Manipulator

- RRT
  - The RRT algorithm was implemented for the 5-DOF [Lynxmotion](http://www.lynxmotion.com/c-130-al5d.aspx) manipulator. We only considered the first 4 joints (the last joint does not affect the end-effector position) so the planning was done in 4D space. 
  - Execute `runsim.m` to run tests with different maps and start/goal locations and get the output paths. Here is a [video](https://drive.google.com/file/d/1eNW-7C3mYjRm3z8w2kxhpAuniF7FMIkW/view?usp=sharing) that shows the test we did on real manipulators.
