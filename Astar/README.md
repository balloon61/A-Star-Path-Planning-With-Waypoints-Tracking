## Author: 
Po-Lun Chen (118307435)
Pin-Hao Huang (117517842)

## Github Link
https://github.com/balloon61/A-star.git


## Require Package
- numpy
- cv2
- heapq
- tqdm
- argparse

This simulation was done using ros-noetic

## Run Simulation
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch x_pos:=-4 y_pos:=-4 yaw:=-0.5 x_goal:=4 y_goal:=4 rpm_min:=15 rpm_max:=50
```

Arguments:
```bash
x_pos: start x position [meter]
y_pos: start y position [meter]
yaw: start orientation [rad]
x_goal: goal x position
y_goal: goal y position
rpm_min: Mininum RPM
rpm_max: Maximum RPM
