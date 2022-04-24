Team member: 
Pin-Hao Huang (117517542), Po-Lun Chen (118307435)

Github Link:


Require package:
numpy, cv2, heapq, tqdm, argparse, Odometry, euler_from_quaternion, quaternion_from_euler

Arguments:
(x_pos, y_pos, yaw) = start position 
(x_goal, y_goal) = goal position 
(rpm_min, rpm_max) = Maximum RPM and Minimum RPM
# The following command line is the sample input
roslaunch turtlebot3_gazebo turtlebot3_world.launch x_pos:=-4 y_pos:=-4 yaw:=-0.5 x_goal:=4 y_goal:=4 rpm_min:=15 rpm_max:=50

