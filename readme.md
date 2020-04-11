# ENPM661-Project3
This project is an implementation of A* path planning algorithm in a continuous workspace for a turtlebot, which is a non-holonomic robot. 

To run the program, enter the following in one terminal 

$export ROBOT_INITIAL_POSE="-x -4 -y -3 -Y 0"

$roslaunch turtlebot_astar turtlebot_run.launch

In an other terminal, run: 

$rosrun turtlebot_astar vel_publish.py

Remember to source your workspace before that! 

$source devel/setup.bash

## Dependencies
1. Python 2.7
2. Numpy
3. OpenCV
4. ROS Kinetic

