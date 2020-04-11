# ENPM661-Project3
This project is an implementation of A* path planning algorithm in a continuous workspace for a turtlebot, which is a non-holonomic robot. 

To find the path, run:

$ python Astar_rigid.py

To run the simulation in ros, enter the following in one terminal 

$export ROBOT_INITIAL_POSE="-x -4 -y -3 -Y 0"

$roslaunch turtlebot_astar turtlebot_run.launch

In an other terminal, run: 

$rosrun turtlebot_astar vel_publish.py

Note: If you are not able to tab complete the above command, you might have to make vel_publish.py an executable by entering the following in the scripts folder of the package: 
$ chmod +x vel_publish.py

Remember to source your workspace before that! 

$source devel/setup.bash

## Dependencies
1. Python 2.7
2. Numpy
3. OpenCV
4. ROS Kinetic

