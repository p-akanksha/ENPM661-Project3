# ENPM661: Project3, Phase 3, 4
This project is an implementation of A* path planning algorithm in a continuous workspace for a turtlebot, which is a non-holonomic robot. 

To begin with, add the package in a catkin workspace and build the workspace using ```catkin build```.

Make vel_publish.py and Astar_rigid.py an executable by entering the following in the terminal opened at the scripts folder of the package: 

```chmod +x vel_publish.py```

```chmod +x Astar_rigid.py```

To find the path and run the simulation, enter the following in the command line. Note that the values corresponding to x_s, y_s, theta in the below command are the starting x, y coordinates (in m) and theta (in radians) respectively.

```roslaunch turtlebot_astar turtlebot_run.launch x_s:=4 y_s:=3 theta:=3.14```

These three parameters (x_s, y_s, theta) define the start position of the robot in gazebo. Since the map provided has the coordinate axis flipped, the value of x_s = -x_start, y_s = -y_start and theta = theta_start + 3.14.

Note: Remember to source your workspace in all terminals!

```source devel/setup.bash```

## Dependencies
1. Python 2.7
2. Numpy
3. OpenCV
4. ROS Kinetic 
5. Gazebo

