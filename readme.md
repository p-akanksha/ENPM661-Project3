# ENPM661: Project3, Phase 3, 4
This project is an implementation of A* path planning algorithm in a continuous workspace for a turtlebot, which is a non-holonomic robot. 

To begin with, add the package in a catkin workspace and build the workspace using ```catkin build```.

To find the path and run the simulation, enter the following in the command line. Note that the value after -x, -y, -Y in the below command are the starting x, y coordinates and theta respectively.

```roslaunch turtlebot_astar turtlebot_run.launch ```

Note: If you are not able to tab complete the above command, you might have to make vel_publish.py an executable by entering the following in the scripts folder of the package: 
```chmod +x vel_publish.py```

Remember to source your workspace before that! 

```source devel/setup.bash```

## Dependencies
1. Python 2.7
2. Numpy
3. OpenCV
4. ROS Kinetic 
5. Gazebo

