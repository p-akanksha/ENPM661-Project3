# ENPM661-Project3
This project is an implementation of A* path planning algorithm in a continuous workspace for a differential drive robot. 

To run the program, enter the following in your terminal 
```python Astar_rigid.py```

Upon running the script, the user will be prompted to enter the following:
1. Two wheel RPMs
2. Start point (x, y, th)
3. Goal point (x, y)
4. Clearance (minimum 0.1 m)

An error message will be displayed for the following:
1. If any of the start or end points lie within the obstacle or outside the workspace.
2. If the clearance entered is less than 0.1 m 

# Notes

1. The input values used for the video submission:
	1. Wheel RPM = [10, 20]
	2. Start Point: [-4, -3, 0]
	3. Goal Point: [4, 2.5]
	4. Clearance: 0.1
2. As the number of nodes explored can be huge, the video submission displays the explored region that is updated after exploring 10 nodes.
3. The gaol check threshould is 0.1 m
4. The threshoud used to check for visited node is : 
	1. thresh_d = 0.1
	2. thresh_theta = 10 

## Dependencies
1. Python 2.7
2. Numpy
3. OpenCV

