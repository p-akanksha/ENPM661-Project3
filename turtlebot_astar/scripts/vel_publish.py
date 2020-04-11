#!/usr/bin/env python

import numpy as np 
import rospy
from geometry_msgs.msg import Twist
import os

# class get_going():

#   def __init__(self):
    
#     dirpath = os.path.dirname(os.path.realpath(__file__))
#     file_val = np.load(dirpath + '/params.npy', None, True, True, 'ASCII')

#     self.R = 0.177 # robot radius - m 
#     self.r = 0.038 # wheel radius - m
  
#     self.left = file_val[0,:]*self.r
#     self.right = file_val[1,:]*self.r
#     self.vel = np.vstack(( self.left, self.right )).T

#     self.linear = np.mean(self.vel, axis=0)
#     # ccw rot is +ve
#     self.angular = (self.right - self.left)/self.R 

#     self.cnt = 0

#     self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

#     self.timer = rospy.Timer(rospy.Duration(1), self.callback)
    
#   def callback(self, timer):

#     move_cmd = Twist()
#     # move_cmd_init.linear.x = 0
#     # move_cmd_init.angular.z = 0

#     self.cmd_vel.publish(move_cmd)
def get_going():
    
    dirpath = os.path.dirname(os.path.realpath(__file__))
    file_val = np.load(dirpath + '/params.npy', None, True, True, 'ASCII')

    R = 0.177 # robot radius - m 
    r = 0.038 # wheel radius - m
  
    left = file_val[0,:]*r
    right = file_val[1,:]*r
    vel = np.vstack(( left, right )).T
    
    linear = np.mean(vel, axis=1)
    # ccw rot is +ve
    angular = (right - left)/R 

    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    move_cmd_init = Twist()
    move_cmd_init.linear.x = 0
    move_cmd_init.angular.z = 0

    cnt = 0
    move_cmd = Twist()

    while cnt < len(left):

        move_cmd.linear.x = linear[cnt]
        move_cmd.angular.z = angular[cnt]
        cnt += 1

        t0 = rospy.Time.now().to_sec()
        tf = t0

        while( tf - t0 <= 1):
            cmd_vel.publish(move_cmd)
            tf = rospy.Time.now().to_sec()
            
        cmd_vel.publish(move_cmd_init)

if __name__ == '__main__':

    rospy.init_node('vel_publish', anonymous=True)

    try:
        get_going()
       
    except rospy.ROSInterruptException:
        print("exception thrown")
        pass