#!/usr/bin/env python
import numpy as np 
import rospy
from geometry_msgs.msg import Twist
import os
import time
flag = 0

def callback(data):
    flag = data.data

def get_going():
    
    dirpath = os.path.dirname(os.path.realpath(__file__))
    file_val = np.load(dirpath + '/params.npy', None, True, True, 'ASCII')

    L = 0.354
    R = 0.177 # robot radius - m 
    r = 0.038 # wheel radius - m

    left = file_val[0,:]*r
    right = file_val[1,:]*r
    vel = np.vstack(( left, right )).T
    
    linear = np.mean(vel, axis=1)
    # ccw rot is +ve
    angular = (right - left)/L

    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1000)

    move_cmd_init = Twist()
    move_cmd_init.linear.x = 0
    move_cmd_init.angular.z = 0

    cmd_vel.publish(move_cmd_init)

    time.sleep(3)

    cnt = 0
    move_cmd = Twist()

    while not rospy.is_shutdown():

        if cnt < len(left):
            move_cmd.linear.x = linear[cnt]
            move_cmd.angular.z = angular[cnt]
            cnt += 1

            t0 = rospy.Time.now().to_sec()
            tf = t0

            r = rospy.Rate(10)

            while( tf - t0 <= 1):
                cmd_vel.publish(move_cmd)
                tf = rospy.Time.now().to_sec()
                r.sleep()
                
            cmd_vel.publish(move_cmd_init)

if __name__ == '__main__':

    rospy.init_node('vel_publish', anonymous=True)
    rospy.Subscriber("my_flag", Int8, callback)
    rospy.spin() 
    try:
        if flag:
            get_going()
       
    except rospy.ROSInterruptException:
        print("exception thrown")
        pass
