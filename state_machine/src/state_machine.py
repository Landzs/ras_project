#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import std_msgs.msg
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import itertools
import tf



#####################################################
#             /robot_odom Callback          #
#####################################################
def odomCallback(msg):
    global x_odom, y_odom, theta_odom
    x_odom = msg.pose.pose.position.x
    y_odom = msg.pose.pose.position.y
    (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    theta_odom = y

#####################################################
#               Initialize Publisher                #
#####################################################
rospy.init_node('state_machine_node', anonymous=True)

# publishers
pub_path_following_VEL = rospy.Publisher('/keyboard/vel', geometry_msgs.msg.Twist, queue_size=1)
pub_target_pose = rospy.Publisher('/target_pose', geometry_msgs.msg.Pose, queue_size=1)

# subscribers
rospy.Subscriber("/robot_odom", Odometry, odomCallback)

rate = rospy.Rate(10)

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
	self.y = y
	self.yaw = yaw

def send_message(LINEAR_VELOCITY, ANGULAR_VELOCITY):
    tf.transformations.quaternion_from_euler()

    POSE = geometry_msgs.msg.Pose()
    POSE.position.x = 0
    POSE.position.y = 0
    POSE.orientation.z = 0
    POSE.orientation.w = 0
    pub_target_pose.publish(POSE)

    (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])


    VEL = geometry_msgs.msg.Twist()
    VEL.linear.x = LINEAR_VELOCITY
    VEL.linear.y = 0.0
    VEL.linear.z = 0.0
    VEL.angular.x = 0.0
    VEL.angular.y = 0.0
    VEL.angular.z = ANGULAR_VELOCITY

    pub_path_following_VEL.publish(VEL)

def main():
    state = "look_for_object"

    while not rospy.is_shutdown():

        print("is in state", state)

        if state == "look_for_object":
            
            send_message(lin_vel, ang_vel*GAIN)
            if (found):
                target_position = [0.25, 0, 0]
                target_orientation = [0.25, 0, 0]
                publish_new_target
                state = "follow_path"

        elif state == "follow_path":
            if path_following_done == True and found:
                state = "grip_object"

        elif state == "grip_object":
            do_the_gripping
            if gripping_done == True:
                target_pose = [0, 0, 0]
                publish_new_target
                state = "follow_path"

        elif state == "release_object":
            do_the_release

        elif state == "stop":

        rate.sleep()

	    



if __name__ == '__main__':
    print("state machine started")
    main()

