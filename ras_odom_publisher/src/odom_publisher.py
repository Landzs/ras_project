#!/usr/bin/env python


import rospy
import std_msgs.msg
import phidgets.msg
import geometry_msgs.msg
import math
from nav_msgs.msg import Odometry
#####################################################
#               Initialize Variables                #
#####################################################
ENCODER_LEFT = 0
ENCODER_RIGHT = 0
LINEAR_VELOCITY = 0.0
ANGULAR_VELOCITY = 0.0


#####################################################
#             /left_motor/encoder Callback          #
#####################################################
def update_feedback_enc_left(feedback_enc):
    global ENCODER_LEFT
    ENCODER_LEFT = feedback_enc.count_change


#	self.FEEDBACK_ENC_UPDATED = True

#####################################################
#             /right_motor/encoder Callback         #
#####################################################
def update_feedback_enc_right(feedback_enc):
    global ENCODER_RIGHT
    # NOTE THE MINUS SIGN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ENCODER_RIGHT = -feedback_enc.count_change




#####################################################
#               Initialize Publisher                #
#####################################################
rospy.init_node('odom_publish_node', anonymous=True)
pub_odom = rospy.Publisher('/robot_odom', Odometry, queue_size=1)
rate = rospy.Rate(10)

rospy.Subscriber('/left_motor/encoder', phidgets.msg.motor_encoder, update_feedback_enc_left)
rospy.Subscriber('/right_motor/encoder', phidgets.msg.motor_encoder, update_feedback_enc_right)
#rospy.spin()

#####################################################
#            Controller Function                    #
#####################################################
def publisher():
    global LINEAR_VELOCITY, ANGULAR_VELOCITY, ENCODER_LEFT, ENCODER_RIGHT

    ODOM = Odometry()
    base = 0.24
    sita = 0
    x = 0
    y = 0
    dt = 0.1

    pi = 3.14
    control_frequency = 10
    ticks_per_rev = 900

    while not rospy.is_shutdown():
        current_time = rospy.get_rostime()

        v_left = (ENCODER_LEFT * 2 * pi * control_frequency) / (ticks_per_rev)
        v_right = (ENCODER_RIGHT * 2 * pi * control_frequency) / (ticks_per_rev)
        sita = sita + (1/base) * dt *(v_right - v_left)

        x = x + 0.5 * (v_left + v_right) * math.cos(sita)

        y = y + 0.5 * (v_left + v_right) * math.sin(sita)

        ODOM.pose.pose.position.x = x
        ODOM.pose.pose.position.y = y

        ODOM.pose.pose.orientation.z = sita
        ODOM.header.stamp = current_time
        ODOM.header.frame_id = "base_link"
        ODOM.child_frame_id = "odom"
        pub_odom.publish(ODOM)

        rate.sleep()


#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
