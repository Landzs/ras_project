#!/usr/bin/env python

import os
import rospy
import readchar
import threading
import std_msgs.msg
import phidgets.msg
import geometry_msgs.msg
#####################################################
#               Initialize Variables                #
#####################################################
LINEAR_VELOCITY = 0.0
ANGULAR_VELOCITY = 0.0
ENCODER_LEFT = 0
ENCODER_RIGHT = 0
KEY = ''
STATE_UPDATED = False
LINEAR_STEP_VEL = 0.025
ANGULAR_STEP_VEL = 0.25

#####################################################
#             /left_motor/encoder Callback          #
#####################################################
def update_feedback_enc_left(feedback_enc):
    global ENCODER_LEFT
    ENCODER_LEFT = feedback_enc.count_change
#	self.FEEDBACK_ENC_UPDATED = True

#####################################################
#             /right_motor/encoder Callback          #
#####################################################
def update_feedback_enc_right(feedback_enc):
    global ENCODER_RIGHT
    ENCODER_RIGHT = feedback_enc.count_change
#	self.FEEDBACK_ENC_UPDATED = True


#####################################################
#               Initialize Publisher                #
#####################################################
rospy.init_node('keyboard_interface_node', anonymous=True)
pub_KEYBOARD_VEL = rospy.Publisher('/keyboard/vel', geometry_msgs.msg.Twist, queue_size=1)
rate = rospy.Rate(10)

rospy.Subscriber('/left_motor/encoder', phidgets.msg.motor_encoder, update_feedback_enc_left)
rospy.Subscriber('/right_motor/encoder', phidgets.msg.motor_encoder, update_feedback_enc_right)


#rospy.spin()


#####################################################
#               Initialize Threading                #
#####################################################
class ThreadedFunction(threading.Thread):

    def __init__(self, fcn_to_thread):
        threading.Thread.__init__(self)

        self.runnable = fcn_to_thread
        self.daemon = True

    def run(self):
        self.runnable()


####################################################
#          Publish KEYBOARD INPUT                  #
#####################################################
def publish_vel():
    global LINEAR_VELOCITY, ANGULAR_VELOCITY, STATE_UPDATED
    VEL = geometry_msgs.msg.Twist()

    while not rospy.is_shutdown():
        VEL.linear.x = LINEAR_VELOCITY
        VEL.linear.y = 0.0
        VEL.linear.z = 0.0

        VEL.angular.x = 0.0
        VEL.angular.y = 0.0
        VEL.angular.z = ANGULAR_VELOCITY
    # rospy.spin()
        pub_KEYBOARD_VEL.publish(VEL)
        STATE_UPDATED = True
        rate.sleep()


#####################################################
#                   Clear Screen                    #
#####################################################
def cls():
    os.system("clear")


#####################################################
#   Initialize Control Interface for Terminal       #
#####################################################
def control():
    global LINEAR_VELOCITY, ANGULAR_VELOCITY, ENCODER_LEFT, ENCODER_RIGHT, KEY
    global STATE_UPDATED, LINEAR_STEP_VEL, ANGULAR_STEP_VEL

    #####################################################
    #               WHILE LOOP for User Input           #
    #####################################################
    while KEY != 'q' and not rospy.is_shutdown():

        while not STATE_UPDATED:
            pass

        #####################################################
        #        Print Control Interface to Terminal        #
        #####################################################
        cls()
        print("#####################################")
        print("LINEAR_VELOCITY:    [     w | s    ]")
        print("ANGULAR_VELOCITY:   [     a | d    ]")
        print("UPDATE_ENCODER:     [       u      ]")
        print("QUIT:               [       q      ]")
        print("#####################################")
        print('Input:   LINEAR_VELOCITY: {0} | ANGULAR_VELOCITY: {1}'.format(LINEAR_VELOCITY, ANGULAR_VELOCITY))
        print('Encoder: LEFT_VEL: {0} | RIGHT_VEL: {1}'.format(ENCODER_LEFT, ENCODER_RIGHT))
        print("Control input: " + KEY)
        KEY = readchar.readchar()
        # cls()

        #####################################################
        #        READ KEY for updating LEFT                 #
        #####################################################
        if KEY == 'w':
            LINEAR_VELOCITY = LINEAR_VELOCITY + LINEAR_STEP_VEL
        elif KEY == 's':
            LINEAR_VELOCITY = LINEAR_VELOCITY - LINEAR_STEP_VEL
        else:
            pass

        #####################################################
        #        READ KEY for updating RIGHT                #
        #####################################################
        if KEY == 'a':
            ANGULAR_VELOCITY = ANGULAR_VELOCITY + ANGULAR_STEP_VEL
        elif KEY == 'd':
            ANGULAR_VELOCITY = ANGULAR_VELOCITY - ANGULAR_STEP_VEL
        else:
            pass

        STATE_UPDATED = False


#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        Thread_vel = ThreadedFunction(publish_vel)
        Thread_vel.start()
        control()
    except rospy.ROSInterruptException:
        pass
