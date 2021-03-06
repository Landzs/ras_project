#!/usr/bin/env python

import os
import rospy
import readchar
import threading
import std_msgs.msg
import phidgets.msg
import geometry_msgs.msg



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

#####################################################
#             Keyboard Interface Class              #
#####################################################
class Keyboard_Interface:
    #####################################################
    #          Initialize Object                        #
    #####################################################
    def __init__(self):
        #####################################################
        #               Initialize Variables                #
        #####################################################
        self.LINEAR_VELOCITY = 0.0
        self.ANGULAR_VELOCITY = 0.0
        self.ENCODER_LEFT = 0
        self.ENCODER_RIGHT = 0
        self.KEY = ''
        self.STATE_UPDATED_LEFT = False
        self.STATE_UPDATED_RIGHT = False
        self.LINEAR_STEP_VEL = 0.05
        self.ANGULAR_STEP_VEL = 0.3
        self.VEL_LEFT = 0.0
        self.VEL_RIGHT = 0.0
        self.EST_LINEAR_VELOCITY = 0.0
        self.EST_ANGULAR_VELOCITY = 0.0



        #####################################################
        #             Initialize ROS Parameter              #
        #####################################################
        rospy.init_node('keyboard_interface_node', anonymous=True)
        self.pub_KEYBOARD_VEL = rospy.Publisher('/keyboard/vel', geometry_msgs.msg.Twist, queue_size=1)
        self.rate = rospy.Rate(10)

        rospy.Subscriber('/left_motor/encoder', phidgets.msg.motor_encoder, self.update_feedback_enc_left)
        rospy.Subscriber('/right_motor/encoder', phidgets.msg.motor_encoder, self.update_feedback_enc_right)



    #####################################################
    #             /left_motor/encoder Callback          #
    #####################################################
    def update_feedback_enc_left(self, feedback_enc):
        self.ENCODER_LEFT = feedback_enc.count_change
        # self.STATE_UPDATED_LEFT = True

    #####################################################
    #             /right_motor/encoder Callback          #
    #####################################################
    def update_feedback_enc_right(self,feedback_enc):
        self.ENCODER_RIGHT = feedback_enc.count_change
        # self.STATE_UPDATED_RIGHT = True

    ####################################################
    #          Publish KEYBOARD INPUT                  #
    #####################################################
    def publish_vel(self):

        vel = geometry_msgs.msg.Twist()

        while not rospy.is_shutdown():

            vel.linear.x = self.LINEAR_VELOCITY
            vel.linear.y = 0.0
            vel.linear.z = 0.0

            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = self.ANGULAR_VELOCITY

            self.pub_KEYBOARD_VEL.publish(vel)
            #self.STATE_UPDATED = True
            self.rate.sleep()


    #####################################################
    #                   Clear Screen                    #
    #####################################################
    def cls(self):
        os.system("clear")

    #####################################################
    #                  Calculate Vel                    #
    #####################################################
    def calculate_vel(self):
        pi = 3.14
        control_frequency = 10
        ticks_per_rev = 900

        self.VEL_LEFT = (self.ENCODER_LEFT * 2 * pi * control_frequency) / (ticks_per_rev)
        self.VEL_RIGHT = (self.ENCODER_RIGHT * 2 * pi * control_frequency) / (ticks_per_rev)
        self.EST_LINEAR_VELOCITY = (self.VEL_RIGHT + self.VEL_LEFT)
        self.EST_ANGULAR_VELOCITY = (self.VEL_RIGHT - self.VEL_LEFT)


    #####################################################
    #               Display Interface                   #
    #####################################################
    def display_interface(self):

        #####################################################
        #               WHILE LOOP for User Input           #
        #####################################################
        while not rospy.is_shutdown():

            # while not self.STATE_UPDATED_LEFT or not self.STATE_UPDATED_RIGHT:
            #     pass
            self.calculate_vel()
            #####################################################
            #        Print Control Interface to Terminal        #
            #####################################################
            self.cls()
            print("#####################################" + '\r')
            print("LINEAR_VELOCITY:    [     w | s    ]" + '\r')
            print("ANGULAR_VELOCITY:   [     a | d    ]" + '\r')
            print("EMERGENCY STOP:     [       e      ]" + '\r')
            print("QUIT:               [       q      ]" + '\r')
            print("#####################################" + '\r')
            print('Input:   LINEAR_VELOCITY: {0} | ANGULAR_VELOCITY: {1}\r'.format(self.LINEAR_VELOCITY, self.ANGULAR_VELOCITY))
            print('Encoder: LEFT_VEL: {0} | RIGHT_VEL: {1}\r'.format(self.ENCODER_LEFT, self.ENCODER_RIGHT))
            print('Est: LINEAR_VEL: {0} | ANGULAR_VEL: {1}\r'.format(self.EST_LINEAR_VELOCITY, self.EST_ANGULAR_VELOCITY))
            print("Control input: " + self.KEY)

            # self.STATE_UPDATED_LEFT = False
            # self.STATE_UPDATED_RIGHT = False

            self.rate.sleep()

    #####################################################
    #                    Read Input                     #
    #####################################################
    def read_input(self):
        while self.KEY != 'q' and not rospy.is_shutdown():
            self.KEY = readchar.readchar()

            #####################################################
            #        READ KEY for updating LEFT                 #
            #####################################################
            if self.KEY == 'w':
                self.LINEAR_VELOCITY = self.LINEAR_VELOCITY + self.LINEAR_STEP_VEL
            elif self.KEY == 's':
                self.LINEAR_VELOCITY = self.LINEAR_VELOCITY - self.LINEAR_STEP_VEL
            else:
                pass

            #####################################################
            #        READ KEY for updating RIGHT                #
            #####################################################
            if self.KEY == 'a':
                self.ANGULAR_VELOCITY = self.ANGULAR_VELOCITY + self.ANGULAR_STEP_VEL
            elif self.KEY == 'd':
                self.ANGULAR_VELOCITY = self.ANGULAR_VELOCITY - self.ANGULAR_STEP_VEL
            else:
                pass



            #####################################################
            #        READ KEY for emergency exit                #
            #####################################################
            if self.KEY == 'e':
                self.LINEAR_VELOCITY = 0
                self.ANGULAR_VELOCITY = 0
            else:
                pass

            #self.STATE_UPDATED = False

#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        RAS = Keyboard_Interface()
        Thread_vel = ThreadedFunction(RAS.publish_vel)
        Thread_display = ThreadedFunction(RAS.display_interface)
        Thread_vel.start()
        Thread_display.start()
        RAS.read_input()
    except rospy.ROSInterruptException:
        pass
