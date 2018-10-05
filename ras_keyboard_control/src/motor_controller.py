#!/usr/bin/env python

import rospy
import std_msgs.msg
import phidgets.msg
import geometry_msgs.msg
import numpy as np

'''
CLASS: motor_controller()
'''
class motor_controller():
    '''Member variables'''
    def __init__(self):
        self.ENCODER_LEFT = 0
        self.ENCODER_LEFT = 0
        self.LINEAR_VELOCITY = 0.0
        self.ANGULAR_VELOCITY = 0.0
        self.control_frequency = 10  # in Hz, so dt = 1/control_frequency
        self.ticks_per_rev = 950*3

        # vehicle parameters
        self.dt = 1/self.control_frequency
        self.base = 0.24
        self.wheel_radius = 0.0485

        # PID parameters
        self.Kp_left = 30.0
        self.Kp_right = 40.0
        self.Ki_left = 400.0
        self.Ki_right = 400.0
        self.Kd_left = 0
        self.Kd_right = 0
    
        #####################################################
        #               Initialize Publisher                #
        #####################################################
        rospy.init_node('motor_control_node', anonymous=True)
        self.pub_LEFT_MOTOR = rospy.Publisher(
            '/left_motor/cmd_vel', std_msgs.msg.Float32, queue_size=1)
        self.pub_RIGHT_MOTOR = rospy.Publisher(
            '/right_motor/cmd_vel', std_msgs.msg.Float32, queue_size=1)
        self.rate = rospy.Rate(self.control_frequency)
    
    '''Member functions'''

    #####################################################
    #             /left_motor/encoder Callback          #
    #####################################################
    def update_feedback_enc_left(self, feedback_enc):
        #global ENCODER_LEFT, ENCODER_LEFT_TEMP, has_updated_left
        self.ENCODER_LEFT = feedback_enc.count_change

    #####################################################
    #             /right_motor/encoder Callback         #
    #####################################################
    def update_feedback_enc_right(self, feedback_enc):
        #global ENCODER_RIGHT, ENCODER_RIGHT_TEMP, has_updated_right
        # NOTE THE MINUS SIGN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.ENCODER_RIGHT = -feedback_enc.count_change

    #####################################################
    #             /keyboard/vel Callback                #
    #####################################################
    def update_feedback_keyboard_vel(self, feedback_enc):
        #global LINEAR_VELOCITY, ANGULAR_VELOCITY
        self.LINEAR_VELOCITY = feedback_enc.linear.x
        self.ANGULAR_VELOCITY = feedback_enc.angular.z

    #####################################################
    #            Controller Function                    #
    #####################################################
    def controller(self):
            # # global parameters
            # pi = 3.14
            # control_frequency = 10
            # ticks_per_rev = 950*3

            # # vehicle parameters
            # dt = 0.1
            # base = 0.24
            # wheel_radius = 0.0485

            # error integral part
            int_error_left = 0.0
            int_error_right = 0.0

            # # PID parameters
            # Kp_left = 30.0
            # Kp_right = 40.0
            # Ki_left = 400.0
            # Ki_right = 400.0
            # Kd_left = 0
            # Kd_right = 0

            PWM = std_msgs.msg.Float32()

            while not rospy.is_shutdown():
                #####################################################
                #            Left Wheels                           #
                #####################################################
                estimated_w = (self.ENCODER_LEFT * 2 * np.pi *
                            self.control_frequency) / (self.ticks_per_rev)
                desired_w = 0.33*0.25*(self.LINEAR_VELOCITY - (self.base / 2.0)
                                    * self.ANGULAR_VELOCITY) / self.wheel_radius

                print("est,desired left", estimated_w, desired_w)
                error = desired_w - estimated_w
                print("Error left", error)

                int_error_left = int_error_left + error * self.dt

                PWM_LEFT = (int)(self.Kp_left * error + self.Ki_left * int_error_left)

                #####################################################
                #            Right Wheels                           #
                #####################################################

                estimated_w = (self.ENCODER_RIGHT * 2 * np.pi *
                            self.control_frequency) / (self.ticks_per_rev)
                desired_w = 0.33*0.25*(self.LINEAR_VELOCITY + (self.base / 2.0)
                                    * self.ANGULAR_VELOCITY) / self.wheel_radius
                print("est,desired right", estimated_w, desired_w)

                error = desired_w - estimated_w
                print("Error right", error)

                int_error_right = int_error_right + error * dt

                PWM_RIGHT = (int)(self.Kp_right * error + self.Ki_right * int_error_right)

                print("encoder ", self.ENCODER_LEFT, self.ENCODER_RIGHT)
                print("PWM", PWM_LEFT, PWM_RIGHT)

                if (abs(self.LINEAR_VELOCITY) < 0.001 and abs(self.ANGULAR_VELOCITY) < 0.001):
                    PWM_LEFT = 0
                    PWM_RIGHT = 0

                PWM.data = PWM_LEFT
                self.pub_LEFT_MOTOR.publish(PWM)
                PWM.data = -PWM_RIGHT
                self.pub_RIGHT_MOTOR.publish(PWM)
                self.rate.sleep()



#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        mc = motor_controller()
        rospy.Subscriber('/left_motor/encoder',
                 phidgets.msg.motor_encoder, mc.update_feedback_enc_left)
        rospy.Subscriber('/right_motor/encoder',
                        phidgets.msg.motor_encoder, mc.update_feedback_enc_right)
        rospy.Subscriber('/keyboard/vel', geometry_msgs.msg.Twist,
                        mc.update_feedback_keyboard_vel)

        mc.controller()
    except rospy.ROSInterruptException:
        pass
