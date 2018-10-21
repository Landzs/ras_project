#!/usr/bin/env python

import rospy
import smach
import smach_ros
#import numpy as np
import math
#import matplotlib.pyplot as plt
import rospy
import std_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import itertools
import tf
import time
#from arduino_servo_control.srv import *
#####################################################
#                  Initialization                   #
#####################################################
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Initialization Done'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialization')
        return 'Initialization Done'


#####################################################
#                    Standby                       #
#####################################################
class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Receive Start Message'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Standby')
        return 'Receive Start Message'
        
#####################################################
#                    Standby                       #
#####################################################
class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Receive Start Message'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Standby')
        return 'Receive Start Message'



def main():
    rospy.init_node('state_machine_node')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Stop'])
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Initialization', Initialization(), 
                               transitions={'Initialization Done':'Standby'})
        smach.StateMachine.add('Standby', Standby(), 
                               transitions={'Receive Start Message':'Stop'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
