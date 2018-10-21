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
import threading
#from arduino_servo_control.srv import *
GRIP = False
START = False



#####################################################
#                  Initialization                   #
#####################################################
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Initialization Done'])

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
        global START
        rospy.loginfo('Executing state Standby')
        while not START:
            pass
        return 'Receive Start Message'



        
#####################################################
#                Look For Object                   #
#####################################################
class Look_For_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Detected Object'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Look_For_Object')
        return 'Detected Object'


#####################################################
#                 Follow_Path                       #
#####################################################
class Follow_Path(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Reached Gripping Target','Reached Releasing Target'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Follow_Path')
        if not GRIP:
            return 'Reached Gripping Target'
        else:
            return 'Reached Releasing Target'

#####################################################
#                 Grip_Object                       #
#####################################################
class Grip_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Gripped Object'])

    def execute(self, userdata):
        global GRIP
        rospy.loginfo('Executing state Grip_Object')
        GRIP = True
        return 'Gripped Object'

#####################################################
#                Release_Object                     #
#####################################################
class Release_Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Released Object'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Release_Object')
        return 'Released Object'
#####################################################
#                  Feedback_Start                   #
#####################################################
def feedback_start(msg):
    global START
    print("#")
    if msg.data == True:
        START = True
        print(START)
    else :
        pass



def main():
    rospy.init_node('state_machine_node')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Stop'])
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    rospy.Subscriber('/Start', std_msgs.msg.Bool, feedback_start)
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Initialization', Initialization(), 
                               transitions={'Initialization Done':'Standby'})

        smach.StateMachine.add('Standby', Standby(), 
                               transitions={'Receive Start Message':'Look_For_Object'})

        smach.StateMachine.add('Look_For_Object', Look_For_Object(), 
                               transitions={'Detected Object':'Follow_Path'})

        smach.StateMachine.add('Follow_Path', Follow_Path(), 
                               transitions={'Reached Gripping Target':'Grip_Object', 'Reached Releasing Target':'Release_Object'})

        smach.StateMachine.add('Grip_Object', Grip_Object(), 
                               transitions={'Gripped Object':'Follow_Path'})
                               
        smach.StateMachine.add('Release_Object', Release_Object(), 
                               transitions={'Released Object':'Stop'})                             
    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
