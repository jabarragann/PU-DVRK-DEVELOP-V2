#!/usr/bin/env python

import sys
import numpy as np
import time

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Joy
from diagnostic_msgs.msg import KeyValue

class Arm_swaper:

    def __init__(self):
       
        # subscribed Topic
        cam_minus_topic = "/dvrk/footpedals/cam_minus"
        select_teleop_topic = "/dvrk/console/teleop/select_teleop_psm"

        self.subscriber = rospy.Subscriber(cam_minus_topic, Joy, self.pedal_callback,  queue_size = 5)
        self.publisher  = rospy.Publisher(select_teleop_topic,KeyValue, queue_size=5)

        self.mtmr_key_value_message = KeyValue()
        self.mtmr_key_value_message.key = "MTMR"
        self.mtmr_key_value_message.value = ""
        
        self.mtml_key_value_message = KeyValue()
        self.mtml_key_value_message.key = "MTML"
        self.mtml_key_value_message.value = ""

        self.state = 1

    def pedal_callback(self, ros_data):
        if ros_data.buttons[0]:
            #print('state', self.state)
            self.unselect_mtm_psm_pairs()
            self.swap_arms()
        
    def unselect_mtm_psm_pairs(self):
        self.mtml_key_value_message.value = ""
        self.mtmr_key_value_message.value = ""
        
        self.publish_current_state()

    def swap_arms(self):
        if self.state == 1:
            self.mtml_key_value_message.value = "PSM1"
            self.mtmr_key_value_message.value = "PSM2"
            self.state = 2
        elif self.state == 2:
            self.mtml_key_value_message.value = "PSM2"
            self.mtmr_key_value_message.value = "PSM1"
            self.state =1

        self.publish_current_state()

    def publish_current_state(self):
    	self.publisher.publish(self.mtmr_key_value_message)
        time.sleep(0.25)
        self.publisher.publish(self.mtml_key_value_message)
        time.sleep(0.25)

def main():
    '''Initializes and cleanup ros node'''
    rospy.init_node('arm_swaper', anonymous=True)
    time.sleep(0.250)

    arm_swaper = Arm_swaper()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting swapping module"
    
    print("Close system")



if __name__ == '__main__':
    main()