#!/usr/bin/env python
import sys
sys.path.append("/home/isat/juanantonio/davinci_catkin_ws_1.7/src/dvrk-ros/dvrk_robot/scripts/thesis_automation/vision_modules/")

import json
import numpy as np 
#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf_conversions.posemath as pm
import PyKDL
#Ros messages
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Int32, Int32MultiArray, Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy
from std_msgs.msg import String as rosString
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
#Python libraries
import cv2 
import time
import datetime
import numpy as np 
import sys
import random 
import argparse
import json
import socket
import os
from os.path import join, exists

#custom 
from recording_module import createTimeStamp, recording_module


class socket_module:


    def __init__(self, debug_mode=False):
        

        if debug_mode:
            self.client_socket = self.create_fake_socket()
        else:
            self.client_socket = self.start_socket()


        #############
        #Subscribers#
        #############
        ##Pedals
        self.bicoag_sub = rospy.Subscriber("/dvrk/footpedals/bicoag", Joy, self.bicoag_callback)#This is only for testing purposes. It should be disabled on the real-experiment

        #############
        #Publishers #
        #############
        self.is_recording_active_pub = rospy.Publisher("/recording_module/recording_active",Bool,queue_size=5) 
        self.is_autonomy_active_pub = rospy.Publisher("/recording_module/autonomy_active" ,Bool,queue_size=5) 
        self.autonomy_trigger_pub = rospy.Publisher("/recording_module/cognitive_trigger",Bool, queue_size=5)
        
    #############
    #Callbacks  #
    #############
    def bicoag_callback(self, data):
        """
        Trigger autonomy by tapping on the bicoag pedal
        This is only for testing purposes
        """
        if  data.buttons[0]:
            self.autonomy_trigger_pub.publish(True)
            
    def run_main(self):
        while True:
            data = ""
            while not data:
                try:
                    data = self.client_socket.recv(1024).decode()  # receive response
                except socket.timeout as e:
                    time.sleep(0.5)
          
            if data == "close":
                break
            else:
                answer = self.do_command(data)

            print('Received from server: ' + data)  # show in terminal
            message = answer  # again take input
            self.client_socket.send(message.encode())  # send message

        self.client_socket.close()  # close the connection
        print("Module closed restart for new experiment.")

    def do_command(self, cmd):
        
        if cmd == 'start experiment':
            answer = 'started experiment (DV)'
            self.is_recording_active_pub.publish(True)
        elif cmd == 'end experiment':
            answer = 'ended experiment (DV)'
            self.is_recording_active_pub.publish(False)
        elif cmd == 'turn on autonomy':
            answer = 'autonomy on (DV)'
            self.is_autonomy_active_pub.publish(True) 
        elif cmd == 'turn off autonomy':
            answer = 'autonomy off (DV)'
            self.is_autonomy_active_pub.publish(False)
        elif cmd == 'cognitive trigger':
            answer = 'executing a suction'
            self.autonomy_trigger_pub.publish(True)
        else:
            answer = 'cmd not recognized'

        return answer 

    def start_socket(self):
        host = "169.254.217.30"  # as both code is running on same pc
        port = 5555  # socket server port number
        client_socket = socket.socket()  # instantiate
        
        client_socket.settimeout(2)
        count = 0
        while client_socket.connect_ex((host, port)) != 0:
            try:
                print("{:d} Server not available".format(count))
                count += 1
                time.sleep(2)
            except KeyboardInterrupt as e:
                print(e)
                exit(0)

        return client_socket
    def create_fake_socket(self):
        """
        Only for debugging purposes
        """
        class FakeSocket:
            def __init__(self):
                pass
            def recv(self,x):
                return ''
        return FakeSocket()
    def close_files(self):
        pass


def main():
    args =  script_config_arg()
    rospy.init_node('socket_module')

    ts = createTimeStamp()
   
    cm = socket_module(debug_mode=args.debug_mode)
    recording_m = recording_module(dst_path = ts+args.dir_name)

    #Sleep until the subscribers are ready.
    time.sleep(0.20)

    cm.run_main()

    try:
        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.25)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cm.close_files()
        print("Shutting down")

def script_config_arg():
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument('-d', '--dir_name',required=True, help='name of directory, format "Name_Session"')
    parser.add_argument('--debug_mode', action='store_true', default=False, help='activate debug mode of socket module, i.e., create the module without having a realsocket(default: disabled)')

    args = parser.parse_args()
    return args 

if __name__ == '__main__':
    main()
