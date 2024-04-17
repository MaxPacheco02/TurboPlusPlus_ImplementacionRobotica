#!/usr/bin/env python3
''' ----------------------------------------------------------------------------
 * @file: speech_node.py
 * @date: March 4, 2023
 * @author: Max Pacheco
 *
 * -----------------------------------------------------------------------------
'''

import os
import csv
import math 

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ament_index_python.packages import get_package_share_directory

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Vector3, Pose, PoseStamped, TransformStamped
from std_msgs.msg import ColorRGBA, Int8, Bool
from nav_msgs.msg import Path
from dexterous_msgs.msg import VoiceCommand
from visualization_msgs.msg import Marker, MarkerArray

import speech_recognition as sr
import pyaudio
import numpy as np

def normalize(arr):
    return arr / np.sqrt(np.sum(arr**2))

class SpeechNode(Node):

    shape_option = 0
    drawing = MarkerArray()
    i = 0

    def __init__(self):
        super().__init__('speech_node')
        
        self.transform_pub_ = self.create_publisher(VoiceCommand, '/transform', 10)

        self.new_command_sub_ = self.create_subscription(
            Bool,
            '/new_command',
            self.new_command_callback,
            10)
        
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        from_frame_rel = 'world'
        to_frame_rel = 'link_eef'
        pass

        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         to_frame_rel,
        #         from_frame_rel,
        #         rclpy.time.Time())
        #     print(f'Pose: x = {t.transform.translation.x}, y = {t.transform.translation.y}, z = {t.transform.translation.z}')
            
        #     if t.transform.translation.z >= Z_OFFSET - 0.001:
        #         dot = Marker()
        #         dot.color = ColorRGBA()
        #         dot.color.r = 1.0
        #         dot.color.g = 0.0
        #         dot.color.b = 0.0
        #         dot.color.a = 1.0
                
        #         dot.header.frame_id = 'world'
        #         dot.id = self.i
        #         self.i+=1
        #         dot.type = 2
        #         dot.action = 0
        #         dot.scale = Vector3()
        #         dot.scale.x = 0.01
        #         dot.scale.y = 0.01
        #         dot.scale.z = 0.01
        #         dot.pose.position.x = -t.transform.translation.x
        #         dot.pose.position.y = t.transform.translation.y
        #         dot.pose.position.z = t.transform.translation.z - 0.01
        #         self.drawing.markers.append(dot)
        #         self.marker_pub_.publish(self.drawing)

        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        #     return
    
    def new_command_callback(self, msg):
        recognized = False
        r= sr.Recognizer()
        with sr.Microphone() as source:
            self.get_logger().error(
                'Speak the new command')
            r.pause_threshold = 1
            r.adjust_for_ambient_noise(source)
            audio = r.listen(source)                    
            try:
                phrase = (r.recognize_google(audio).lower())
                self.get_logger().error(
                    'You said %f', phrase)

                # opt = ''
                # if(phrase == 'uno' or phrase == '1'):
                #     opt = 'cone'
                # if(phrase == 'dos' or phrase == '2'):
                #     opt = 'cube'
                # if(phrase == 'tres' or phrase == '3'):
                #     opt = 'cylinder'
                # if(phrase == 'cuatro' or phrase == '4'):
                #     opt = 'hexagonal_prism'
                # if(phrase == 'cinco' or phrase == '5'):
                #     opt = 'prism'
                #     FIG_SCALE = 0.1
                # if(phrase == 'seis' or phrase == '6'):
                #     opt = 'sphere'
                #     Z_ADDER = 1
                # if(phrase == 'siete' or phrase == '7'):
                #     opt = 'squared_pyramid'
                # if(phrase == 'ocho' or phrase == '8'):
                #     opt = 'triangular_prism'
                # if(phrase == 'nueve' or phrase == '9'):
                #     opt = 'triangular_pyramid'
                
                # if opt != '':
                # else:
                #     print('Try again!')
            except sr.UnknownValueError:
                self.get_logger().error(
                    'Your last command couldn\'t be heard')


def main(args=None):
    rclpy.init(args=args)

    speech_node = SpeechNode()
    rclpy.spin(speech_node)
    speech_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

