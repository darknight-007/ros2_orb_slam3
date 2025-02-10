#!/usr/bin/env python3

"""
Python node for the MonocularMode cpp node.
Modified to work with ZED2 camera live input.

Author: Azmyin Md. Kamal
Date: 01/01/2024

Requirements
* Dataset must be configured in EuRoC MAV format
* Paths to dataset must be set before bulding (or running) this node
* Make sure to set path to your workspace in common.hpp

Command line arguments
-- settings_name: EuRoC, TUM2, KITTI etc; the name of the .yaml file containing camera intrinsics and other configurations
-- image_seq: MH01, V102, etc; the name of the image sequence you want to run

"""

# Imports
#* Import Python modules
import sys # System specific modules
import os # Operating specific functions
import glob
import time # Python timing module
import copy # For deepcopying arrays
import shutil # High level folder operation tool
from pathlib import Path # To find the "home" directory location
import argparse # To accept user arguments from commandline
import natsort # To ensure all images are chosen loaded in the correct order
import yaml # To manipulate YAML files for reading configuration files
import copy # For making deepcopies of openCV matrices, python lists, numpy arrays etc.
import numpy as np # Python Linear Algebra module
import cv2 # OpenCV

#* ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# If you have more files in the submodules folder
# from .submodules.py_utils import fn1 # Import helper functions from files in your submodules folder

# Import a custom message interface
# from your_custom_msg_interface.msg import CustomMsg #* Note the camel caps convention

# Import ROS2 message templates
from sensor_msgs.msg import Image # http://wiki.ros.org/sensor_msgs
from std_msgs.msg import String, Float64 # ROS2 string message template
from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array

#* Class definition
class MonoDriver(Node):
    def __init__(self, node_name = "mono_py_node"):
        super().__init__(node_name)

        # Initialize parameters
        self.declare_parameter("settings_name", "ZED2")
        
        # Parse values
        self.settings_name = str(self.get_parameter('settings_name').value)

        # DEBUG
        print(f"-------------- Received parameters --------------------------\n")
        print(f"self.settings_name: {self.settings_name}")
        print()

        # Global variables
        self.node_name = "mono_py_node"
        self.frame_id = 0
        self.frame_count = 0

        # Define a CvBridge object
        self.br = CvBridge()

        # ROS2 publisher/subscriber variables
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.zed_image_topic = "/zed/zed_node/left/image_rect_color"
        self.send_config = True
        
        # Setup ROS2 publishers and subscribers
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.exp_config_msg = self.settings_name
        print(f"Configuration to be sent: {self.exp_config_msg}")

        self.subscribe_exp_ack_ = self.create_subscription(String, 
                                                       self.sub_exp_ack_name, 
                                                       self.ack_callback, 10)
        
        # Publisher to send RGB image and timestep
        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, 1)
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)
        
        # Subscribe to ZED2 camera
        self.zed_subscription = self.create_subscription(
            Image,
            self.zed_image_topic,
            self.zed_callback,
            10)

        print(f"MonoDriver initialized, attempting handshake with CPP node")

    def ack_callback(self, msg):
        if(msg.data == "ACK"):
            self.send_config = False
            
    def zed_callback(self, msg):
        """
        Callback for ZED2 camera images
        """
        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(msg)
            
            # Create timestep message
            timestep_msg = Float64()
            timestep_msg.data = float(self.get_clock().now().nanoseconds) / 1e9
            
            # Publish timestep and image
            self.publish_timestep_msg_.publish(timestep_msg)
            self.publish_img_msg_.publish(msg)
            
            self.frame_id += 1
            
        except CvBridgeError as e:
            print(e)
    
    def handshake_with_cpp_node(self):
        if (self.send_config == True):
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

def main(args = None):
    rclpy.init(args=args)
    n = MonoDriver("mono_py_node")
    
    # Blocking loop to initialize handshake
    while(n.send_config == True):
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)
        if(n.send_config == False):
            break
        
    print(f"Handshake complete")
    
    # Spin node
    rclpy.spin(n)
    
    # Cleanup
    n.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
