#!/usr/bin/env python
import rospy
from typing import *

import utils
from utils.base_ros_handler import BaseROSHandler

from geometry_msgs.msg import Twist, Vector3
from detection_msgs.msg import BoundingBox

import cv2
import numpy as np


"""
SIMULATION STARTING SEQUENCE:
1. Make sure simulationConfig.json files in Unity and ROS are the same and are latest version.
2. Start ROS server with:
    cd *insert path*/HallerSim
    source devel/setup.bash
    roslaunch haller_cpo simulation.launch
3. Start Unity
4. Check if IP is correct and click 'Connect to ROS'
5. Click Start Simulation (Screen might turn black and white! It's OK!
6. Run this file (preferably use pyCharm)

7. (optional) Check if data is flowing through topics:
    - for text:
        'rostopic list'
        'rostopic echo *topic name*'
    - for images:
        'rosrun rqt_image_viewer rqt_image_viewer'
"""


class HallerCPO(BaseROSHandler):
    """
    Class for handling CPO. BaseROSHandler does basic ROS system initialization and basic depth data processing.
    See BaseROSHandler code in utils/base_ros_handler.py
    """
    def __init__(self):
        super().__init__()
        # Here goes all initialization code you need to implement.

    def update(self, current_position, video_image, depth_data, bounding_boxes: List[BoundingBox]):
        # here goes CPO code. this method is called every second
        # angles are in degrees
        x, y, z, roll, pitch, yaw = current_position

        pass

    def send_target_position(self, x, y, z, roll, pitch, yaw):
        # Sends new target position to simulation. Remember to use North-East-Down coordinates
        msg = Twist(Vector3(x, y, z), Vector3(roll, pitch, yaw))
        self.publishers['simTargetPosition'].publish(msg)


if __name__ == '__main__':
    haller_cpo_handler = HallerCPO()
    haller_cpo_handler.start()
