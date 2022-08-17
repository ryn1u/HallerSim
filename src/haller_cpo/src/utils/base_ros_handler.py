import rospy
from cv_bridge import CvBridge
from typing import *

import utils

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist

import cv2
import numpy as np
from threading import Thread

# ROS Image topics from haller_sim/image_viewer.py
ROS_VIDEO_IMAGE_TOPIC_NAME = "ros_auv_video_image"
ROS_DEPTH_IMAGE_TOPIC_NAME = "ros_auv_depth_image"

# Definitions of types for topics from auvConfig/simulationConfig.json
DISTANCE_MEASURES_TOPIC = ("simDistanceMeasures", Float32MultiArray)
TARGET_POSITION_TOPIC = ("simTargetPosition", Twist)
DELTA_POSITION_TOPIC = ("simDeltaPosition", Twist)
CURRENT_POSITION_TOPIC = ("simCurrentPosition", Twist)

# Topics which are NOT defined in auvConfig/simulationConfig.json
EXTRA_SUBSCRIBER_TOPICS = {
    "videoImage": (ROS_VIDEO_IMAGE_TOPIC_NAME, Image),
    "depthImage": (ROS_DEPTH_IMAGE_TOPIC_NAME, Image),
    "boundingBoxes": ("/yolov5/detections", BoundingBoxes)
}

SIMULATION_CONFIG = utils.load_config_data()


class BaseROSHandler:
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.subscribers, self.publishers = self.init_ros()
        self.rate: rospy.Rate = None

        self.bounding_boxes = None
        self.current_position = None
        self.video_image = None
        self.depth_image = None
        self.depth_data = None

    def init_ros(self) -> Tuple[Dict[str, rospy.Subscriber], Dict[str, rospy.Publisher]]:
        subscriber_topics = [
            # DISTANCE_MEASURES_TOPIC,
            CURRENT_POSITION_TOPIC
        ]

        subscribers = {
            topic_handle: rospy.Subscriber(
                SIMULATION_CONFIG[topic_handle], topic_type,
                self.__getattribute__(f"update_{utils.parse_camel_to_snake_case(topic_handle)}")
            ) for (topic_handle, topic_type) in subscriber_topics
        }

        extra_subscribers = {
            topic_handle: rospy.Subscriber(
                topic_name, topic_type,
                self.__getattribute__(f"update_{utils.parse_camel_to_snake_case(topic_handle)}")
            ) for topic_handle, (topic_name, topic_type) in EXTRA_SUBSCRIBER_TOPICS.items()
        }

        subscribers = {**subscribers, **extra_subscribers}

        publisher_topics = [
            TARGET_POSITION_TOPIC,
            DELTA_POSITION_TOPIC
        ]
        publishers = {
            topic_handle: rospy.Publisher(SIMULATION_CONFIG[topic_handle], topic_type, queue_size=10)
            for (topic_handle, topic_type) in publisher_topics
        }

        return subscribers, publishers

    def update_depth_image(self, data: Image):
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        self.depth_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        self.depth_data = BaseROSHandler.approx_distance(self.depth_image)
        pass

    def update_video_image(self, data: Image):
        self.video_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')

    def update_sim_current_position(self, data: Twist):
        # x, y, z, roll, pitch, yaw
        self.current_position = (
            data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z
        )

    def update_bounding_boxes(self, data: BoundingBoxes):
        self.bounding_boxes = data.bounding_boxes

    @staticmethod
    def approx_distance(x):
        a = 28125.11707
        b = -2.03966
        return a * np.power(x, b)

    def start(self):
        rospy.init_node("haller_cpo", anonymous=True)
        self.rate = rospy.Rate(1)
        worker = Thread(target=self._update)
        worker.start()
        rospy.spin()

    def _update(self):
        while not rospy.is_shutdown():
            if all([v is not None for v in [self.current_position, self.video_image, self.depth_data, self.bounding_boxes]]):
                self.update(
                    self.current_position,
                    self.video_image,
                    self.depth_data,
                    self.bounding_boxes
                )
            self.rate.sleep()

    def update(self, current_position, video_image, depth_data, bounding_boxes: BoundingBoxes):
        raise NotImplemented
