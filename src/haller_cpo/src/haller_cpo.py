#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from typing import *

from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBoxes

from cpo_handlers.base_cpo import BaseCPO
import os
import importlib
import inspect
from copy import copy
from threading import Thread

"""
This class is supposed to fetch image data (camera frames, yolo detections, depth image) and pass it to CPO programs
inside this package.

It should give:
cv frames of depth image and camera image
dict with bounding boxes from yolo
"""

DEPTH_IMAGE_TOPIC = ("auvDepthImage", Image)
CAMERA_IMAGE_TOPIC = ("auvCameraImage", Image)
BOUNDING_BOX_TOPIC = ("/yolov5/detections", BoundingBoxes)
TOPICS = {
    "camera_image": CAMERA_IMAGE_TOPIC,
    "depth_image": DEPTH_IMAGE_TOPIC,
    "bounding_boxes": BOUNDING_BOX_TOPIC
}


class HallerCPO:
    def __init__(self):
        self.handlers: List[BaseCPO] = list()
        self.cv_bridge = CvBridge()
        self.subscribers = {
            topic_name: rospy.Subscriber(topic_name, message_type, self.__getattribute__(f"relay_{field_name}"))
            for field_name, (topic_name, message_type) in TOPICS.items()
        }
        self.publishers: Dict[str, rospy.Publisher] = {}
        self.rate: rospy.Rate = None

    def relay_camera_image(self, image: Image):
        cv2_img = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        for handler in self.handlers:
            handler.camera_image = copy(cv2_img)

    def relay_depth_image(self, image: Image):
        cv2_img = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        for handler in self.handlers:
            handler.depth_image = copy(cv2_img)

    def relay_bounding_boxes(self, bounding_boxes: BoundingBoxes):
        for handler in self.handlers:
            handler.bounding_boxes = copy(bounding_boxes.bounding_boxes)

    def relay_msg_to_handlers(self, topic, data):
        for handler in self.handlers:
            if topic in handler.received_msgs.keys():
                handler.received_msgs[topic] = data

    def send_handler_msgs(self, handler: BaseCPO):
        for topic, msg_queue in handler.msgs_to_send.items():
            if topic not in self.publishers and len(msg_queue) > 0:
                self.publishers[topic] = rospy.Publisher(
                    topic,
                    msg_queue[0].__class__,
                    queue_size=10
                )

            while len(msg_queue) > 0:
                msg = msg_queue.pop()
                self.publishers[topic].publish(msg)

    def start_cpo(self):
        rospy.init_node("haller_cpo", anonymous=True)
        self.rate = rospy.Rate(1)
        worker = Thread(target=self.update_handlers)
        worker.start()
        rospy.spin()

    def update_handlers(self):
        while not rospy.is_shutdown():
            for handler in self.handlers:
                handler.update()
                self.send_handler_msgs(handler)
            self.rate.sleep()

    def add_required_handler_topics(self, handler: BaseCPO):
        for topic_name, topic_class in handler.required_topics:
            handler.received_msgs[topic_name] = None
            if topic_name not in self.subscribers:
                self.subscribers[topic_name] = rospy.Subscriber(
                    topic_name,
                    topic_class,
                    lambda data: self.relay_msg_to_handlers(topic_name, data)
                )

    def init_cpo_handlers(self):
        handlers_dir = os.path.join(os.path.dirname(__file__), "cpo_handlers")
        files = [f.replace(".py", "") for f in os.listdir(handlers_dir)
                 if f not in ['__init__.py', '__pycache__', 'base_cpo.py']]

        for file in files:
            module = importlib.import_module(f"cpo_handlers.{file}")
            for attr_name in dir(module):
                if attr_name == 'BaseCPO':
                    continue
                attribute = module.__getattribute__(attr_name)
                if inspect.isclass(attribute):
                    if issubclass(attribute, BaseCPO):
                        handler = attribute()
                        self.add_required_handler_topics(handler)
                        self.handlers.append(handler)
        print(f"Added CPO handlers: {' '.join([h.__class__.__name__ for h in self.handlers])}")


if __name__ == '__main__':
    haller_cpo_handler = HallerCPO()
    haller_cpo_handler.init_cpo_handlers()
    haller_cpo_handler.start_cpo()
