import cv2
from typing import *
from collections import deque
from genpy import Message
from geometry_msgs.msg import Twist, Vector3

"""
This class provides base functionality for interacting with ros cpo handler
Use this class as 
"""


class BaseCPO:
    def __init__(self):
        self.camera_image = None
        self.depth_image = None
        self.bounding_boxes = None
        # list of pairs of required topic names and associated classes
        self.required_topics: List[Tuple[str, Type[Message]]] = list()
        # dict of topic name to last received msg
        self.received_msgs: dict = dict()
        self.msgs_to_send: Dict[str, Deque[Message]] = dict()

    def update(self):
        """
        override this method to perform realtime cpo
        """
        pass

    def enqueue_ros_msg(self, topic: str, msg: Message):
        """
        Enqueues a ros message of particular ros topic to be sent at next update loop.
        """
        if topic not in self.msgs_to_send:
            self.msgs_to_send[topic] = deque()

        self.msgs_to_send[topic].append(msg)
