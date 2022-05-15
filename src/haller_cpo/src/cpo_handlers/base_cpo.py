import cv2
from typing import List, Dict, Tuple

"""
This class provides base functionality for interacting with ros cpo handler
Use this class as 
"""


class BaseCPO:
    def __int__(self):
        self.camera_image = None
        self.depth_image = None
        self.bounding_boxes = None

    def update(self):
        """
        override this method to perform realtime cpo
        """
        pass
