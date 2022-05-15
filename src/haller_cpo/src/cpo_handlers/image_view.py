from .base_cpo import BaseCPO
import cv2

"""
Example cpo handler class
"""

ENABLED = False


class ImageView(BaseCPO):
    def __init__(self):
        super().__int__()

    def update(self):
        if self.camera_image is not None and ENABLED:
            cv2.imshow("image", self.camera_image)
            cv2.waitKey(1)
