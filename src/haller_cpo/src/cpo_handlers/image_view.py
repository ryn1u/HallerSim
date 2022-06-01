from .base_cpo import BaseCPO
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist

"""
Example cpo handler class
"""

ENABLED = True
POSITION_TOPIC = ("/AuvInternalSystem/Position/globalEstimatedPosition", Twist)


class ImageView(BaseCPO):
    def __init__(self):
        super().__init__()
        self.required_topics = [
            POSITION_TOPIC
        ]

    def update(self):
        if self.camera_image is not None and ENABLED:
            cv2.imshow("image", self.camera_image)
            cv2.waitKey(1)

        # Example of how to send data to ros topics
        if self.bounding_boxes is not None and ENABLED:
            topic = "TestTopic"
            msg = String(str(self.bounding_boxes))
            self.enqueue_ros_msg(topic, msg)

        # Example of how to receive data from any ros topic
        topic_name = POSITION_TOPIC[0]
        if self.received_msgs[topic_name] is not None:
            print(self.received_msgs[topic_name])
