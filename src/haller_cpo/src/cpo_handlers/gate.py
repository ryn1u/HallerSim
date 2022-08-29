from .base_cpo import BaseCPO
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

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

    def wait(self, global_cor_now, global_cor_end):
        time.sleep(10)
        print("next")
        return 1

    def update(self):
        if self.camera_image is not None and ENABLED:
            frame = self.camera_image
            depth = self.depth_image
            bounds = self.bounding_boxes

            # Phase 1 - rotating to find gate
            if bounds is not None:
                if any(bound.Class == "gman" for bound in bounds):
                    for i in range(0, len(bounds)):
                        if bounds[i].Class == "gman":
                            ymin = bounds[i].ymin
                            ymax = bounds[i].ymax
                            ycen = int((ymin + ymax) / 2)
                            if ycen >= self.ycen_of_view - 50 and ycen >= self.ycen_of_view + 50:
                                gate_found = 1
                            elif ycen <= self.ycen_of_view - 50:
                                self.global_cor_end['yaw'] += 15
                            elif ycen >= self.ycen_of_view + 50:
                                self.global_cor_end['yaw'] -= 15
                else:
                    self.global_cor_end['yaw'] += 30

            """
            if bounds is not None:
                for i in range(0, len(bounds)):
                    if bounds[i].Class == "gman":
                        xmin = bounds[i].xmin
                        ymin = bounds[i].ymin
                        xmax = bounds[i].xmax
                        ymax = bounds[i].ymax
                        xcen = int((xmin + xmax) / 2)
                        ycen = int((ymin + ymax) / 2)
                        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                        cv2.circle(frame, (xcen, ycen), 5, (0, 0, 255), 5)

                        if self.global_cor_now == self.global_cor_end:
                            # x, y centering
                            if xcen < self.xcen_of_view:
                                self.global_cor_end['x'] = self.global_cor_end['x'] + self.min_step
                            elif xcen > self.xcen_of_view:
                                self.global_cor_end['x'] = self.global_cor_end['x'] - self.min_step
                            if ycen < self.ycen_of_view:
                                self.global_cor_end['y'] = self.global_cor_end['y'] + self.min_step
                            elif ycen > self.ycen_of_view:
                                self.global_cor_end['y'] = self.global_cor_end['y'] - self.min_step
            """

            cv2.imshow("image", frame)
            cv2.waitKey(100)


"""
        # Example of how to send data to ros topics
        if self.bounding_boxes is not None and ENABLED:
            topic = "TestTopic"
            msg = String(str(self.bounding_boxes))
            self.enqueue_ros_msg(topic, msg)

        # Example of how to receive data from any ros topic
        topic_name = POSITION_TOPIC[0]
        if self.received_msgs[topic_name] is not None:
            print(self.received_msgs[topic_name])
            """
