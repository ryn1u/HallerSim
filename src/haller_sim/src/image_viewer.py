#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
import haller_sim


class ImageViewer:
    def __init__(self) -> None:
        config = haller_sim.load_config_data()
        image_publisher_topic = config["videoImageTopic"]
        self.simulation_video_topic = config["videoROSTopic"]
        self.br = CvBridge()
        self.pub = rospy.Publisher(image_publisher_topic, Image, queue_size=10)
        self.depth_pub = rospy.Publisher("auvDepthImage", Image, queue_size=10)

    def convert_image_msg(self, data):
        cv2_img = self.br.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.pub.publish(self.br.cv2_to_imgmsg(cv2_img, encoding='bgr8'))

    def convert_depth_img(self, data):
        cv2_img = self.br.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.depth_pub.publish(self.br.cv2_to_imgmsg(cv2_img, encoding='bgr8'))

    def listener(self):
        rospy.init_node('image_viewer', anonymous=True)
        rospy.Subscriber(self.simulation_video_topic, CompressedImage, self.convert_image_msg)
        rospy.Subscriber("sim_depth_image", CompressedImage, self.convert_depth_img)

        rospy.spin()


if __name__ == '__main__':
    image_viewer = ImageViewer()
    image_viewer.listener()
