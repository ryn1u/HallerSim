#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class RovTransform:
    def rov_transform_callback(self, data):
        pass

    def listener(self):
        rospy.init_node('sim_rov_transform_node', anonymous=True)
        rospy.Subscriber('sim_rov_transform', Twist, self.rov_transform_callback)

        rospy.spin()


if __name__ == '__main__':
    rov_transform = RovTransform()
    rov_transform.listener()
