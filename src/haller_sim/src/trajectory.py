#!/usr/bin/env python

import rospy
from math import pi, sin, cos
from haller_sim import load_config_data
from geometry_msgs.msg import Twist, Vector3

def trajectory():
    config = load_config_data()
    sim_tranform_topic = config["rovTransformTopic"]
    pub = rospy.Publisher(sim_tranform_topic, Twist, queue_size=10)
    rospy.init_node('trajectory', anonymous=True)
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        time = rospy.get_time()
        translation = Vector3(5*sin(time), 5*cos(time), 1)
        rotation = Vector3(0, 0, 0)
        msg = Twist(translation, rotation)
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        trajectory()
    except rospy.ROSInterruptException:
        pass
