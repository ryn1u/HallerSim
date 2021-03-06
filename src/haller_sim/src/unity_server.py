#!/usr/bin/env python

from distutils.command.config import config
import rospy
import json

from haller_sim import load_config_data
from ros_tcp_endpoint import TcpServer
from ros_tcp_endpoint.subscriber import RosSubscriber
from ros_tcp_endpoint.publisher import RosPublisher
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import CompressedImage, Imu


def main():
    config_data = load_config_data()
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", "TCPServer")
    ip = config_data["rosIP"]
    tcp_server = TcpServer(ros_node_name, tcp_ip=ip, tcp_port=10000)

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start(
        {
            'haller_move': RosSubscriber(config_data["rovTransformTopic"], Twist, tcp_server),
            'sim_image': RosPublisher('sim_image', CompressedImage),
            'sim_imu': RosPublisher('sim_imu', Imu),
            'sim_dvl': RosPublisher('sim_dvl', Vector3),
            'sim_depth_image': RosPublisher('sim_depth_image', CompressedImage)
        }
    )
    rospy.spin()


if __name__ == "__main__":
    main()
