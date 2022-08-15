#!/usr/bin/env python

from distutils.command.config import config
import rospy
import json

from haller_sim import load_config_data
from ros_tcp_endpoint import TcpServer
from ros_tcp_endpoint.subscriber import RosSubscriber
from ros_tcp_endpoint.publisher import RosPublisher
from std_msgs.msg import Float32MultiArray
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
            config_data["simTargetPosition"]: RosSubscriber(config_data["simTargetPosition"], Twist, tcp_server),
            config_data["simCurrentPosition"]: RosPublisher(config_data["simCurrentPosition"], Twist),
            config_data['simVideoImage']: RosPublisher(config_data['simVideoImage'], CompressedImage),
            config_data['simDepthImage']: RosPublisher(config_data['simDepthImage'], CompressedImage),
            config_data['simDistanceMeasures']: RosPublisher(config_data['simDistanceMeasures'], Float32MultiArray)
        }
    )
    rospy.spin()


if __name__ == "__main__":
    main()
