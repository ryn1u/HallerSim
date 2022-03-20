#!/usr/bin/env python
from distutils.command.config import config

from simplejson import load
import rospy
from haller_sim import load_config_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class SimSensors:
    def imu_data_callback(self, data):
        pass
    def dvl_data_callback(self, data):
        pass

    def listener(self):
        config = load_config_data()
        sim_imu_topic = config['imuROSTopic']
        sim_dvl_topic = config['dvlROSTopic']

        rospy.init_node('sim_sensors', anonymous=True)
        rospy.Subscriber(sim_imu_topic, Imu, self.imu_data_callback)
        rospy.Subscriber(sim_dvl_topic, Vector3, self.dvl_data_callback)

        rospy.spin()


if __name__ == '__main__':
    sim_sensors = SimSensors()
    sim_sensors.listener()
