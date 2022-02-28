#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

IMU_OUT = ''
ODOM_OUT = ''
GPS_OUT = ''

IMU_IN = ''
ODOM_IN = ''
GPS_IN = ''


class Converter:
    def __init__(self):
        self.publisher_imu = rospy.Publisher(IMU_OUT, Imu, queue_size=10)
        self.publisher_odom = rospy.Publisher(ODOM_OUT, Odometry, queue_size=10)
        self.publisher_gps = rospy.Publisher(GPS_OUT, NavSatFix, queue_size=10)
        
        rospy.Subscriber(IMU_IN, Imu, self.imu_callback)
        rospy.Subscriber(ODOM_IN, Odometry, self.odom_callback)
        rospy.Subscriber(GPS_IN, NavSatFix, self.gps_callback)


    def odom_callback(self, data):
        data.header.frame_id = 'odom'
        data.child_frame_id = 'base_link'
        self.publisher_odom.publish(data)


    def imu_callback(self, data):
        data.header.frame_id = 'base_link'
        self.publisher_imu.publish(data)
    

    def gps_callback(self, data):
        data.header.frame_id = 'base_link'
        self.publisher_gps.publish(data)


if __name__ == '__main__':
    rospy.init_node('converter', anonymous=True)
    
    con = Converter()

    rospy.spin()
