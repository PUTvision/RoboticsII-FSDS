#!/usr/bin/env python3

import math
from typing import List

import numpy as np
import rospy as rp
from ros_numpy import point_cloud2
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import PointCloud2


class LidarDetector:
    def __init__(self, cones_range_cutoff: float = 8.0):
        self.cones_range_cutoff = cones_range_cutoff

        rp.Subscriber("/fsds/lidar/Lidar", PointCloud2, self.lidar_detection_callback)

        self.cones_position_publisher = rp.Publisher('/fsds_roboticsII/cones_poses', PoseArray, queue_size=10)


    def lidar_detection_callback(self, data: PointCloud2):
        pose_array = PoseArray()

        # return array of shape (width, 3) with X, Y, Z coordinates 
        points = point_cloud2.pointcloud2_to_xyz_array(data)

        pose_array.header.stamp = rp.Time.now()
        pose_array.header.frame_id = 'fsds/Lidar'
        pose_array.poses = self.find_cones(points)

        self.cones_position_publisher.publish(pose_array)


    def points_group_to_cone(self, points_group: np.ndarray) -> Pose:
        pose = Pose()

        # TODO: Calculate average position of cone in X and Y axes
        points_group_average_x = 0.0
        points_group_average_y = 0.0

        pose.position.x = points_group_average_x
        pose.position.y = points_group_average_y
        pose.position.z = 0.0

        pose.orientation.x = 0.707
        pose.orientation.y = 0.0
        pose.orientation.z = 0.707
        pose.orientation.w = 0.0

        return pose


    def distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        # TODO: Implement Euclidean distance calculation

        return 1000.0


    def find_cones(self, points: np.ndarray) -> List[Pose]:
        cones = []

        # TODO: Implement points grouping (cones detection), remove cones (points) which exceed the cut off range, 
        # use points_group_to_cone function to get cone Pose for grouped points from Point Cloud scan

        return cones


if __name__ == '__main__':
    rp.init_node('lidar', log_level=rp.DEBUG)

    cones_range_cutoff = rp.get_param('cones_range_cutoff')

    LD = LidarDetector(cones_range_cutoff=cones_range_cutoff)

    rp.spin()
