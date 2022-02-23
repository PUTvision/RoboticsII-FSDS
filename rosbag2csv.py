#!/usr/bin/python3
import csv

import rosbag
import argparse

topis = ['/fsds/testing_only/odom']

class Point:
    timestamp = None
    x = None
    y = None
    z = None

    def __init__(self, timestamp, x, y, z):
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return '(' + str(self.timestamp/8) + ', ' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.z) + ')'

class Helper:
    home_point = None

    def is_init(self):
        return self.home_point is not None

    def init(self, timestamp, x, y, z):
        self.home_point = Point(timestamp, x*0.5, y*0.5, z*0.5)

    def create_new_point(self, timestamp, x, y, z):
        assert self.is_init()

        return Point(timestamp, x*0.5-self.home_point.x, y*0.5-self.home_point.y, z*0.5-self.home_point.z)


def main(rosbag_file, output_file):
    bag = rosbag.Bag(rosbag_file)
    helper = Helper()

    with open(output_file, 'w') as csvfile:
        ##### TODO: create csv writer

        for topic, msg, t in bag.read_messages(topics=topis):
            position = msg.pose.pose.position
            timestamp = float(str(msg.header.stamp))/10**9

            ##### TODO: if helper not init, do it

            ##### TODO: create new point from 
            
            ##### TODO: write row to csv file using TUM Ground-truth trajectories format 

    bag.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Say hello')
    parser.add_argument('rosbag_file', help='Path to rosbag')
    parser.add_argument('output_file', help='Path to result')
    args = parser.parse_args()

    main(args.rosbag_file, args.output_file)

