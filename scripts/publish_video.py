#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Publish a video as ROS messages.
"""

import argparse

import numpy as np

import cv2

import rospy

from sensor_msgs.msg import Image


from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def main():
    """Publish a video as ROS messages.
    """
    # Patse arguments.
    parser = argparse.ArgumentParser(description="Convert video into a rosbag.")
    parser.add_argument("video_file", help="Input video.")
    parser.add_argument("-c", "--camera", default="camera", help="Camera name.")
    parser.add_argument("-f", "--frame_id", default="camera",
                        help="tf frame_id.")
    parser.add_argument("--width", type=np.int32, default="640",
                        help="Image width.")
    parser.add_argument("--height", type=np.int32, default="480",
                        help="Image height.")
    parser.add_argument("--fps", type=np.int32, default="30",
                        help="Image fps")
    parser.add_argument("--info_url", default="file:///camera.yml",
                        help="Camera calibration url.")

    args = parser.parse_args()

    print("Publishing %s." % (args.video_file))

    # Set up node.
    rospy.init_node("video_publisher", anonymous=True)
    img_pub = rospy.Publisher("/fsds/camera/cam_left", Image,
                              queue_size=10)

    # Open video.
    video = cv2.VideoCapture(args.video_file)

    # Get frame rate.
    fps = args.fps
    rate = rospy.Rate(fps)

    # Loop through video frames.
    while not rospy.is_shutdown() and video.grab():
        tmp, img = video.retrieve()

        if not tmp:
            print ("Could not grab frame.")
            break

        # Compute input/output aspect ratios.

        s = args.height/img.shape[0]

        img_out = cv2.resize(img, None, fx=s, fy=s)
        img_out = img_out[:args.height, img_out.shape[1]//2-args.width//2:img_out.shape[1]//2+args.width//2]

        try:
            # Publish image.
            img_msg = bridge.cv2_to_imgmsg(img_out, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = args.frame_id
            img_pub.publish(img_msg)

        except CvBridgeError as err:
            print (err)

        rate.sleep()

    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
