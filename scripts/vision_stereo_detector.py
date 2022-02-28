#!/usr/bin/env python3

import math
from typing import List
import numpy as np
import rospy as rp
import cv2

from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber


class VisionDetector:
    def __init__(self, model_path, cone_too_small_thresh, conf_thres, iou_thres, cone_clf_path):
        self.model_path = model_path
        self.cone_clf_path = cone_clf_path

        # two subscriber
        left_sub = Subscriber('/fsds/camera/cam_left', Image) # left camera
        right_sub = Subscriber('/fsds/camera/cam_right', Image) # right camera

        ats = ApproximateTimeSynchronizer([left_sub, right_sub], 1, slop=0.01)
        ats.registerCallback(self.stereo_callback)

        self.inferenced_depth_pub = rp.Publisher("/fsds_utils/camera/inferenced_depth_image", Image, queue_size=1)

        # TODO: STEREO MATCHING
        


    def stereo_callback(self, left_data: Image, right_data: Image):
        """Callback for topics data from stereo camera.
        Parameters
        ----------
        left_data : Image
            Image topic data from left camera.
        right_data : Image
            Image topic data from right camera.
        """
        left_img = np.frombuffer(left_data.data, dtype=np.uint8).reshape(left_data.height, left_data.width, -1)
        right_img = np.frombuffer(right_data.data, dtype=np.uint8).reshape(right_data.height, right_data.width, -1)

        self.depth_estimation(left_img, right_img)

    def depth_estimation(self, left_img: np.ndarray, right_img: np.ndarray):
        
        #TODO: DEPTH IMAGE
        img = left_img # REMOVE THIS

        self.publish_depth_img(img)

    def publish_depth_img(self, img: np.ndarray):
        """Publish image from camera with added detection and classification boxes. 
        Parameters
        ----------
        img : np.ndarray
            Inferenced image from camera.
        boxes : np.ndarray
            Detection boxes.
        cone_colors : np.ndarray
            Color classification results for detection boxes.
        """
        img = cv2.applyColorMap(img.astype(np.uint8), cv2.COLORMAP_JET) if img.dtype != np.uint8 else img
        img_msg = Image()
        img_msg.header.stamp = rp.Time.now()
        img_msg.header.frame_id = "/fsds_utils/camera/inferenced_depth_image"
        img_msg.height = img.shape[0]
        img_msg.width = img.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = img.flatten().tobytes()
        img_msg.step = len(img_msg.data) // img_msg.height

        self.inferenced_depth_pub.publish(img_msg)



if __name__ == '__main__':
    rp.init_node('depth-vision', log_level=rp.DEBUG)

    vision_conf = rp.get_param('vision')

    vd = VisionDetector(**vision_conf)

    rp.spin()
