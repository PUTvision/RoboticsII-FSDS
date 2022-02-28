#!/usr/bin/env python3

import math
from typing import List
import numpy as np
import rospy as rp
import cv2

from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image



class VisionDetector:
    def __init__(self, model_path, cone_too_small_thresh, conf_thres, iou_thres, cone_clf_path):
        self.model_path = model_path
        self.cone_clf_path = cone_clf_path

        # we work on left camera 
        rp.Subscriber("/fsds/camera/cam_depth", Image, self.camera_depth_callback)

        self.inferenced_depth_pub = rp.Publisher("/fsds_utils/camera/inferenced_depth_image", Image, queue_size=1)


    def camera_depth_callback(self, data: Image):
        depth = np.frombuffer(data.data, dtype=np.float32).reshape(data.height, data.width, -1).copy()

        # TODO: process depth image to image


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
