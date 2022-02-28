#!/usr/bin/env python3

import math
from typing import List
import numpy as np
import rospy as rp
import cv2

from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from message_filters import ApproximateTimeSynchronizer, Subscriber



class VisionPoseEstimator:
    def __init__(self, model_path, cone_too_small_thresh, conf_thres, iou_thres, cone_clf_path):
        self.model_path = model_path
        self.cone_clf_path = cone_clf_path

        left_sub = Subscriber('/fsds/camera/cam_left', Image) # left camera
        depth_sub = Subscriber('/fsds/camera/cam_depth', Image) # depth camera
        bbox_sub = Subscriber('/fsds_roboticsII/cones_boxes', Detection2DArray) # left camera

        ats = ApproximateTimeSynchronizer([left_sub, depth_sub, bbox_sub], 1, slop=10)
        ats.registerCallback(self.subsciber_callback)

        self.inferenced_pub = rp.Publisher("/fsds_utils/camera/inferenced_image", Image, queue_size=1)
        self.red_cones_position_publisher = rp.Publisher('/fsds_utils/red_cones_position', PoseArray, queue_size=10)
        self.yellow_cones_position_publisher = rp.Publisher('/fsds_utils/yellow_cones_position', PoseArray, queue_size=10)
        self.blue_cones_position_publisher = rp.Publisher('/fsds_utils/blue_cones_position', PoseArray, queue_size=10)

        self.colors = [(0,0,255), (0,255,255), (255,0,0)] 

    def subsciber_callback(self, left_camera_data: Image, depth_camera_data: Image, bbox_data: Detection2DArray):
        img = np.frombuffer(left_camera_data.data, dtype=np.int8).reshape(left_camera_data.height, left_camera_data.width, -1)
        depth = np.frombuffer(depth_camera_data.data, dtype=np.float32).reshape(depth_camera_data.height, depth_camera_data.width, -1)
        bboxes = bbox_data.detections

        cone_poses = [self.estimate_cones_poses(box, depth) for box in bboxes]

        # TODO: split cone_poses into three parts: red_cones, yellow_cones, blue_cones
        # TODO: each value of cone_poses is a tuple: category and Pose
        # TODO: new lists should contain only Pose values


        # TODO: iterate over bboxes and draw rectangles on the img with respect to colours
        for box in bboxes:
            category = box.results[0].id
            bbox = box.bbox

            # TODO: draw rectangle here


        self.publish_img(img)
        self.publish_cones_pose_arrays(red_cones, yellow_cones, blue_cones)

    def estimate_cones_poses(self, box, depth):
        pose = Pose()

        # TODO: get the value of depth in the centre of the box
        d = depth[int(box.bbox.center.y), int(box.bbox.center.x)]

        ratio = (-1)*(box.bbox.center.x-608//2)

        pose.position.x = d
        pose.position.y = d*dy/box.bbox.center.y
        pose.position.z = 0.3

        pose.orientation.x = 0.707
        pose.orientation.y = 0.0
        pose.orientation.z = 0.707
        pose.orientation.w = 0.0

        color = box.results[0].id

        return [color, pose]

    def publish_img(self, img: np.ndarray):
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
        img_msg = Image()
        img_msg.header.stamp = rp.Time.now()
        img_msg.header.frame_id = "/fsds_utils/camera/inferenced_depth_image"
        img_msg.height = img.shape[0]
        img_msg.width = img.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = img.flatten().tobytes()
        img_msg.step = len(img_msg.data) // img_msg.height

        self.inferenced_pub.publish(img_msg)

    def publish_cones_pose_arrays(self, red_cones: PoseArray, yellow_cones: PoseArray, blue_cones: PoseArray):
        """Detected cones poses publishers with color division.
        Parameters
        ----------
        red_cones : PoseArray
            Array of red cones poses.
        yellow_cones : PoseArray
            Array of yellow cones poses.
        blue_cones : PoseArray
            Array of blue cones poses.
        """
        red_pose_array = PoseArray()
        yellow_pose_array = PoseArray()
        blue_pose_array = PoseArray()

        timestamp = rp.Time.now()

        for pose_array, poses in zip([red_pose_array, yellow_pose_array, blue_pose_array], [red_cones, yellow_cones, blue_cones]):
            pose_array.header.stamp = timestamp
            pose_array.header.frame_id = 'fsds/cam_left'
            pose_array.poses = poses

        self.red_cones_position_publisher.publish(red_pose_array)
        self.yellow_cones_position_publisher.publish(yellow_pose_array)
        self.blue_cones_position_publisher.publish(blue_pose_array)

if __name__ == '__main__':
    rp.init_node('vision-pose', log_level=rp.DEBUG)

    vision_conf = rp.get_param('vision')

    vd = VisionPoseEstimator(**vision_conf)

    rp.spin()
