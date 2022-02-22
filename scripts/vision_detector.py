#!/usr/bin/env python3

import math
from typing import List
import numpy as np
import rospy as rp
import cv2
import onnxruntime as ort

from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image

from utils.vision_utils import non_max_suppression

class YOLOOnnxDetector:
    def __init__(self, model_path, conf_thres, iou_thres) -> None:
        self.ort_sess = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])

        self.in_names = [inp.name for inp in self.ort_sess.get_inputs()]
        self.input_shape = self.ort_sess.get_inputs()[0].shape[2:4][::-1]
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres

    def preprocess(self, img: np.ndarray) -> np.ndarray:
        # TODO: Implement image preprocess function

        return img

    def detect(self, img):
        img_o = cv2.resize(img, self.input_shape)
        inp = self.preprocess(img_o)

        if len(inp.shape) != 4 or inp.dtype!=np.float32:
            return [], img_o

        outputs = self.ort_sess.run(None, {self.in_names[0]: inp})

        batch_detections = np.array(outputs[0])
        batch_detections = non_max_suppression(batch_detections, conf_thres=self.conf_thres, iou_thres=self.iou_thres, agnostic=False)

        return batch_detections, img_o


class VisionDetector:
    def __init__(self, model_path, cone_too_small_thresh, conf_thres, iou_thres):
        self.model_path = model_path

        self.detector = YOLOOnnxDetector(model_path=self.model_path, conf_thres=conf_thres, iou_thres=iou_thres)
        self.cone_too_small_thresh = cone_too_small_thresh
        self.image_size = 608

        # we work on left camera 
        rp.Subscriber("/fsds/camera/cam_left", Image, self.camera_detection_callback)

        self.cones_position_publisher = rp.Publisher('/fsds_roboticsII/cones_poses', PoseArray, queue_size=10)
        self.inferenced_img_pub = rp.Publisher("/fsds_utils/camera/inferenced_image", Image, queue_size=1)


    def camera_detection_callback(self, data: Image):
        pose_array = PoseArray()
        pose_array.header.stamp = rp.Time.now()
        pose_array.header.frame_id = 'fsds/cam_left'

        # return 2d array
        img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

        # inference
        boxes, im = self.detector.detect(img)

        if len(boxes)>0:
            # change (x1,y1,x2,y2) to (x,y,w,h)
            boxes[:,[2,3]] = boxes[:,[2,3]] - boxes[:,[0,1]]

            # throw out small detection
            boxes = boxes[boxes[:,2]*boxes[:,3]>self.cone_too_small_thresh]
            pose_array.poses = self.postprocess_cones(boxes)

        self.cones_position_publisher.publish(pose_array)
        
        # TODO: Implement boxes visualisation on the im array
                
                
        self.publish_inferenced_img(im)

    def postprocess_cones(self, bboxes: np.ndarray) -> List[Pose]:
        cones = []

        for x,y,w,h in bboxes:
            cones.append(self.points_to_cone([1e4/(h*abs(self.image_size/2-(x+w/2))), (self.image_size/2-(x+w/2))*0.02]))

        return cones

    def points_to_cone(self, points: np.ndarray) -> Pose:
        pose = Pose()

        pose.position.x = points[0]
        pose.position.y = points[1]
        pose.position.z = 0.0

        pose.orientation.x = 0.707
        pose.orientation.y = 0.0
        pose.orientation.z = 0.707
        pose.orientation.w = 0.0

        return pose

    def publish_inferenced_img(self, img: np.ndarray):
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
        img_msg.header.frame_id = "/fsds_utils/camera/inferenced_image"
        img_msg.height = img.shape[0]
        img_msg.width = img.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = img.flatten().tobytes()
        img_msg.step = len(img_msg.data) // img_msg.height

        self.inferenced_img_pub.publish(img_msg)



if __name__ == '__main__':
    rp.init_node('vision', log_level=rp.DEBUG)

    vision_conf = rp.get_param('vision')

    vd = VisionDetector(**vision_conf)

    rp.spin()