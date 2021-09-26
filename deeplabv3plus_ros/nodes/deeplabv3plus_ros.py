#!/usr/bin/env python
from __future__ import division
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64
from controller_msgs.msg import Angle2D
from cv_bridge import CvBridge, CvBridgeError
import os
from io import BytesIO
import tarfile
import tempfile
from six.moves import urllib
import rosparam
import math

# from PIL import Image

import tensorflow as tf
import time
from tensorflow.compat.v1 import InteractiveSession

#os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

#config = tf.ConfigProto()
#config = tf.ConfigProto()
#config.gpu_options.allow_growth = True
#session = InteractiveSession(config=config)

#gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=1)
#config.gpu_options.allow_growth = True
#sess = tf.Session(config=config)

rospy.init_node('semantic_segmentation', anonymous=True)

class SegmentImage(object):
    def __init__(self):
        self._image_pub = rospy.Publisher(
            '/seg_result', Image, queue_size=1)
        self._seg_map_pub = rospy.Publisher(
          '/seg_map', Image, queue_size=1)
        self._seg_map_pub = rospy.Publisher(
          '/seg_goal', Image, queue_size=1)
        self._tracking_point_pub = rospy.Publisher(
          '/tracking_msg', String, queue_size=10)
        self._image_sub = rospy.Subscriber(
            '/image', Image, self.image_callback, queue_size=1, buff_size=5000000)
        self._bridge = CvBridge()
        self._msg_sub = rospy.Subscriber(
            '/msg', String, self.msg_callback, queue_size=1, buff_size=5000000)
    def image_callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError, e:
            print(e)

        try:
            seg_img,seg_map = self.segmentation(cv_image)
            pub_img = self._bridge.cv2_to_imgmsg(seg_img)
            pub_map = self._bridge.cv2_to_imgmsg(seg_map)
            pub_img.header.frame_id = data.header.frame_id
            pub_img.header.stamp = data.header.stamp
            pub_map.header.frame_id = data.header.frame_id
            pub_map.header.stamp = data.header.stamp
            self._image_pub.publish(pub_img)
            self._seg_map_pub.publish(pub_map)
        except CvBridgeError, e:
            print(e)

    def segmentation(self, img):
        segmented_image, segmented_map = run_visualization(img)
        return segmented_image, segmented_map

    def msg_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self._pub_tracking_point.publish(tracking_point)

class DeepLabModel(object):
    """Class to load deeplab model and run inference."""

    INPUT_TENSOR_NAME = 'ImageTensor:0'
    OUTPUT_TENSOR_NAME = 'SemanticPredictions:0'
    INPUT_SIZE = 480
    FROZEN_GRAPH_NAME = 'frozen_inference_graph'

    def __init__(self, frozen_graph_filename):
        """Creates and loads pretrained deeplab model."""
        self.graph = tf.Graph()

        graph_def = None
        with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())

        if graph_def is None:
            raise RuntimeError('Cannot find inference graph.')

        with self.graph.as_default():
            tf.import_graph_def(graph_def, name='')

        self.sess = tf.Session(graph=self.graph)

    def run(self, image):
        """Runs inference on a single image.

        Args:
          image: A PIL.Image object, raw input image.

        Returns:
          resized_image: RGB image resized from original input image.
          seg_map: Segmentation map of `resized_image`.
        """
        resized_image = image
        batch_seg_map = self.sess.run(
            self.OUTPUT_TENSOR_NAME,
            feed_dict={self.INPUT_TENSOR_NAME: [np.asarray(resized_image)]})
        seg_map = batch_seg_map[0]
        return resized_image, seg_map


def create_pascal_label_colormap():
    """Creates a label colormap used in PASCAL VOC segmentation benchmark.

    Returns:
      A Colormap for visualizing segmentation results.
    """
    colormap = np.zeros((256, 3), dtype=int)
    ind = np.arange(256, dtype=int)

    for shift in reversed(range(8)):
        for channel in range(3):
            colormap[:, channel] |= ((ind >> channel) & 1) << shift
        ind >>= 3

    return colormap


def label_to_color_image(label):
    """Adds color defined by the dataset colormap to the label.

    Args:
      label: A 2D array with integer type, storing the segmentation label.

    Returns:
      result: A 2D array with floating type. The element of the array
        is the color indexed by the corresponding element in the input label
        to the PASCAL color map.

    Raises:
      ValueError: If label is not of rank 2 or its value is larger than color
        map maximum entry.
    """
    if label.ndim != 2:
        raise ValueError('Expect 2-D input label')

    colormap = create_pascal_label_colormap()

    if np.max(label) >= len(colormap):
        raise ValueError('label value too large.')
    return colormap[label]

#def object_tracking(tracking_data):

def vis_segmentation(image, seg_map):
    seg_image = label_to_color_image(seg_map).astype(np.uint8)
    result = cv2.add(image, seg_image)

    #pub_tracking_msg
    pub_tracking_point = rospy.Publisher('/tracking_msg/tracking_point',String, queue_size=10)
    pub_angle = rospy.Publisher('/tracking_msg/tracking_angle',String, queue_size=15)
    pub_radian = rospy.Publisher('/curiosity_mars_rover/mast_controller/rotate_cam', Angle2D, queue_size=100)

    red = (0, 0, 255)
    imgray = cv2.cvtColor(seg_image,cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(imgray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 200, 255)

    ret,thresh = cv2.threshold(imgray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    contours,hierarchy = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)

    cnt = contours[0]
    M = cv2.moments(cnt)

    if M['m00'] == 0:
        M['m00'] = M['m00'] + 0.01
    else:
        pass

    cen_x = int(M['m10']/M['m00'])
    cen_y = int(M['m01']/M['m00'])

    # camera angle solver
    k = [[400, 0.0, 400], [0.0, 400, 400], [0.0, 0.0, 1.0]]
    fx = k[0][0]
    fy = k[0][2]
    cx = k[1][1]
    cy = k[1][2]
    rx = (cen_x - cx) / fx
    ry = (cen_y - cy) / fy

    radian_x = math.atan(rx)
    radian_y = math.atan(ry)

    angle_x = math.atan(rx) / np.pi*180
    angle_y = math.atan(ry) / np.pi*180

    #tracking_angle = ("angle_x: " + str(angle_x), "angle_y: " + str(angle_y))
    tracking_angle = ("angle_x: %s angle_y: %s" %(str(angle_x),str(angle_y)))
    tracking_radian = ("(%s, %s)" %(str(radian_x),str(radian_y)))
    tracking_point = str("tracking_point: " + str((cen_x, cen_y)))

    tracking_msg = Angle2D()
    tracking_msg.x = float(radian_x)
    tracking_msg.y = -float(radian_y)
    #print(msg)

    pub_radian.publish(tracking_msg)
    pub_tracking_point.publish("%s" %tracking_point)
    pub_angle.publish("%s" %tracking_radian)

    cv2.circle(seg_image, (cen_x, cen_y), 5, red, -1)
    #cv2.rectangle(seg_image, (350, 450), (450, 350), (0, 255, 0), 1)
    cv2.putText(seg_image,"camera_angle:"+ tracking_angle, (20,60), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(seg_image,tracking_point, (20,40), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
    #cv2.resizeWindow("frame", 480, 480)
    #cv2.imshow("frame", seg_image)


    cv2.circle(result, (cen_x, cen_y), 5, red, -1)
    cv2.rectangle(result, (300, 450), (450, 300), (0, 255, 0), 2)
    cv2.putText(result,"camera_angle:"+ tracking_angle, (20,60), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(result,tracking_point, (20,40), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
    cv2.resizeWindow("image 2", 320, 320)
    cv2.imshow("image 2", result)

    cv2.waitKey(3)

    return result, seg_image
'''
# VOC
LABEL_NAMES = np.asarray([
    'background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus',
    'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike',
    'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tv'
])
'''
# Cityscapes
LABEL_NAMES = np.asarray([
    'road', 'sidewalk', 'building', 'wall', 'fence', 'pole',
    'traffic light', 'traffic sign', 'vegetation', 'terrain', 'sky',
    'person', 'rider', 'car', 'truck', 'bus', 'train', 'motorcycle',
    'bicycle'
])

FULL_LABEL_MAP = np.arange(len(LABEL_NAMES)).reshape(len(LABEL_NAMES), 1)
FULL_COLOR_MAP = label_to_color_image(FULL_LABEL_MAP)

model_path = rospy.get_param('~model_path')

MODEL = DeepLabModel(model_path)

def run_visualization(img):
    height, width = img.shape[:2]
    size = (width, height)
    #print(size)
    original_img = cv2.resize(img, (480, 320))
    #original_img = img
    resized_img, seg_map = MODEL.run(original_img)
    seg_map = np.array(seg_map, dtype='uint8')
    resized_img = cv2.resize(resized_img, size, interpolation=cv2.INTER_LINEAR)
    seg_map = cv2.resize(seg_map, size, interpolation=cv2.INTER_LINEAR) # resize
    seg_img, seg_map_img = vis_segmentation(resized_img, seg_map)
    return seg_img, seg_map_img

maps = [128, 64, 128]
def generateResult(results, nums=1, maps=maps):
    """ make gray picture to colorful"""
    h, w = results.shape
    painters = maps
    goal = np.zeros((h, w, 3))
    for _ in range(nums):
        goal[results==_] = np.array(painters[_])
    return seg_goal


if __name__ == '__main__':
    si = SegmentImage()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
