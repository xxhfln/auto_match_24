#!/usr/bin/env python
## Author: Rohit
## Date: July, 25, 2017
# Purpose: Ros node to detect objects using tensorflow

import os
import sys
import cv2
import numpy as np

try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
import rospy
import rospkg
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Object detection module imports
# import object_detection
# from object_detection.utils import label_map_util
# from object_detection.utils import visualization_utils as vis_util
from ultralytics import YOLO

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.4

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# list all packages, equivalent to rospack list
rospack.list()
# get the file path for tensorflow_object_detector
PACKAGE_PATH = os.path.join(rospack.get_path('auto_match'))

######### Set model here ############
MODEL_NAME = 'best.pt'
# # By default models are stored in data/models/
MODEL_PATH = os.path.join(PACKAGE_PATH, 'model', MODEL_NAME)
# # Path to frozen detection graph. This is the actual model that is used for the object detection.
# PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb'
# ######### Set the label map file here ###########
# LABEL_NAME = 'mscoco_label_map.pbtxt'
# # By default label maps are stored in data/labels/
# PATH_TO_LABELS = os.path.join(PACKAGE_PATH, 'data', 'labels', LABEL_NAME)
# ######### Set the number of classes here #########
# NUM_CLASSES = 90

# detection_graph = tf.compat.v1.Graph()
# with detection_graph.as_default():
#     od_graph_def = tf.compat.v1.GraphDef()
#     with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
#         serialized_graph = fid.read()
#         od_graph_def.ParseFromString(serialized_graph)
#         tf.compat.v1.import_graph_def(od_graph_def, name='')

# ## Loading label map
# # Label maps map indices to category names, so that when our convolution network predicts `5`,
# # we know that this corresponds to `airplane`.  Here we use internal utility functions,
# # but anything that returns a dictionary mapping integers to appropriate string labels would be fine
# label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
# categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
#                                                             use_display_name=True)
# category_index = label_map_util.create_category_index(categories)

# # Setting the GPU options to use fraction of gpu that has been set
# config = tf.compat.v1.ConfigProto()
# config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION


# Detection

class Detector:

    def __init__(self):
        self.image_pub = rospy.Publisher("debug_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image", Image, self.image_cb, queue_size=1, buff_size=2 ** 24)
        # self.sess = tf.compat.v1.Session(graph=detection_graph, config=config)
        self.model = YOLO(MODEL_PATH)

    def image_cb(self, data):
        # print("11111\n")
        objArray = Detection2DArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.model.predict(cv_image,conf=0.6)
        image_plot = results[0].plot()


        objArray.detections = []
        objArray.header = data.header
        object_count = 1

        for i in range(len(results)):
            object_count += 1
            objArray.detections.append(self.object_predict(results[i], data.header, cv_image))

        self.object_pub.publish(objArray)

        # img = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        image_out = Image()
        try:
            image_out = self.bridge.cv2_to_imgmsg(image_plot, "bgr8")
        except CvBridgeError as e:
            print(e)
        image_out.header = data.header
        self.image_pub.publish(image_out)

    def object_predict(self, result, header, image):
        image_height, image_width, channels = image.shape
        obj = Detection2D()
        obj_hypothesis = ObjectHypothesisWithPose()
        
        box = result.boxes
        xywh = box.xywh.numpy().astype(int)
        cls = box.cls.numpy().astype(int)
        score = box.conf.numpy().astype(float)

        object_id = cls[0]
        object_score = score[0]
        # dimensions = object_data[2]

        obj.header = header
        obj_hypothesis.id = object_id
        obj_hypothesis.score = object_score
        obj.results.append(obj_hypothesis)
        label = xywh[0]
        obj.bbox.size_y = int(label[3])
        obj.bbox.size_x = int(label[2])
        obj.bbox.center.x = int(label[0])
        obj.bbox.center.y = int(label[1])

        return obj


def main(args):
    rospy.init_node('detector_node')
    obj = Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
