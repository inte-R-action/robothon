#!/usr/bin/env python3.7

from math import sin, cos, radians
from rs_cam import rs_cam
import cv2
import pyrealsense2 as rs
import argparse
import traceback, sys
# from object_classifier import Mask_Rcnn_object_detection
import os
from database_funcs import database
import numpy as np
from object_localisation.msg import object_state


import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
ROOT_DIR = os.path.dirname(__file__)

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
class ImageListener:
    def __init__(self):
        self.bridge = CvBridge()
        self.depthsub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.imageDepthCallback)
        self.colorsub = rospy.Subscriber('/camera/color/image_raw', Image, self.imageColorCallback)
        self.infosub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.infoCallback)
        self.depth_image = None
        self.colour_image = None
        self.cameraInfo = None

    def imageDepthCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except Exception as e:
            print("depth error: ", e)
            return
    
    def imageColorCallback(self, data):
        try:
            self.colour_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except Exception as e:
            print("color error: ", e)
            return
    def infoCallback(self, data):
        try:
            self.cameraInfo = rs.intrinsics()
            self.cameraInfo.width = data.height
            self.cameraInfo.height = data.width
            self.cameraInfo.ppx = data.K[2]
            self.cameraInfo.ppy = data.K[5]
            self.cameraInfo.fx = data.K[0]
            self.cameraInfo.fy = data.K[4]
            self.cameraInfo.model = rs.distortion.brown_conrady
            self.cameraInfo.coeffs = data.D
        except Exception as e:
            print("info error: ", e)
            return

class obj_class:
    def __init__(self, frame_id, queue=10):
        # frame_id=str, queue=int
        # Object message definitions
        self.obj_msg = object_state()
        self.obj_msg.Header.stamp = rospy.get_rostime()
        self.obj_msg.Header.seq = None
        self.obj_msg.Header.frame_id = frame_id
        self.obj_msg.Id = None
        self.obj_msg.Obj_type = None
        self.obj_msg.Pose.orientation.x = None
        self.obj_msg.Pose.orientation.y = None
        self.obj_msg.Pose.orientation.z = None
        self.obj_msg.Pose.orientation.w = None
        self.obj_msg.Pose.position.x = None
        self.obj_msg.Pose.position.y = None
        self.obj_msg.Pose.position.z = None

        self.publisher = rospy.Publisher('ObjectStates', object_state, queue_size=queue)

    def publish(self, pose, id, name):
        if self.obj_msg.Header.seq is None:
            self.obj_msg.Header.seq = 0
        else:
            self.obj_msg.Header.seq += 1
        
        self.obj_msg.Id = id
        self.obj_msg.Obj_type = name
        self.obj_msg.Pose = pose.pose
        self.obj_msg.Header.stamp = rospy.get_rostime()

        self.publisher.publish(self.obj_msg)

# def cam2rob_transform(pose):
#     print(pose)
#     p_in_base = None
#     print(tf_listener_.getFrameStrings())
#     tf_listener_.waitForTransform("/base", "/camera_frame", rospy.Time(), rospy.Duration(4.0))
#     print(tf_listener_.lookupTransform("/base", "/camera_frame", rospy.Time()))
#     #if tf_listener_.frameExists('base') and tf_listener_.frameExists('camera_frame'):
#     _ = tf_listener_.getLatestCommonTime("/base", "/camera_frame")
#     p_in_base = tf_listener_.transformPose("/base", pose)

#     # Moveit inverse()
#     p_in_base.pose.position.x = -p_in_base.pose.position.x
#     p_in_base.pose.position.y = -p_in_base.pose.position.y
#     print(p_in_base)
#     return p_in_base
def cam2rob_transform(pose):
    print(pose)
    p_in_base = None
    # print(tf_listener_.getFrameStrings())
    tf_listener_.waitForTransform("/base", "/camera_color_optical_frame", rospy.Time(), rospy.Duration(4.0))
    # print(tf_listener_.lookupTransform("/base", "/camera_color_optical_frame", rospy.Time()))
    #if tf_listener_.frameExists('base') and tf_listener_.frameExists('camera_frame'):
    _ = tf_listener_.getLatestCommonTime("/base", "/camera_color_optical_frame")
    p_in_base = tf_listener_.transformPose("/base", pose)

    # Moveit inverse()
    p_in_base.pose.position.x = -p_in_base.pose.position.x
    p_in_base.pose.position.y = -p_in_base.pose.position.y
    print(p_in_base)
    return p_in_base

def create_pose(x, y, angle, dist):
    p = PoseStamped()
    p.header.frame_id = "/camera_color_optical_frame"
    p.header.stamp = rospy.Time()

    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = dist
    # Make sure the quaternion is valid and normalized
    angle = radians(angle)
    p.pose.orientation.x = sin(angle/2) * 0
    p.pose.orientation.y = sin(angle/2) * 0
    p.pose.orientation.z = sin(angle/2) * 1
    p.pose.orientation.w = cos(angle/2)

    return p

# cam = rs_cam(depth=True)
# cam.depth_frames(cam.pipeline.wait_for_frames())
# cameraInfo = cam.depth_intrinsics

# cameraInfo = rs.intrinsics()
# cameraInfo.width = 640
# cameraInfo.height = 480
# cameraInfo.ppx = 324.7422
# cameraInfo.ppy = 225.8990
# cameraInfo.fx = 630.9241
# cameraInfo.fy = 629.5015
# # self.cameraInfo.coeffs = [Radial: [0.140784267654903,-0.223962950366363] Tangential: [0,0]]
# # self.cameraInfo.model = cameraInfo.distortion_model
# cameraInfo.model = rs.distortion.none
# self.cameraInfo.coeffs = [i for i in cameraInfo.D]
# print(f"custom intrinsics: {cameraInfo}")

frame_id = 'Realsense_node'
rospy.init_node(frame_id, anonymous=True)
tf_listener_ = TransformListener()
obj_pub = obj_class(frame_id)

listener = ImageListener()
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    # Get frames from camera
    # frames = cam.pipeline.wait_for_frames()
    #color_image, depth_colormap, depth_image, aligned_depth_frame = cam.depth_frames(frames)
    color_image = listener.colour_image
    depth_image = listener.depth_image
  # up right 489-175
  # down right 502-387
  # down left 284-399
  # up left 274-187
  # battery holder 441-380
  # up mid screw 370-152
  # up left screw 122-145
    x1 = 509
    y1 = 311
    x2 = 400
    y2 = 100

    try:
        depth1 = depth_image[y1, x1]
        # depth2 = aligned_depth_frame.get_distance(int(x1),int(y1))
        # print(depth1, depth2)
        result1 = rs.rs2_deproject_pixel_to_point(listener.cameraInfo, [x1, y1], depth1)
        # depth2 = depth_image[y2, x2]
        # result2 = rs.rs2_deproject_pixel_to_point(cameraInfo, [x2, y2], depth2)

        if result1[2] != 0:
            obj_cam_pose = create_pose(result1[0]/1000, result1[1]/1000, 0, result1[2]/1000) # Pose in camera frame
            obj_rob_pose = cam2rob_transform(obj_cam_pose)  # Pose in base frame (here or in cpp file)
            
            if obj_rob_pose is not None:
                obj_pub.publish(obj_rob_pose, 0, 'test')

        print(result1)
        # print("x: ", result1[0], result2[0], result2[0]-result1[0])
        # print("y: ", result1[1], result2[1], result2[1]-result1[1])
        cv2.circle(color_image, (x1, y1), 5, [255, 0, 0])
        # cv2.circle(color_image, (x2, y2), 5, [255, 0, 0])
        
        cv2.imshow("RS colour image", listener.colour_image)
        cv2.imshow("RS depth image", depth_image)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
    except Exception as e:
        print("exception: ", e)
    rate.sleep()

cv2.destroyAllWindows()
# cam.pipeline.stop()