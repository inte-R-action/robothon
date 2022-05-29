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

def cam2rob_transform(pose):
    print(pose)
    p_in_base = None
    print(tf_listener_.getFrameStrings())
    tf_listener_.waitForTransform("/base", "/camera_frame", rospy.Time(), rospy.Duration(4.0))
    print(tf_listener_.lookupTransform("/base", "/camera_frame", rospy.Time()))
    #if tf_listener_.frameExists('base') and tf_listener_.frameExists('camera_frame'):
    _ = tf_listener_.getLatestCommonTime("/base", "/camera_frame")
    p_in_base = tf_listener_.transformPose("/base", pose)

    # Moveit inverse()
    p_in_base.pose.position.x = -p_in_base.pose.position.x
    p_in_base.pose.position.y = -p_in_base.pose.position.y
    print(p_in_base)
    return p_in_base

def create_pose(x, y, angle, dist):
    p = PoseStamped()
    p.header.frame_id = "/camera_frame"
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

cam = rs_cam(depth=True)
cam.depth_frames(cam.pipeline.wait_for_frames())
cameraInfo = cam.depth_intrinsics

frame_id = 'Realsense_node'
rospy.init_node(frame_id, anonymous=True)
tf_listener_ = TransformListener()
obj_pub = obj_class(frame_id)

while 1:
    # Get frames from camera
    frames = cam.pipeline.wait_for_frames()
    color_image, depth_colormap, depth_image = cam.depth_frames(frames)
  # up right 489-175
  # down right 502-387
  # down left 284-399
  # up left 274-187
  # battery holder 441-380
  # up mid screw 370-152
  # up left screw 122-145
    x1 = 158
    y1 = 406
    x2 = 400
    y2 = 100

    depth1 = depth_image[y1, x1]
    result1 = rs.rs2_deproject_pixel_to_point(cameraInfo, [x1, y1], depth1)
    # depth2 = depth_image[y2, x2]
    # result2 = rs.rs2_deproject_pixel_to_point(cameraInfo, [x2, y2], depth2)

    if result1[2] != 0:
        obj_cam_pose = create_pose(result1[0], result1[1], 0, result1[2]) # Pose in camera frame
        obj_rob_pose = cam2rob_transform(obj_cam_pose)  # Pose in base frame (here or in cpp file)
        
        if obj_rob_pose is not None:
            obj_pub.publish(obj_rob_pose, 0, 'test')

    print(result1)
    # print("x: ", result1[0], result2[0], result2[0]-result1[0])
    # print("y: ", result1[1], result2[1], result2[1]-result1[1])
    cv2.circle(color_image, (x1, y1), 5, [255, 0, 0])
    # cv2.circle(color_image, (x2, y2), 5, [255, 0, 0])
    
    cv2.imshow("RS colour image", color_image)
    cv2.imshow("RS depth image", depth_image)
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break

cv2.destroyAllWindows()
cam.pipeline.stop()