#!/usr/bin/env python3.7

import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from math import sin, cos, radians
from object_localisation.msg import object_state
import traceback, sys
import os
from database_funcs import database
import pyrealsense2 as rs
import pickle
import pandas as pd
ROOT_DIR = os.path.dirname(__file__)

class obj_class:
    def __init__(self, frame_id, queue=1):
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



class object_localiser:
    def __init__(self):
        frame_id = 'Realsense_node'
        rospy.init_node(frame_id, anonymous=True)
        self.tf_listener_ = TransformListener()

        self.obj_pub = obj_class(frame_id)
        self.db = database()
        self.predictions = []
        self.obj_tab_col_names, _ = self.db.query_table('detected_objects', rows=0)
        self.cameraInfo = rs.pyrealsense2.intrinsics()
        self.cameraInfo = pickle.load( open( ROOT_DIR+"/cameraInfo.p", "rb" ) )
        print(self.cameraInfo)

    def cam2rob_transform(self, pose):
        p_in_base = None
        print(self.tf_listener_.getFrameStrings())
        if self.tf_listener_.frameExists('base_link') and self.tf_listener_.frameExists('camera_frame'):
            t = self.tf_listener_.getLatestCommonTime("/base_link", "/camera_frame")
            # p1 = PoseStamped()
            # p1.header.frame_id = "fingertip"
            # p1.pose.orientation.w = 1.0    # Neutral orientation
            p_in_base = self.tf_listener_.transformPose("/base_link", pose)

        return p_in_base

    def create_pose(self, x, y, angle, dist):

        _intrinsics = rs.intrinsics()
        _intrinsics.width = self.cameraInfo["width"]
        _intrinsics.height = self.cameraInfo["height"]
        _intrinsics.ppx = self.cameraInfo["ppx"]
        _intrinsics.ppy = self.cameraInfo["ppy"]
        _intrinsics.fx = self.cameraInfo["fx"]
        _intrinsics.fy = self.cameraInfo["fy"]
        _intrinsics.coeffs = self.cameraInfo["coeffs"]
        ###_intrinsics.model = cameraInfo.distortion_model
        #_intrinsics.model  = rs.distortion.none
        #_intrinsics.coeffs = [i for i in cameraInfo.D]
        result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x, y], dist)

        p = PoseStamped()
        p.header.frame_id = "/camera_frame"
        
        p.pose.position.x = result[2]
        p.pose.position.y = -result[0]
        p.pose.position.z = -result[1]
        # Make sure the quaternion is valid and normalized
        angle = radians(angle+45)
        p.pose.orientation.x = sin(angle/2) * 1
        p.pose.orientation.y = sin(angle/2) * 0
        p.pose.orientation.z = sin(angle/2) * 0
        p.pose.orientation.w = cos(angle/2)

        return p

    def run_localisation(self):
        self.read_database()

        self.detections = self.detections.reset_index()  # make sure indexes pair with number of rows
        print(f"Detections: {self.detections['obj_name']}")
        for index, row in self.detections.iterrows():
            x = row['center_x']
            y = row['center_y']
            angle = row['rotation']
            dist = row['distance']
            print(x, y, row['obj_name'])
            obj_cam_pose = self.create_pose(x, y, angle, dist)
            obj_rob_pose = self.cam2rob_transform(obj_cam_pose)
            
            if obj_rob_pose is not None:
                obj_id = row['obj_id']
                obj_name = row['obj_name']
                self.obj_pub.publish(obj_rob_pose, obj_id, obj_name)
    
    def read_database(self):
        col_names, data = self.db.query_table('detected_objects', 'all')
        self.detections = pd.DataFrame(data, columns=col_names)


if __name__ == '__main__':               
    try:
        localiser = object_localiser()

        rate = rospy.Rate(1)
        while (not rospy.is_shutdown()):
            try:
                localiser.run_localisation()
            except Exception as e:
                print(e)
            rate.sleep()
    
    except rospy.ROSInterruptException:
        print("realsense_run ROS exception")
    
    except Exception as e:
        print("**Image Error**")
        traceback.print_exc(file=sys.stdout)
