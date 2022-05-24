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
import pandas as pd
import time
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


class object_transformer:
    def __init__(self):
        frame_id = 'Realsense_node'
        rospy.init_node(frame_id, anonymous=True)
        self.tf_listener_ = TransformListener()

        self.obj_pub = obj_class(frame_id)
        self.db = database()
        self.predictions = []
        self.obj_tab_col_names, _ = self.db.query_table('detected_objects', rows=0)

    def cam2rob_transform(self, pose, frame):
        print(pose)
        p_in_base = None
        print(self.tf_listener_.getFrameStrings())
        self.tf_listener_.waitForTransform("/base", frame, rospy.Time(), rospy.Duration(4.0))
        print(self.tf_listener_.lookupTransform("/base", frame, rospy.Time()))
        #if self.tf_listener_.frameExists('/base') and self.tf_listener_.frameExists(frame):
        _ = self.tf_listener_.getLatestCommonTime("/base", frame)
        p_in_base = self.tf_listener_.transformPose("/base", pose)
        
        # Moveit inverse()
        p_in_base.pose.position.x = -p_in_base.pose.position.x
        p_in_base.pose.position.y = -p_in_base.pose.position.y

        print(p_in_base)
        return p_in_base

    def create_pose(self, x, y, z, angle):
        p = PoseStamped()
        p.header.frame_id = "/camera_frame"

        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        # Make sure the quaternion is valid and normalized
        #angle = radians(angle+45)
        p.pose.orientation.x = sin(angle/2) * 1
        p.pose.orientation.y = sin(angle/2) * 0
        p.pose.orientation.z = sin(angle/2) * 0
        p.pose.orientation.w = cos(angle/2)

        return p

    def run_pipeline(self):
        self.read_database()

        self.detections = self.detections.reset_index()  # make sure indexes pair with number of rows
        print(f"Detections: {self.detections['obj_name']}")
        for index, row in self.detections.iterrows():
            print(row['center_x'], row['center_y'], row['distance'], row['rotation'], row['obj_name'])
            if (row['center_x'] != 0) and (row['center_y'] != 0):
                obj_cam_pose = self.create_pose(row['center_x'], row['center_y'], row['distance'], row['rotation']) # Pose in camera frame
                if row['obj_name'] == "new_view":
                    frame = "/wrist_3_link"
                else:
                    frame = "/camera_frame"
                obj_rob_pose = self.cam2rob_transform(obj_cam_pose, frame)  # Pose in base frame (here or in cpp file)
                
                if obj_rob_pose is not None:
                    self.obj_pub.publish(obj_rob_pose, row['obj_id'], row['obj_name'])
    
    def read_database(self):
        col_names, data = self.db.query_table('detected_objects', 'all')
        self.detections = pd.DataFrame(data, columns=col_names)


if __name__ == '__main__':               
    try:
        obj_transformer = object_transformer()

        rate = rospy.Rate(0.5) # 0.5 Hz
        while (not rospy.is_shutdown()):
            try:
                obj_transformer.run_pipeline()
            except Exception as e:
                print(e)
            rate.sleep()
    
    except rospy.ROSInterruptException:
        print("object_ros_publisher ROS exception")
    
    except Exception as e:
        print("**object_ros_publisher Error**")
        traceback.print_exc(file=sys.stdout)
