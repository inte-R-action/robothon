#!/usr/bin/env python3.7

#* Author: inte-R-action Team
#* Date: 29-May-2022
# *
#* University of Bath
#* Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
#* Centre for Autonomous Robotics (CENTAUR)
#* Department of Electronics and Electrical Engineering
#*
#* Description:  publisher class to pass objects infromation from database to ROS

from matplotlib.pyplot import box
import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from math import sin, cos, radians, degrees
from object_localisation.msg import object_state
import traceback, sys
import os
from database_funcs import database
import pyrealsense2 as rs
import pandas as pd
import time
from scipy.spatial.transform import Rotation
ROOT_DIR = os.path.dirname(__file__)
import numpy as np

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
        self.updateSub = rospy.Subscriber("UpdateLocations", Bool, self.updateCallback)
        self.updatePub = rospy.Publisher("LocalisationActive", Bool, queue_size=1)
        self.tf_listener_ = TransformListener()

        self.obj_pub = obj_class(frame_id)
        self.updateFlag = True
        self.db = database()
        self.obj_tab_col_names, _ = self.db.query_table('detected_objects', rows=0)

        self.detections = pd.DataFrame()
        self.all_detections = pd.DataFrame()
        self.average_detections = pd.DataFrame()
        self.poses2pub = []

        self.frames2ave = 4
        self.frames_observed = 0

    def updateCallback(self, msg):
        self.updateFlag = msg.data
        print(f"Flag set {self.updateFlag}")

    def cam2rob_transform(self, pose, frame):
        # print(pose)
        p_in_base = None
        #print(self.tf_listener_.getFrameStrings())
        self.tf_listener_.waitForTransform("/base", frame, rospy.Time(), rospy.Duration(4.0))
        #print(self.tf_listener_.lookupTransform("/base", frame, rospy.Time()))
        #if self.tf_listener_.frameExists('/base') and self.tf_listener_.frameExists(frame):
        _ = self.tf_listener_.getLatestCommonTime("/base", frame)
        p_in_base = self.tf_listener_.transformPose("/base", pose)
        
        # Moveit inverse()
        p_in_base.pose.position.x = -p_in_base.pose.position.x
        p_in_base.pose.position.y = -p_in_base.pose.position.y
        box_angle = Rotation.from_quat([p_in_base.pose.orientation.x, p_in_base.pose.orientation.y, p_in_base.pose.orientation.z, p_in_base.pose.orientation.w]).as_euler('xyz', degrees=True)
        box_angle = -box_angle[2]+90

        if box_angle < -180:
            box_angle = -(360 + box_angle)
        elif box_angle > 180:
            box_angle = -(360 - box_angle)
        p_in_base.pose.orientation.z = -box_angle

        # print(p_in_base)
        return p_in_base

    def create_pose(self, x, y, z, angle):
        p = PoseStamped()
        p.header.frame_id = "/camera_color_optical_frame"

        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        # Make sure the quaternion is valid and normalized
        angle = radians(angle)
        p.pose.orientation.x = sin(angle/2) * 0
        p.pose.orientation.y = sin(angle/2) * 0
        p.pose.orientation.z = sin(angle/2) * 1
        p.pose.orientation.w = cos(angle/2)

        return p

    def run_pipeline_old(self):
        # Update predictions
        self.updatePub.publish(self.updateFlag) # Let GUI know if we're updating
        if self.detections.empty:
            self.read_database()
        else:
            if self.updateFlag or (self.detections.iloc[0]['obj_name'] == 'new_view'):
                self.read_database()

                # Transform and add to poses to publish
                self.poses2pub = []
                for index, row in self.detections.iterrows():
                    if (row['center_x'] != 0) and (row['center_y'] != 0):
                        # Pose in camera frame
                        obj_cam_pose = self.create_pose(row['center_x'], row['center_y'], row['distance'], row['rotation'])
                        if row['obj_name'] == "new_view":
                            frame = "/wrist_3_link" # want to position under camera
                        else:
                            frame = "/camera_color_optical_frame" # want to position under gripper
                        # Pose in base frame (here or in cpp file)
                        obj_rob_pose = self.cam2rob_transform(obj_cam_pose, frame)

                        self.poses2pub.append({'name': row['obj_name'], 'id': row['obj_id'], 'pose': obj_rob_pose})
                
                if self.detections.iloc[0]['obj_name'] != 'new_view':
                    self.updateFlag = False
                    print("Flag set False")

            for object in self.poses2pub:
                if object['pose'] is not None:
                    if object['name'] == 'battery':
                        print(object['name'])
                        print(object['pose'])
                    self.obj_pub.publish(object['pose'], object['id'], object['name'])
                   
        print("Detections Camera Frame:")
        print(self.detections)
    
    def run_pipeline_new(self):
        # Update predictions
        self.updatePub.publish(self.updateFlag) # Let GUI know if we're updating
        
        if self.updateFlag or (self.average_detections.iloc[0]['obj_name'] == 'new_view'):
            self.read_database()
            # Transform and add to poses to publish
            self.poses2pub = []
            for index, row in self.average_detections.iterrows():
                if (row['center_x'] != 0) and (row['center_y'] != 0):
                    # Pose in camera frame
                    obj_cam_pose = self.create_pose(row['center_x'], row['center_y'], row['distance'], row['rotation'])
                    if row['obj_name'] == "new_view":
                        frame = "/wrist_3_link" # want to position under camera
                    else:
                        frame = "/camera_color_optical_frame" # want to position under gripper
                    # Pose in base frame (here or in cpp file)
                    obj_rob_pose = self.cam2rob_transform(obj_cam_pose, frame)

                    self.poses2pub.append({'name': row['obj_name'], 'id': row['obj_id'], 'pose': obj_rob_pose})
            
            if not self.average_detections.empty:
                if (self.detections.iloc[0]['obj_name'] != 'new_view') and (self.frames_observed >= self.frames2ave):
                    self.updateFlag = False
                    self.detections = pd.DataFrame()
                    self.all_detections = pd.DataFrame()
                    self.frames_observed = 0
                    print("Flag set False")

        for object in self.poses2pub:
            if object['pose'] is not None:
                if object['name'] == 'battery':
                    print(object['name'])
                    print(object['pose'])
                self.obj_pub.publish(object['pose'], object['id'], object['name'])
                   
        print("All Detections Camera Frame:")
        print(self.all_detections)
        print("Average Detections Camera Frame:")
        print(self.average_detections)
    
    def read_database(self):
        col_names, data = self.db.query_table('detected_objects', 'all')
        self.detections = pd.DataFrame(data, columns=col_names)
        self.average_over_detections(col_names)
    
    def average_over_detections(self, col_names):
        if self.all_detections.empty:
            self.all_detections = self.detections
            self.average_detections = pd.DataFrame(columns=col_names[1:])
        else:
            new_observation = False
            for index, row in self.detections.iterrows():
                # make sure we only add detections not seen yet
                if not self.all_detections['identification'].isin([row['identification']]).any():
                    self.all_detections.loc[len(self.all_detections)] = row
                    new_observation = True
            if new_observation:
                self.frames_observed = self.frames_observed + 1
                print(f"Number of observations: {self.frames_observed}/{self.frames2ave}")
        
        self.average_detections = pd.DataFrame(columns=col_names[1:])
        for obj_name in pd.unique(self.all_detections.obj_name):
            obj_id = self.all_detections.loc[self.all_detections['obj_name'] == obj_name]['obj_id'].values[0]
            xs = self.all_detections.loc[self.all_detections['obj_name'] == obj_name]['center_x'].values
            ys = self.all_detections.loc[self.all_detections['obj_name'] == obj_name]['center_y'].values
            dists = self.all_detections.loc[self.all_detections['obj_name'] == obj_name]['distance'].values
            rots = self.all_detections.loc[self.all_detections['obj_name'] == obj_name]['rotation'].values
            confs = self.all_detections.loc[self.all_detections['obj_name'] == obj_name]['confidence'].values

            med_x = np.median(xs)
            med_y = np.median(ys)
            med_dist = np.median(dists)
            med_rot = np.median(rots)
            med_conf = np.median(confs)

            new_row = [obj_id, obj_name, med_conf, med_rot, med_x, med_y, med_dist]
            self.average_detections.loc[len(self.average_detections)] = new_row


if __name__ == '__main__':               
    try:
        obj_transformer = object_transformer()

        rate = rospy.Rate(0.5) # 0.5 Hz
        while (not rospy.is_shutdown()):
            try:
                obj_transformer.run_pipeline_new()
            except Exception as e:
                print("error: ", e)
            rate.sleep()
    
    except rospy.ROSInterruptException:
        print("object_ros_publisher ROS exception")
    
    except Exception as e:
        print("**object_ros_publisher Error**")
        traceback.print_exc(file=sys.stdout)
