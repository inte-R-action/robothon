#!~/anaconda3/envs/ros/bin/python3.6

import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import Pose, PoseStamped
from math import sin, cos, radians
from rs_cam import rs_cam
from object_localisation.msg import object_state
import cv2
import pyrealsense2 as rs
import argparse

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
    def __init__(self, display):
        self.display = display
        frame_id = 'Realsense_node'
        rospy.init_node(frame_id, anonymous=True)
        self.tf_listener_ = TransformListener()

        self.cam = rs_cam(depth=True)
        classsifier = None
        self.obj_pub = obj_class(frame_id)

    def cam2rob_transform(self, pose):
        p_in_base = None
        # if self.tf_listener_.frameExists("/base_link") and self.tf_listener_.frameExists("/camera_frame"):
        t = self.tf_listener_.getLatestCommonTime("/base_link", "/camera_frame")
        # p1 = PoseStamped()
        # p1.header.frame_id = "fingertip"
        # p1.pose.orientation.w = 1.0    # Neutral orientation
        print(t)
        print(pose)
        p_in_base = self.tf_listener_.transformPose("/base_link", pose)
        print("Position of the fingertip in the robot base:")
        print(p_in_base)

        return p_in_base

    def create_pose(self, x, y, angle, dist):

        cameraInfo = self.cam.depth_intrinsics
        _intrinsics = rs.intrinsics()
        _intrinsics.width = cameraInfo.width
        _intrinsics.height = cameraInfo.height
        _intrinsics.ppx = cameraInfo.ppx
        _intrinsics.ppy = cameraInfo.ppy
        _intrinsics.fx = cameraInfo.fx
        _intrinsics.fy = cameraInfo.fy
        ###_intrinsics.model = cameraInfo.distortion_model
        #_intrinsics.model  = rs.distortion.none
        #_intrinsics.coeffs = [i for i in cameraInfo.D]
        result = rs.rs2_deproject_pixel_to_point(cameraInfo, [x, y], dist)

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

        # Get frames from camera
        frames = self.cam.pipeline.wait_for_frames()
        color_image, depth_colormap, depth_image = self.cam.depth_frames(frames)
        if self.display:
            cv2.imshow("RS colour image", color_image)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()

        # Classify image
        identified_objects = [[1, "button"]]#self.classifier.detect(color_image)

        for object in identified_objects:
            x = 0
            y = 0
            angle = 0
            dist = depth_image[x, y]
            obj_cam_pose = self.create_pose(x, y, angle, dist)
            obj_rob_pose = self.cam2rob_transform(obj_cam_pose)
            
            if obj_rob_pose is not None:
                obj_id = object[0]
                obj_name = object[1]
                self.obj_pub.publish(obj_rob_pose, obj_id, obj_name)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run realsense vision recognition ROS node')
    parser.add_argument('--disp', '-V',
                        help='Enable displaying of camera image',
                        default=True,
                        action="store")
    args = parser.parse_known_args()[0]                 
    try:
        localiser = object_localiser(args.disp)
        while (not rospy.is_shutdown()):
            localiser.run_localisation()
    except rospy.ROSInterruptException:
        print("realsense_run ROS exception")
    # except Exception as e:
    #     print("**Image Error**")
    #     traceback.print_exc(file=sys.stdout)
    # finally:
    #     cam.pipeline.stop()
    #     cv2.destroyAllWindows()
    #     plt.close('all')