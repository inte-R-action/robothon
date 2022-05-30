#* Author: inte-R-action Team
#* Date: 29-May-2022
# *
#* University of Bath
#* Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
#* Centre for Autonomous Robotics (CENTAUR)
#* Department of Electronics and Electrical Engineering
#*
#* Description: upload objects geometrical information to the database 


from math import sin, cos, radians
from rs_cam import rs_cam
import cv2
import pyrealsense2 as rs
import argparse
import traceback, sys
from object_classifier import Mask_Rcnn_object_detection
import os
from database_funcs import database
import numpy as np
import math
ROOT_DIR = os.path.dirname(__file__)


class object_localiser:
    def __init__(self, display):
        self.display = display

        weight_path=os.path.join(ROOT_DIR, "mrcnn_weights/mrcnn_135obj_BGR_0029.h5")
        print(weight_path)
        self.classifier = Mask_Rcnn_object_detection(weight_path)
        self.db = database()
        self.predictions = []
        self.obj_tab_col_names, _ = self.db.query_table('detected_objects', rows=0)

        self.cam = rs_cam(depth=True)
        self.cam.depth_frames(self.cam.pipeline.wait_for_frames())
        # self.cameraInfo = self.cam.depth_intrinsics
        print(f"depth intrinsics: {self.cam.depth_intrinsics}")
        print(f"colour intrinsics: {self.cam.colour_intrinsics}")
        self.cameraInfo = rs.intrinsics()
        self.cameraInfo.width = 640
        self.cameraInfo.height = 480
        self.cameraInfo.ppx = 324.7422
        self.cameraInfo.ppy = 225.8990
        self.cameraInfo.fx = 630.9241
        self.cameraInfo.fy = 629.5015
        # self.cameraInfo.coeffs = [Radial: [0.140784267654903,-0.223962950366363] Tangential: [0,0]]
        # self.cameraInfo.model = cameraInfo.distortion_model
        self.cameraInfo.model = rs.distortion.none
        # self.cameraInfo.coeffs = [i for i in cameraInfo.D]
        print(f"custom intrinsics: {self.cameraInfo}")

    def depth_search(self, x, y, depth_image):
        depth = 0
        max_i = 10
        for i in range(max_i):
            new_x_max = min(x+i, depth_image.shape[0])
            new_x_min = max(x-i, 0)
            new_y_max = min(y+i, depth_image.shape[1])
            new_y_min = max(y-i, 0)
            #print(new_x_min,new_x_max, new_y_min,new_y_max)
            arr = depth_image[new_x_min:new_x_max, new_y_min:new_y_max]
            arr[arr == 0] = np.nan
            if arr.any():
                if not np.isnan(arr).all():
                    depth = np.nanmean(arr)

            if depth != 0:
                break
        
        return depth

    def run_localisation(self):
        no_attempts = 4
        self.classifier.erase_history()

        for att in range(no_attempts):
            print(f"Attempt no: {att}")
            # Get frames from camera
            frames = self.cam.pipeline.wait_for_frames()
            color_image, depth_colormap, depth_image = self.cam.depth_frames(frames)
            color_image= cv2.cvtColor(color_image,  cv2.COLOR_RGB2BGR)

            if self.display:
                cv2.imshow("RS colour image", np.hstack((cv2.cvtColor(color_image,  cv2.COLOR_BGR2RGB), depth_colormap)))
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()

            # Classify image
            detect_success, self.predictions, bounding_Boxes, new_view = self.classifier.detect_object(color_image)

            ouput_predictions = []
            if detect_success:
                ID = self.predictions[0]
                Names = self.predictions[1]
                confident = self.predictions[2]
                box_angle = self.predictions[3]
                Cpoints = self.predictions[4]
                ID.append(99)
                Names.append('test')
                confident.append(1)
                box_angle = self.predictions[3]
                Cpoints.append([320, 240])
                depths = [float(depth_image[p[1], p[0]]) for i, p in enumerate(Cpoints)]
                bounding_Boxes.append([])

                if self.classifier.all_detected:

                    # Update name of bplaces to be red and blue based on distance from box center
                    if Names.count('bplace') > 0:
                        bplace_idxs = [ i for i in range(len(Names)) if Names[i] == 'bplace' ]
                        box_idx = Names.index('box')
                        dist = []
                        for idx in bplace_idxs:
                            dist.append(math.sqrt( (Cpoints[idx][0]-Cpoints[box_idx][0])**2 + (Cpoints[idx][1]-Cpoints[box_idx][1])**2))
                        
                        min_idx = bplace_idxs[min(range(len(dist)), key=dist.__getitem__)]
                        max_idx = bplace_idxs[max(range(len(dist)), key=dist.__getitem__)]

                        Names[min_idx] = 'bplace_red'
                        Names[max_idx] = 'bplace_blue'
                    
                    for p, id in enumerate(ID):
                        # Wider depth search if invalid point
                        depth2 = depth_image.get_distance(int(Cpoints[p][0]),int(Cpoints[p][1]))
                        if depths[p] == 0:
                            depths[p] = self.depth_search(Cpoints[p][1], Cpoints[p][0], depth_image)
                        print(f"depth 1: {depths[p]}, depth 2: {depth2}")
                        # Convert pixels to spatial coords
                        result = rs.rs2_deproject_pixel_to_point(self.cameraInfo, [Cpoints[p][0], Cpoints[p][1]], depths[p])

                        x = result[0]
                        y = result[1]
                        z = result[2]

                        # make sure data types are correct for database
                        ouput_predictions.append([])
                        ouput_predictions[-1].append(int(id))
                        ouput_predictions[-1].append(str(Names[p]))
                        ouput_predictions[-1].append(float(confident[p]))
                        ouput_predictions[-1].append(float(box_angle))
                        ouput_predictions[-1].append(float(x))
                        ouput_predictions[-1].append(float(y))
                        ouput_predictions[-1].append(float(z))

                    self.classifier.visualize(color_image, bounding_Boxes, Cpoints, box_angle)

                    print(self.predictions)
                    if ouput_predictions:
                        self.publish_to_database(ouput_predictions)
                    return

                elif self.classifier.box_detected:
                    depth = float(depth_image[new_view[1], new_view[0]])
                    depth2 = depth_image.get_distance(int(new_view[0]),int(new_view[1]))
                    if depth == 0:
                        depth = self.depth_search(new_view[1], new_view[0], depth_image)
                    print(f"depth: {depth}, depth2: {depth2}")
                    result = rs.rs2_deproject_pixel_to_point(self.cameraInfo, [new_view[0], new_view[1]], depth)

                    x = result[0]
                    y = result[1]
                    z = result[2]
                    ouput_predictions = [[0, "new_view", 1.0, 0.0, x, y, z]]

                    self.classifier.visualize(color_image, bounding_Boxes, Cpoints, box_angle)


                elif (self.classifier.box_detected is False) and (new_view):
                    self.classifier.visualize(color_image, [], [], [])
                    print('unusual detection better to go to new view: ', self.new_view)
                    break
            
            else:
                print('Nothing detected, this sucks..')
                self.classifier.visualize(color_image, [], [], [])
                break
        
        if new_view:
            depth = float(depth_image[new_view[1], new_view[0]])
            depth2 = depth_image.get_distance(int(new_view[0]),int(new_view[1]))
            print(f"depth: {depth}")
            if depth == 0:
                depth = self.depth_search(new_view[1], new_view[0], depth_image)
            print(f"depth: {depth}, depth2: {depth2}")
            result = rs.rs2_deproject_pixel_to_point(self.cameraInfo, [new_view[0], new_view[1]], depth)

            x = result[0]
            y = result[1]
            z = result[2]
            ouput_predictions = [[0, "new_view", 1.0, 0.0, x, y, z]]
            if ouput_predictions:
                self.publish_to_database(ouput_predictions)
        else:
            print("No new view to go to")

    def publish_to_database(self, ouput_predictions):
        # Delete old rows for user
        sql_cmd = f"""DELETE FROM detected_objects"""
        self.db.gen_cmd(sql_cmd)
        
        # Insert new data
        self.db.insert_data_list("detected_objects", self.obj_tab_col_names[1:], ouput_predictions)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run realsense vision recognition ROS node')
    parser.add_argument('--disp', '-V',
                        help='Enable displaying of camera image',
                        default=True,
                        action="store")
    args = parser.parse_known_args()[0]                 
    localiser = None
    try:
        localiser = object_localiser(args.disp)
        while (True):
            try:
                localiser.run_localisation()
            except Exception as e:
                print("**Image Error**")
                traceback.print_exc(file=sys.stdout)
            cv2.waitKey(1)
    except Exception as e:
        print("**Image Error**")
        traceback.print_exc(file=sys.stdout)
    finally:
        if localiser is not None:
            localiser.cam.pipeline.stop()
        cv2.destroyAllWindows()
