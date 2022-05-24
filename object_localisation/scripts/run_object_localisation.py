
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
ROOT_DIR = os.path.dirname(__file__)


class object_localiser:
    def __init__(self, display):
        self.display = display

        weight_path=os.path.join(ROOT_DIR, "mrcnn_weights/mrcnn_99obj_BGR_0015.h5")
        print(weight_path)
        self.classifier = Mask_Rcnn_object_detection(weight_path)
        self.db = database()
        self.predictions = []
        self.obj_tab_col_names, _ = self.db.query_table('detected_objects', rows=0)

        self.cam = rs_cam(depth=True)
        self.cam.depth_frames(self.cam.pipeline.wait_for_frames())
        self.cameraInfo = self.cam.depth_intrinsics

    def depth_search(self, x, y, depth_image):
        depth = 0
        max_i = 5
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

        for att in range(no_attempts):
            print(f"Attempt no: {att}")
            # Get frames from camera
            frames = self.cam.pipeline.wait_for_frames()
            color_image, depth_colormap, depth_image = self.cam.depth_frames(frames)
            color_image= cv2.cvtColor(color_image,  cv2.COLOR_RGB2BGR)

            if self.display:
                cv2.imshow("RS colour image", np.hstack((color_image, depth_colormap)))
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
                depths = [float(depth_image[p[1], p[0]]) for i, p in enumerate(Cpoints)]

                if self.classifier.all_detected:
                    for p, id in enumerate(ID):
                        # Wider depth search if invalid point
                        if depths[p] == 0:
                            depths[p] = self.depth_search(Cpoints[p][1], Cpoints[p][0], depth_image)
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
                    self.classifier.erase_history()
                    return

                elif self.classifier.box_detected:
                    depth = float(depth_image[new_view[1], new_view[0]])
                    if depth == 0:
                        depth = self.depth_search(new_view[1], new_view[0], depth_image)
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
            if depth == 0:
                depth = self.depth_search(new_view[1], new_view[0], depth_image)
            result = rs.rs2_deproject_pixel_to_point(self.cameraInfo, [new_view[0], new_view[1]], depth)

            x = result[0]
            y = result[1]
            z = result[2]
            ouput_predictions = [[0, "new_view", 1.0, 0.0, x, y, z]]
            if ouput_predictions:
                self.publish_to_database(ouput_predictions)
            self.classifier.erase_history()
        else:
            self.classifier.erase_history()
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
        localiser.cam.pipeline.stop()
        cv2.destroyAllWindows()
