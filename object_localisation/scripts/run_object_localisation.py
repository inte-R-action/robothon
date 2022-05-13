
from math import sin, cos, radians
from rs_cam import rs_cam
import cv2
import pyrealsense2 as rs
import argparse
import traceback, sys
from object_classifier import Mask_Rcnn_object_detection
import os
from database_funcs import database
import pickle
ROOT_DIR = os.path.dirname(__file__)


class object_localiser:
    def __init__(self, display):
        self.display = display

        self.cam = rs_cam(depth=True)
        
        self.cam.depth_frames(self.cam.pipeline.wait_for_frames())
        cameraInfo = self.cam.depth_intrinsics
        cameraInfoDict =	{
                    "coeffs": cameraInfo.coeffs,
                    "fx": cameraInfo.fx,
                    "fy": cameraInfo.fy,
                    "height": cameraInfo.height,
                    # "model": cameraInfo.model,
                    "ppx": cameraInfo.ppx,
                    "ppy": cameraInfo.ppy,
                    "width": cameraInfo.width
                    }
        pickle.dump( cameraInfoDict, open( ROOT_DIR+"/cameraInfo.p", "wb" ) )
        weight_path=ROOT_DIR+"/mask_rcnn_object_0015.h5"
        self.classifier = Mask_Rcnn_object_detection(weight_path)
        self.db = database()
        self.predictions = []
        self.obj_tab_col_names, _ = self.db.query_table('detected_objects', rows=0)

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
        self.predictions, _, _ = self.classifier.detect_object(color_image)
        self.predictions = [list(x) for x in zip(*self.predictions)]
        for p in self.predictions:
            p[-1:] = p[-1]
            p[0] = int(p[0])
            p[1] = str(p[1])
            p[2] = float(p[2])
            p[3] = float(p[3])
            p[4] = int(p[4])
            p[5] = int(p[5])
            p.append(float(depth_image[p[5], p[4]]))

        print(self.predictions)
        self.publish_to_database()

    def publish_to_database(self):
        # Delete old rows for user
        sql_cmd = f"""DELETE FROM detected_objects"""
        self.db.gen_cmd(sql_cmd)
        
        # Insert new data
        self.db.insert_data_list("detected_objects", self.obj_tab_col_names[1:], self.predictions)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run realsense vision recognition ROS node')
    parser.add_argument('--disp', '-V',
                        help='Enable displaying of camera image',
                        default=False,
                        action="store")
    args = parser.parse_known_args()[0]                 
    try:
        localiser = object_localiser(args.disp)
        while (True):
            localiser.run_localisation()
            cv2.waitKey(1)
    except Exception as e:
        print("**Image Error**")
        traceback.print_exc(file=sys.stdout)
    finally:
        localiser.cam.pipeline.stop()
        cv2.destroyAllWindows()
