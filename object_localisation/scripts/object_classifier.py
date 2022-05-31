#* Author: inte-R-action Team
#* Date: 29-May-2022
# *
#* University of Bath
#* Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
#* Centre for Autonomous Robotics (CENTAUR)
#* Department of Electronics and Electrical Engineering
#*
#* Description: Mask-RCNN object detection and localization 

import os, sys
# Root directory of the project
ROOT_DIR = os.path.dirname(__file__)
# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
sys.path.append(ROOT_DIR+"/Mask_RCNN")
sys.path.append(ROOT_DIR+"/Mask_RCNN/mrcnn")
import Mask_RCNN.mrcnn
import Mask_RCNN.mrcnn.config
import Mask_RCNN.mrcnn.model
#import Mask_RCNN.mrcnn.visualize
import cv2
import os
import skimage
import numpy as np
import math
PI = math.pi


# read label name and id from external text file(object_names.txt)
def read_class_names():
    CLASS_NAMES = []
    with open(os.path.join(ROOT_DIR,'object_names.txt')) as t:
        lines = t.readlines()
        for line in lines:
            label, num = line.split(":")
            CLASS_NAMES.append(label)
    return CLASS_NAMES


class SimpleConfig(Mask_RCNN.mrcnn.config.Config):
    NAME = "coco_inference"
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    no_objects = len(read_class_names())
    NUM_CLASSES = no_objects + 1


class Mask_Rcnn_object_detection():

    def __init__(self, weight_path):
        super().__init__()
        self.model = Mask_RCNN.mrcnn.model.MaskRCNN(mode="inference",
                                                    config=SimpleConfig(),
                                                    model_dir=os.getcwd())
        self.model.load_weights(filepath=weight_path,
                                by_name=True)

        # class properties
        self.object_names = read_class_names()
        self.Bplace_id = self.object_names.index('bplace') + 1
        self.NO_Bplace = 0
        self.box_detected = False
        self.all_detected = False
        self.compute_angle = False
        self.class_ids = []
        self.masks = []
        self.scores = []
        self.class_names = []
        self.new_view = None

        # setting for txt on picture
        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        self.font_scale = 1
        self.font_color = np.random.randint(0, 255, (len(self.object_names)+1, 3))
        self.font_width = 1

    def predict(self, image):
        objs_to_detect = 12
        successful_detection = False

        self.box_detected = False
        self.all_detected = False
        self.compute_angle = False

        r = self.model.detect([image])
        r = r[0]
        # boxes = r['rois']
        CLID = r['class_ids']
        MSK = r['masks']
        SCR = r['scores']

        # 0 < detected_objects < 13
        if (len(CLID) > 0) and (len(CLID) <= objs_to_detect):
            print('Up to 12 Objects Are Detected')
            successful_detection = True
            self.update_detected_objects(CLID, MSK, SCR)

        # detected_objects > 12 (excess detection)
        elif len(CLID) > objs_to_detect:
            print('More Than 12 Objects Are Detected!')
            box_id = self.object_names.index('box')
            if box_id in CLID:
                successful_detection = True
                self.all_detected = False
                self.box_detected = False
                bidx = np.where(CLID == box_id)  # index of the box in CLID
                _, box_center, _ = self.find_obj_pose(MSK[:, :, bidx])
                self.new_view = box_center

        return successful_detection

    # This function update the list of detected variables with new funding
    # (once the camera is fixed to one pose it happens sometimes that due to
    # light condition number of detected objs fluctuates. Therefore, we use update
    # function to add those new items that haven't been recognized before )
    def update_detected_objects(self, CLID, mask, score):
        # there are 12 objects to detect on the box with 11 of them are unique
        unique_objs = 11

        for idx, a in enumerate(CLID):
            if not (a in self.class_ids) or (a == self.Bplace_id and self.NO_Bplace < 2):
                self.class_ids.append(a)
                # class_id number starts from 1 but class_names list starts from 0
                self.class_names.append(self.object_names[a - 1])

                if (a == self.Bplace_id):
                    idx = idx
                    self.NO_Bplace += 1

                self.masks.append(mask[:, :, idx])
                self.scores.append(score[idx])

        # check if 11 of detected objects are unique(2-times bplace)
        if len(np.unique(self.class_ids)) == unique_objs:
        #if ('bholder' in self.class_names) and ( 'blue_button' in self.class_names) and ('red_button' in self.class_names) and ('box' in self.class_names):
            self.all_detected = True

        # check if box is detected
        if 'box' in self.class_names:
            self.box_detected = True

    # this function erase the history of detection
    def erase_history(self):
        self.NO_Bplace = 0
        self.box_detected = False
        self.all_detected = False
        self.compute_angle = False
        self.class_ids = []
        self.masks = []
        self.scores = []
        self.class_names = []
        self.new_view = None

    # show boxes, names and center of box on image
    def visualize(self, image, bounding_Boxes, center_points, angle):

        board_width = 200
        RGB_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # add extra board to the side of the image to print names
        board = np.zeros((image.shape[0], board_width, image.shape[2])).astype('uint8')
        image_board = np.concatenate((RGB_image, board), axis=1)

        for a in range(len(bounding_Boxes)):#self.class_ids)):
            if bounding_Boxes[a] == []:
                # draw a circle in a center point of each object
                cv2.circle(image_board, (center_points[a][0], center_points[a][1]), 2, (255, 0, 0), 2)
            else:
                obj_score = self.scores[a]
                obj_score = round(obj_score, 3)  # round decimal float point up to 3
                # random color to show each object
                c = [int(self.font_color[a - 1, 0]), int(self.font_color[a - 1, 1]), int(self.font_color[a - 1, 2])]

                # draw minimum area bounding box around each object
                cv2.drawContours(image_board, [bounding_Boxes[a]], -1, c, 4)

                # draw a circle in a center point of each object
                cv2.circle(image_board, (center_points[a][0], center_points[a][1]), 2, (0, 255, 0), 2)

                # write a object name and score on the image board
                name_coordinate = (650, 30 + (a * 23))
                score_coordinate = (bounding_Boxes[a][2, 0], bounding_Boxes[a][2, 1])
                cv2.putText(image_board, self.class_names[a], name_coordinate, self.font, 1, c, 2)
                cv2.putText(image_board, str(obj_score), score_coordinate, self.font, self.font_scale, (0, 0, 255),
                            self.font_width)
                if angle:
                    show_angle = round(angle, 3)

                elif angle == 0: 
                    show_angle= 0

                else:
                    show_angle = None

                cv2.putText(image_board, f'angle: {str(show_angle)}', (650, 350), self.font, 1, (0, 0, 255), 2)
                # cv2.putText(image_board, str(show_angle), (720, 350), self.font, self.font_scale, (0, 0, 255),
                #             self.font_width)

        cv2.imshow('img', image_board)
        # cv2.waitKey(0)

    def find_obj_pose(self, masks):

        no_masks = len(masks)
        img_height = masks[0].shape[0]
        img_width = masks[0].shape[1]

        center_points = []
        angles = []
        bounding_boxes = []
        for m in range(no_masks):
            # convert mask from false-true format to 0-255 format
            msk = masks[m].reshape(masks[m].shape[0], masks[m].shape[1])
            binary_mask = np.zeros((img_height, img_width)).astype("uint8")
            binary_mask[np.where(msk == True)] = 255

            # find the contour
            contour, hierarchy = cv2.findContours(binary_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # find the oriented rectangular  bounding the object 
            rect = cv2.minAreaRect(contour[0])  # (x_center, y_center), (W,H), angle
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # extract information of minimum box (center point, (l,w), angle)
            (x_center, y_center), _, angle = rect
            center_points.append([int(x_center), int(y_center)])
            angles.append(angle)
            bounding_boxes.append(box)

        return bounding_boxes, center_points, angles

    def base_rotation(self, bounding_boxes, class_names):

        candidate_objs = ['box', 'bholder']
        box_rotation = []

        for i in candidate_objs:

            # extract index
            box_idx = class_names.index(i)

            # extract box
            box = bounding_boxes[box_idx]

            # compute length of sides
            L = []
            for a in range(1, 3):
                diff = box[a, :] - box[a - 1, :]
                L.append(np.sqrt(pow(diff[0], 2) + pow(diff[1], 2)))

            # find bigger side
            longSide_idx = L.index(max(L))
            longSide_points = (box[longSide_idx, :], box[longSide_idx + 1, :])

            # find angle of the bigger side and horizontal line
            d = L[longSide_idx]
            diffx = abs(longSide_points[0][0] - longSide_points[1][0])
            angle = math.acos(diffx / d) * (180 / PI)
            box_rotation.append(angle)

        # for now rotation of the box is equal to rotation of the bholder
        return box_rotation[1]

    def find_world_rotation(self, center_points, base_angle, class_names):

        red_center = center_points[class_names.index('red_button')]
        blue_center = center_points[class_names.index('blue_button')]

        diff_x = abs(red_center[0]-blue_center[0])
        diff_y = abs(red_center[1]-blue_center[1])
        if diff_x != 0:
            angle2 = math.degrees(math.atan(diff_y/diff_x))
        else:
            angle2 = 90
        print(angle2)
        base_angle = angle2

        # lcd is at right side of the image(reference)
        if red_center[0] > blue_center[0]:

            # if y_lcd > y_battery (negative angle)
            if red_center[1] > blue_center[1]:
                box_angle = -base_angle

            # if y_lcd < y_battery (positive angle)
            elif red_center[1] <= blue_center[1]:
                box_angle = base_angle

        # lcd is in  left side of the  image(90 degree shift)
        elif red_center[0] < blue_center[0]:

            # if y_lcd > y_battery (negative angle)
            if red_center[1] > blue_center[1]:
                box_angle = base_angle - 180

            # if y_lcd < y_battery (positive angle)
            elif red_center[1] <= blue_center[1]:
                box_angle = -base_angle + 180

        else:
            # if y_lcd > y_battery (negative angle)
            if red_center[1] < blue_center[1]:
                box_angle = 90

            # if y_lcd < y_battery (positive angle)
            elif red_center[1] >= blue_center[1]:
                box_angle = -90
            # print('red and blue buttons vague relative position')

        return box_angle

    def detect_object(self, image):

        # apply weights to detect object on the box
        detect_success = self.predict(image)

        # more than zero object is detected
        if detect_success:
            # find center point and angle of each object
            bounding_boxes, center_points, _ = self.find_obj_pose(self.masks)

            # print if mrcnn detects all objects or at least the box
            print('all_detected:', self.all_detected, '-box_detected:', self.box_detected)

            # output array
            output = [self.class_ids, self.class_names, self.scores, [], center_points]

            # calculate the box angle
            if self.all_detected:
                angle = self.base_rotation(bounding_boxes, self.class_names)
                box_rotation = self.find_world_rotation(center_points, angle, self.class_names)
                output[3] = box_rotation

            # if not all objects are detected but the box is detected
            elif self.box_detected:
                self.new_view = center_points[self.class_names.index('box')]
                print('Look For a Better View at ', self.new_view)

            # more than 12 object is detected
            elif (self.box_detected is False) and (self.new_view is not None):
                print('unusual detection better to go to new view: ', self.new_view)

            return detect_success, output, bounding_boxes, self.new_view

        else:
            print('Detection Was Not Successful!')
            return detect_success, [], [], []
