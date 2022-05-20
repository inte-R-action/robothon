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
        self.box_detected = False
        self.all_detected = False

        # setting for txt on picture
        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        self.font_scale = 1
        self.font_color = np.random.randint(0, 255, (len(self.object_names), 3))
        self.font_width = 1

    def predict(self, image):

        successful_detection = False

        # there are 12 objects to detect on the box with 11 of them are unique
        objs_to_detect = 12
        unique_objs = 11

        self.box_detected = False
        self.all_detected = False
        r = self.model.detect([image])
        r = r[0]
        boxes = r['rois']
        class_ids = r['class_ids']
        masks = r['masks']
        scores = r['scores']
        class_names = []
        
        for a in class_ids:
            class_names.append(self.object_names[a-1])
  

        # 0 < detected_objects < 13
        if (len(class_ids) > 0) and (len(class_ids) <= objs_to_detect):

            print('Up to 12 Objects Are Detected')

            successful_detection = True

            for a in range(len(class_ids)):
                # extract class name based on predicted class_id
                class_names.append(
                    self.object_names[class_ids[a] - 1])  # class_id starts from 1 and class_name list start from 0

            # check if 11 of detected objects are unique(2-times bplace)
            #if len(np.unique(class_ids)) == unique_objs:
            if ('box' in class_names) and('bholder' in class_names) and('lcd' in class_names) and('blue_button' in class_names) and ('battery' in class_names):
                self.all_detected = True

            # check if box is detected
            if 'box' in class_names:
                self.box_detected = True
            

        # detected_objects > 12 (excess detection)
        elif len(class_ids) > objs_to_detect:
            print('More Than 12 Objects Are Detected!')

            box_id = self.object_names.index('box')
            if box_id in class_ids:
                successful_detection = True
                self.all_detected = False
                self.box_detected = True
                bidx = np.where(class_ids == box_id)
                class_names = ['box']
                class_ids = [box_id]
                scores = scores[bidx]
                masks = masks[bidx]
            else:
                class_names = []

        else:
            class_names = []
            print('Nothing is Detected !!')

        return successful_detection, class_ids, class_names, scores, masks

    # gather all the masks in a single image
    def all_masks_in_one(self, mask):
        img_height = mask.shape[0]
        img_width = mask.shape[1]
        no_masks = mask.shape[2]
        AllMasks = np.zeros((img_height, img_width))
        for a in range(no_masks):
            object = np.where(mask[:, :, a] == True)
            AllMasks[object] = 1
        return (AllMasks)

    # show boxes, names and center of box on image
    def visualize(self, image, class_ids, class_names, scores, bounding_Boxes, center_points, angle):

        board_width = 200
        RGB_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # add extra board to the side of the image to print names
        board = np.zeros((image.shape[0], board_width, image.shape[2])).astype('uint8')
        image_board = np.concatenate((RGB_image, board), axis=1)

        for a in range(len(class_ids)):
            obj_score = scores[a]
            obj_score = round(obj_score, 3)  # round decimal float point up to 3
            # random color to show each object
            c = [int(self.font_color[a - 1, 0]), int(self.font_color[a - 1, 1]), int(self.font_color[a - 1, 2])]

            # draw minimum area bounding box around each object
            cv2.drawContours(image_board, [bounding_Boxes[a]], -1, c, 4)

            # draw a circle in a ceneter point of each object
            cv2.circle(image_board, (center_points[a][0], center_points[a][1]), 2, (0, 255, 0), 2)

            # write a object name and score on the image board
            name_coordinate = (650, 30 + (a * 23))
            score_coordinate = (bounding_Boxes[a][2, 0], bounding_Boxes[a][2, 1])
            cv2.putText(image_board, class_names[a], name_coordinate, self.font, 1, c, 2)
            cv2.putText(image_board, str(obj_score), score_coordinate, self.font, self.font_scale, (0, 0, 255),
                                self.font_width)

            cv2.putText(image_board, 'angle:', (650, 400), self.font, self.font_scale, (0, 0, 255),
                                self.font_width)
            if angle:
               show_angle=round(angle, 3)
               cv2.putText(image_board, str(show_angle), (750, 400), self.font, self.font_scale, (0, 0, 255),
                                self.font_width)
            else:
               cv2.putText(image_board, 'NAN', (750, 400), self.font, self.font_scale, (0, 0, 255),
                                self.font_width)


        cv2.imshow('img', image_board)
        cv2.waitKey(1)

    def find_obj_pose(self, masks):
        no_masks = masks.shape[2]
        img_height = masks.shape[0]
        img_width = masks.shape[1]
        center_points = []
        angles = []
        bounding_boxes = []
        for m in range(no_masks):
            # convert mask from false-true format to 0-255 format
            binary_mask = np.zeros((img_height, img_width)).astype("uint8")
            binary_mask[np.where(masks[:, :, m] == True)] = 255

            # find the contour
            contour, hierarchy = cv2.findContours(binary_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # find the oriented rectangular bounding the object 
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

        # for now
        return box_rotation[1]

    def find_world_rotation(self, center_points, angle, class_names):

        lcd_center = center_points[class_names.index('lcd')]
        battery_center = center_points[class_names.index('battery')]

        # lcd is at right side of the image(reference)
        if lcd_center[0] > battery_center[0]:

            # if y_lcd > y_battery (negative angle)
            if lcd_center[1] > battery_center[1]:
                box_angel = -angle

            # if y_lcd < y_battery (positive angle)
            elif lcd_center[1] < battery_center[1]:
                box_angel = angle

        # lcd is in  left side of the  image(90 degree shift)
        elif lcd_center[0] < battery_center[0]:

            # if y_lcd > y_battery (negative angle)
            if lcd_center[1] > battery_center[1]:
                box_angel = -angle-90

            # if y_lcd < y_battery (positive angle)
            elif lcd_center[1] < battery_center[1]:
                box_angel = angle+90

        else:
            print('lcd and battery vague relative position')

        return box_angel

    def detect_object(self, image):

        # apply weights to detect object on the box
        detect_success, class_ids, class_names, scores, masks = self.predict(image)

        # equal or less than 12 objects are detected
        if detect_success:

            # find center point and angle of each object
            bounding_boxes, center_points, _ = self.find_obj_pose(masks)

            # print if mrcnn detects all objects or at least the box
            print('all_detected:', self.all_detected, '-box_detected:', self.box_detected)

            # output array
            output = [[class_ids], [class_names], [scores], [], [center_points]]

            # new poit for better view
            new_view = []

            # all detected
            if self.all_detected:
                angle = self.base_rotation(bounding_boxes, class_names)
                box_rotation = self.find_world_rotation(center_points, angle, class_names)
                output[3] = box_rotation

            # if not all objects are detected but the box is detected
            elif self.box_detected:
                new_view = center_points[class_names.index('box')]
                print('Look For a Better View at ', new_view)

            else:
                print('Search For The Box')

            return detect_success, output, bounding_boxes, new_view

        else:
            print('Detection Was Not Successful')
            return detect_success, [], [], []
