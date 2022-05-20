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
from database_funcs import database
from object_classifier import Mask_Rcnn_object_detection

weight_path="mask_rcnn_object_0020.h5"
Robothon= Mask_Rcnn_object_detection(weight_path)

image = skimage.io.imread("dataset/IMG_20220503_150839_1.jpg")
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

#info , _, _ = box.detect_object(image)
success, info, bounding_Boxes, new_view = Robothon.detect_object(image)
print(info)
ID = info[0][0]
Names = info[1][0]
confident = info[2][0]
box_angle = info[3]
Cpoints = info[4][0]
print(box_angle)

if success:
    if Robothon.all_detected:
        Robothon.visualize(image, ID, Names, confident, bounding_Boxes, Cpoints)

    elif Robothon.box_detected:
        Robothon.visualize(image, ID, Names, confident, bounding_Boxes, Cpoints)

    else:
        Robothon.visualize(image, ID, Names, confident, bounding_Boxes, Cpoints)

else:
    print('Searching..')

#box.visualize(image, class_ids, class_names, scores, BOXES, AllMasks)

#image = skimage.io.imread("C:/Users/inter/Documents/GitHub/maskrcnn_tf1_training_Arya/Dataset_taskbox/val/IMG_20220503_150933.jpg")
#image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
#box.extract_prediction(image,read_class_names())


# predict my Mask_RCNN trained weights 
#mrcnn.visualize.display_instances(image=image, 
                                 # boxes=r['rois'], 
                                  #masks=r['masks'], 
                                  #class_ids=r['class_ids'], 
                                  #class_names=read_class_names(), 
                                  #scores=r['scores'])
