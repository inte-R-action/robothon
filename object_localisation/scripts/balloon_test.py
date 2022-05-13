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

weight_path=ROOT_DIR+"/mask_rcnn_object_0015.h5"
box= Mask_Rcnn_object_detection(weight_path)

image = skimage.io.imread(ROOT_DIR+"/dataset/IMG_20220503_150827.jpg")
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

info , _, _ = box.detect_object(image)

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