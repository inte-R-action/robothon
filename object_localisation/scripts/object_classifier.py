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

# read label name and id from external text file(object_names.txt)
def read_class_names():
    CLASS_NAMES = []
    with open(ROOT_DIR+'/object_names.txt') as t:
         lines =t.readlines()
         for line in lines:
             label, num=line.split(":")
             CLASS_NAMES.append(label)

    return CLASS_NAMES




class SimpleConfig(Mask_RCNN.mrcnn.config.Config):
    NAME = "coco_inference"
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    no_objects=len(read_class_names())
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
          self.object_names= read_class_names()

          # setting for txt on picture          
          self.font= cv2.FONT_HERSHEY_COMPLEX_SMALL
          self.font_scale=1
          self.font_color = np.random.randint(0, 255, (len(self.object_names)*3, 3) )
          self.font_width=1



      def predict(self, image):
          r = self.model.detect([image])
          r = r[0]
          boxes=r['rois']
          class_ids=r['class_ids']
          masks=r['masks']
          scores=r['scores']
          class_names=[]
          for a in range(len(class_ids)):
              # extract class name based on predicted class_id
              class_names.append(self.object_names[class_ids[a]-1]) # class_id starts from 1 and class_name list start from 0
   
          return class_ids, class_names, scores, masks 


      # gather all the masks in a single image 
      def all_masks_in_one(self, mask):
          img_height=mask.shape[0]
          img_width=mask.shape[1]
          no_masks= mask.shape[2]
          AllMasks=np.zeros((img_height,img_width))
          for a in range(no_masks):
               object= np.where(mask[:,:,a]==True)
               AllMasks[object]=1
          return(AllMasks)




      def visualize(self, image, class_ids, class_names, scores, bounding_Boxes, center_points):
          #AllMasks_Inone=self.all_masks_in_one(masks) # provide all masks in single image 
          for a in range(len(class_ids)):
              obj_score=scores[a]
              obj_score=round(obj_score, 3)# round decimal float point up to 3 
              # randomm color to show each object
              c= [int(self.font_color[a-1,0]), int(self.font_color[a-1,1]) ,int(self.font_color[a-1,2])] 

              # draw minimum area bounding box around each object
              cv2.drawContours(image, [bounding_Boxes[a]], -1, c, 4) 
              
              # draw a circle in a ceneter point of each object 
            #   cv2.circle(image, center_points[a], 2, (0,255,0),2)
              
              # write a object name and score on the image 
              name_coordinate= (480,30+(a*17))
              score_coordinate=(bounding_Boxes[a][2,0],bounding_Boxes[a][2,1])
              image=cv2.putText(image, class_names[a], name_coordinate,self.font,1,c,2)
              image=cv2.putText(image, str(obj_score), score_coordinate ,self.font,self.font_scale,(0,0,255),self.font_width)
          cv2.imshow('img', image)
        #   cv2.waitKey(1)


      def find_obj_pose(self, masks):
          no_masks= masks.shape[2]
          img_height=masks.shape[0]
          img_width=masks.shape[1]
          center_points=[]
          angles=[]
          bounding_boxes=[]
          for m in range(no_masks):
            # convert mask from false, true format to 0-255 format
            binary_mask=np.zeros((img_height, img_width)).astype("uint8")
            binary_mask[np.where(masks[:,:,m]==True)]=255

            # find the countour
            countour, hierarchy = cv2.findContours(binary_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )

            # find the oriented rectangular  bounding the object 
            rect = cv2.minAreaRect(countour[0]) # (x_center, y_center), (W,H), angle
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            # extract information 
            (x_center, y_center), _, angle = rect 
            center_points.append([int(x_center), int(y_center)])
            angles.append(angle)
            bounding_boxes.append(box)

          return bounding_boxes, center_points, angles
    

      def detect_object(self,image):
           class_ids, class_names, scores, masks =self.predict(image)
           bounding_boxes, center_points, angles =self.find_obj_pose(masks)
           detection_info=[class_ids, class_names, scores, angles, center_points]
        #    print(detection_info)
           self.visualize(image, class_ids, class_names, scores, bounding_boxes, center_points)
           
           
           return detection_info, bounding_boxes, masks 