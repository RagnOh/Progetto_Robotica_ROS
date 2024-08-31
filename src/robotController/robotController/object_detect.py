import cv2
import torch
import numpy as np


class object_detection():

    b1=0
    b2=0
    b3=0
    b4=0
   
    def __init__(image):
        self.image=image
        self.image_detected=image
        self.model=model=torch.hub.load('ultralytics/yolov5','custom', weights = 'best-6.pt') 


    def start_inference(image):

        results= self.model(image)

        for detection in results.xyxz[0]:

            class_id= int(detection[0])
            confidence = detection[1]
            bbox = detection[2:].tolist()

            class_name = self.model.names[class_id]

            cv2.rectangle(image, (int(bbox[0])),(int(bbox[1])),(int(bbox[2])),(int(bbox[3])),(0,255,0),2)  

            self.b1= int(bbox[0])
            self.b2= int(bbox[1])
            self.b3= int(bbox[2])
            self.b4= int(bbox[3])

        
        cv2.imshow('Risultati',image)
        self.image=image
        cv2.waitKey(0)  

    def start_inference2(image):
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)   

        lower_red = np.array([160,100,100])
        upper_red = np.array([179,255,255]) 
 
        msk= cv2.inRange(hsv, lower_red, upper_red)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel , iteration=2)
        mask = cv2.dilate(mask, kernel , iteration=2)

        contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:

            approx = cv2.approxPolyDP(cnt,0.02*cv2.arcLength(cnt, True), True)

            if len(approx) == 4:
                x,y,w,h= cv2.boundingRect(approx)
                cv2.rectangle(image, (x,y), (x+w, y+h),(0,255,0),3)

        self.image=image        
        

    def getCenter():
        centro_w=(self.b2-self.b1)/2
        centro_h=(self.b3-self.b1)/2

        return centro_w,centro_h    