import cv2
import numpy as np


class Camera_Calibration():

    PIX_MM_RATIO = 1.33
    X_INITIAL_POS = 150.0 #Centro X dove robot fa la foto
    Y_INITIAL_POS = 120.0 #Centro Y dove robot fa la foto
    CAMERA_WIDTH = 480.0
    CAMERA_HEIGHT = 480.0

    def __init__(self):
        self.pixRatio = 1.33


    

    def findRobotCoordinates(self,x,y):
        
        #Prendo come riferimento il centro della camera
        x_centered = x-(Camera_Calibration.CAMERA_WIDTH/2.0)
        y_centered = y-(Camera_Calibration.CAMERA_HEIGHT/2.0)

        #Asse camera è invertito rispetto ad assi robot, devo dunque aggiustare

        x_centered=x_centered * -1.0
        y_centered=y_centered * -1.0
        
      

        #Scalo in base a rapporto pixels/mm
        x_scaled = x_centered / Camera_Calibration.PIX_MM_RATIO
        y_scaled = y_centered / Camera_Calibration.PIX_MM_RATIO

        #print(x_scaled)
        #print(y_scaled)


        #Sommo il risultato alla posizione della pinza del robot
        x_final = Camera_Calibration.X_INITIAL_POS + x_scaled
        y_final = Camera_Calibration.Y_INITIAL_POS + y_scaled

        Punto= []
        
        Punto.append(x_final)
        Punto.append(y_final)

        #print(Punto[0])
        #print(Punto[1])

        return Punto






