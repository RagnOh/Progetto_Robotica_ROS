import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2
from sympy.matrices import Matrix

from robotController.mover_controller import MoverController

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Float32MultiArray,Float64MultiArray
from sensor_msgs.msg import JointState

import math




class controlNode(Node):

    #Coordinate posizione robot per identificare pezzo
    X_PHOTO = 0.1
    Y_PHOTO = 0.2
    Z_PHOTO = 0.2

    #Coordinate recupero pezzo
    Z_APPROCH = 0.15  #Valore altezza prima di recuperare il pezzo per evitare collisioni
    Z_RECUPERO = 0.08 #Valore altezza per recuperare il pezzo adattato in base a calibrazione automatica


    def __init__(self):
        super().__init__('control_node')

        self.calcolatore = MoverController()
        self.actual_position = []
        self.pos1 = []
        self.posizione_inviata = []
        self.raggiunto = False
        self.take_photo = False
        self.stato = 0
        self.posizione_pezzo = []
        self.colore = 1 #1 se cubo rosso, 2 se blu
        
        
        # Publisher per inviare la posizione desiderata
        self.position_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        
        # Subscriber per confermare quando il robot ha raggiunto la posizione desiderata
        self.create_subscription(JointState, '/joint_states', self.position_reached_callback, 1)
        
        # Subscriber per ricevere la nuova posizione
        self.create_subscription(Float32MultiArray, '/point_coordinates', self.next_position_callback, 10)
        
        
        self.next_position = None

    def position_reached_callback(self, msg):
        
        self.actual_position= []
        for i in range(len(msg.position)):
            #self.get_logger().info('pos giunto %d: %f' % (i,msg.position[i]))
            
            self.actual_position.append(round(msg.position[i],2))

        #print(self.actual_position)
        if not self.calcolatore.compare_positions(self.posizione_inviata,self.actual_position):
           #print(self.posizione_inviata)
           #print(self.actual_position)    
           self.raggiunto = True #Variabile per capire quando robot ha raggiunto posizione desiderata
        #self.get_logger().info(f"Posizione raggiunta: {self.position_reached}")

    def next_position_callback(self, msg):
        # Callback per ottenere la posizione del pezzo da raggiungere
        self.next_position = msg.data
        self.posizione_pezzo = []
        if (len(self.next_position)!=0) and self.take_photo:
          #print(self.next_position[0])
          #print(self.next_position[1])
          self.posizione_pezzo.append(self.next_position[0])
          self.posizione_pezzo.append(self.next_position[1])
          self.colore = self.next_position[2]
          #self.get_logger().info(f"Nuova posizione ricevuta: {self.next_position[0]}")

    def move_to_position(self, position):
        # Pubblica la posizione desiderata sul topic apposito facendo muovere il robot
        pos_msg = Float64MultiArray()
        self.posizione_inviata = []
        pos_msg.data=[position[1],position[2],position[3],position[4],position[5],position[6],position[7],position[8]]
        #print(pos_msg)
        self.posizione_inviata = [round(position[1],2),round(position[2],2),round(position[3],2),round(position[4],2),round(position[5],2),round(position[6],2),round(position[7],2),round(position[8],2)]
        self.position_pub.publish(pos_msg)
        #self.get_logger().info(f"Posizione inviata: {position}")
    
    #Comportamenti per ciascuno stato
    def stato1(self):
        #print("camera:")
        self.calcolatore.apri_pinza()
        
        print(self.posizione_pezzo)
        if (len(self.posizione_pezzo)!=0):
         #print(round(self.posizione_pezzo[0],1))
         #print(round(self.posizione_pezzo[1],1))
         pos2 = self.calcolatore.get_angoli(round(self.posizione_pezzo[0],1),round(self.posizione_pezzo[1],1),controlNode.Z_APPROCH)
         self.posizione_pezzo_x = round(self.posizione_pezzo[0],1)
         self.posizione_pezzo_y = round(self.posizione_pezzo[1],1)
         #print(pos2)
         self.move_to_position(pos2)
         self.raggiunto = False 
         self.stato = 1
         self.take_photo = False

    def stato2(self):
        pos3 = self.calcolatore.get_angoli(self.posizione_pezzo_x,self.posizione_pezzo_y,controlNode.Z_RECUPERO)
        self.move_to_position(pos3)    
        self.raggiunto = False
        self.stato = 2
        time.sleep(2)

    def stato3(self):  
        self.calcolatore.chiudi_pinza()
        self.pos4 = self.calcolatore.get_angoli(self.posizione_pezzo_x,self.posizione_pezzo_y,controlNode.Z_RECUPERO)
        self.pos4[6] = 0.1
        print(self.pos4)
        self.move_to_position(self.pos4)    
        self.raggiunto = False
        self.stato = 3
        time.sleep(5)

    def stato4(self):
        print(self.pos4)
        if self.colore == 1.0:
         self.pos4[1] = 6.28#-2.27
        if self.colore == 2.0:
         self.pos4[1] = -4.70 
        self.pos4[3] = -1.4
        self.move_to_position(self.pos4)  
        self.raggiunto = False
        self.stato = 4
        time.sleep(6)
        print(self.posizione_inviata)
        print(self.actual_position)

    def stato5(self):
        print(self.pos4) 
        self.pos4[7] = 1.0
        self.pos4[8] = -1.0
        self.move_to_position(self.pos4)  
        self.raggiunto = False
        time.sleep(3)  
        self.stato = 5
        





    def run(self):
        # Loop principale
        rate = self.create_rate(10)  # Frequenza del ciclo in Hz
        self.pos1= self.calcolatore.get_angoli(controlNode.X_PHOTO,controlNode.Y_PHOTO,controlNode.Z_PHOTO)
        #self.pos1.append(0.0)
        #self.pos1.append(0.0)
        self.pos1[4]=0.0
        
        #Raggiungo posizione per raccogliere foto
        self.move_to_position(self.pos1)        

        while rclpy.ok():

            # Implementazione macchina a stati

            if self.raggiunto and self.stato == 0:
                
                self.take_photo = True #Variabile per ottenere la foto solo quando il robot sarà nella posizione corretta
                self.stato1()
                
                print("raggiunto stato1") 

            if self.stato == 0 and not self.raggiunto:
                self.move_to_position(self.pos1)
                

                
            if self.raggiunto and self.stato == 1:
                
                self.stato2()
                print("raggiunto stato2")   

            if self.stato == 2:
                
                self.stato3()
                print("raggiunto stato3")   

            if  self.stato == 3:
                
                self.stato4()
                print("raggiunto stato4")   

            if self.raggiunto and self.stato == 4:
                
                self.stato5()
                print("raggiunto stato5")  

            if self.stato == 5:
                self.move_to_position(self.pos1)
                self.stato = 0
                print("finito")  

            rclpy.spin_once(self, timeout_sec=0.1)



def main(args=None):
    rclpy.init(args=args)

    # Crea l'oggetto controller e avvia il ciclo principale
    robot_controller = controlNode()
    robot_controller.run()
    rclpy.spin(robot_controller)
    #robot_controller.move_to_position(1)

    # Shutdown di ROS 2
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
