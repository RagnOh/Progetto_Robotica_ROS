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
    Z_APPROCH = 0.15
    Z_RECUPERO = 0.1


    def __init__(self):
        super().__init__('control_node')

        self.calcolatore = MoverController()
        
        # Publisher per inviare la posizione desiderata
        self.position_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        
        # Subscriber per confermare quando il robot ha raggiunto la posizione desiderata
        self.create_subscription(JointState, '/joint_states', self.position_reached_callback, 10)
        
        # Subscriber per ricevere la nuova posizione
        self.create_subscription(Float32MultiArray, '/point_coordinates', self.next_position_callback, 10)
        
        # Variabili per memorizzare lo stato e la prossima posizione
        self.position_reached = False
        self.next_position = None

    def position_reached_callback(self, msg):
        # Callback per quando il robot ha raggiunto la posizione desiderata
        #self.position_reached = msg.data
        #print(self.position_reached)
        print("r")
        for i in range(len(msg.position)):
            self.get_logger().info('pos giunto %d: %f' % (i,msg.position[i]))
        #self.get_logger().info(f"Posizione raggiunta: {self.position_reached}")

    def next_position_callback(self, msg):
        # Callback per ottenere la prossima posizione
        self.next_position = msg.data
        self.get_logger().info(f"Nuova posizione ricevuta: {self.next_position[0]}")

    def move_to_position(self, position):
        # Pubblica la posizione desiderata
        pos_msg = Float64MultiArray()
        pos_msg.data=[position[1],position[2],position[3],position[4],position[5],position[6],position[7],position[8]]
        print(pos_msg)
        self.position_pub.publish(pos_msg)
        self.get_logger().info(f"Posizione inviata: {position}")

    def run(self):
        # Loop principale
        rate = self.create_rate(10)  # Frequenza del ciclo in Hz
        pos1= self.calcolatore.calcolo_angolo_giunti(controlNode.X_PHOTO,controlNode.Y_PHOTO,controlNode.Z_PHOTO)
        pos1.append(0.0)
        pos1.append(0.0)
        pos1[4]=0.0
        
        #Raggiungo posizione foto
        #self.move_to_position(pos1)

        #Controllo se posizione raggiunta

        #while rclpy.ok():
            #if self.position_reached and self.next_position:
                # Se il robot ha raggiunto la posizione e c'è una nuova posizione, muovi il robot
              #  self.move_to_position(self.next_position)
                
                # Reset delle variabili
               ## self.position_reached = False
               # self.next_position = None

        #rclpy.spin_once(self, timeout_sec=0.1)

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
