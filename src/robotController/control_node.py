import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2
from sympy.matrices import Matrix

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time
import numpy as np

import ikpy.chain
import ikpy.utils.plot as plot_utils

import numpy as np
import time
import math




class controlNode(Node):

    l1=0.28
    l2=0.28
    l3=0.08
    l4=0.1
    theta1=0.0
    theta2=0.0
    theta3=0.0
    theta4=0.0
    theta5=0.0
    theta6=0.0

    def __init__(self):
        super().__init__('control_node')
        self.publisher = self.create_publisher(Float64MultiArray,"/forward_position_controller/commands",10)
        self.timer = self.create_timer(2.0,self.timer_callback)

    def timer_callback(self):

        msg = Float64MultiArray()
        my_chain = ikpy.chain.Chain.from_urdf_file("src/robotController/robotController/arm.urdf",active_links_mask=[False, True, True, True, True, True, True])
        x=0.1
        y=0.1
        z=0.01
        target_position = [ x,y ,z]

        

        target_orientation = [0, 0, 0]
        ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")
        for angle in enumerate(ik):
            print(angle)
        print("The angles of each joints are : ", list(map(lambda r:math.degrees(r),ik.tolist())))
        
        q1=ik[1]
        q2=ik[2]
        q3=ik[3]
        q4=ik[4]
        q5=ik[5]
        q6=ik[6]
        
        print(q1)
        print(q2)
        print(q3)
        print(q4)
        print(q5)
        print(q6)
       

        #q11=round(q1,1)
        #q21=round(q2,1)
        #q31=round(q3,1)
        #q41=round(q4,1)
        #q51=round(q5,1)
        #q61=round(q6,1)

        #print(q11)
        
        
        msg.data =[q1,q2,q3,q4,q5,q6,0.0,0.0]
        self.publisher.publish(msg)
        
        
    
    def pointToAngoli(self,x,y,z):

        self.theta1=np.arctan2(x,y)
        #Distanza punto di base a piano xy
        r=np.sqrt(x**2+y**2)

        calc=(z-self.l3)/self.l2

        if calc>1 or calc <-1:
            calc=1

        #print(calc)
        #theta2
        alpha= np.arccos(calc)
        beta=np.arctan2(r,z-self.l3)

        if r==0:
            beta=0

        #print(alpha)    


        self.theta2 = alpha -beta - np.pi/2

        calc= (r**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2)
        if calc>1 or calc <-1:
            calc=1

        #theta3

        gamma = np.arccos(calc)

        if gamma > np.pi or gamma<-np.pi:
            gamma=np.pi
        self.theta3 = gamma - self.theta2

        #theta4
        self.theta4= np.arctan2(z - self.l3,r)

        self.theta5 = 0

        self.theta6 = np.arctan2(y,x)-self.theta1

        

        self.theta1 = np.radians(self.theta1)
        self.theta2 = np.radians(self.theta2)
        self.theta3 = np.radians(self.theta3)
        self.theta4 = np.radians(self.theta4)
        self.theta5 = np.radians(self.theta5)
        self.theta6 = np.radians(self.theta6)

def main():
    rclpy.init()
    node = controlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()            
