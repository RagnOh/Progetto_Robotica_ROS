import ikpy.chain
import ikpy.utils.plot as plot_utils

class MoverController:

    NOME_FILE = "./param4.urdf"

    def __init__(self):

        self.my_chain = ikpy.chain.Chain.from_urdf_file(MoverController.NOME_FILE,active_links_mask=[False,True,True,True,True,True,True])


    def calcolo_angolo_giunti(self,x,y,z):
       
       
       target_position = [x,y,z]
       target_orientation = [0,0,-1]

       ik = self.my_chain.inverse_kinematics(target_position, target_orientation,orientation_mode="Z")
       
       return ik.tolist()

    def calcolo_coord_cartesiane(self,vet):

        computed_position = my_chain.forward_kinematics(ik)

        return computed_position

    def compare_positions(self,desidered_pos,actual_pos):
        
        #print(len(desidered_pos)) 
        #print(actual_pos)
        #print(desidered_pos)
        #print(len(actual_pos))  
         
        if len(actual_pos)<8:
            print("l1")
            return True
        if len(desidered_pos)<8:
            print("l2")
            return True    
          

        if desidered_pos[2]!=actual_pos[0]:
            print(desidered_pos[0])
            print(actual_pos[2]) 
            return True

          
        if desidered_pos[0]!=actual_pos[1]:
            return True

        if desidered_pos[4]!=actual_pos[3]:
            return True

        if desidered_pos[1]!=actual_pos[6]:
            return True

        if desidered_pos[4]!=actual_pos[3]:
            return True           

        #if desidered_pos[5]!=actual_pos[5]:
        #    return True          
        
        return False   

