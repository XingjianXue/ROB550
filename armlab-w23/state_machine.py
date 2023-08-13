"""!
The state machine that implements the logic.
"""
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
import time
import numpy as np
import rospy
import copy
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel,
                         QMainWindow, QCursor, QFileDialog)
from math import pi
import kinematics



class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner
        @param      camera   The camera
        """
        
        self.rxarm = rxarm
        self.camera = camera
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoints = [
            [-np.pi/2,       -0.5,      -0.3,            0.0,       0.0],
            [0.75*-np.pi/2,   0.5,      0.3,      0.0,       np.pi/2],
            [0.5*-np.pi/2,   -0.5,     -0.3,     np.pi / 2,     0.0],
            [0.25*-np.pi/2,   0.5,     0.3,     0.0,       np.pi/2],
            [0.0,             0.0,      0.0,         0.0,     0.0],
            [0.25*np.pi/2,   -0.5,      -0.3,      0.0,       np.pi/2],
            [0.5*np.pi/2,     0.5,     0.3,     np.pi / 2,     0.0],
            [0.75*np.pi/2,   -0.5,     -0.3,     0.0,       np.pi/2],
            [np.pi/2,         0.5,     0.3,      0.0,     0.0],
            [0.0,             0.0,     0.0,      0.0,     0.0]]

        self.teach_point = []
        self.stop_teaching_flag = False
        self.grab_state = False
        self.calibrate_ = False
        self.K = np.array([[904.5715942382812, 0.0, 635.9815063476562],[0.0, 905.2954711914062, 353.06036376953125], [0.0,0.0,1.0]],dtype=np.float64)

        self.H = np.array([[1,0,0,-30],[0,-1,-0.2,340],[0,0.2,-1,995] ,[0,0,0,1]],dtype=np.float64)
    

    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and funcitons as needed.
        """
        

        if self.next_state == "initialize_rxarm":
            self.initialize_rxarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate": 
            self.calibrate()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "manual":
            self.manual()


        if self.next_state =="start_teaching":
            self.start_teaching()

        if self.next_state =="stop_teaching":
            self.stop_teaching()

        if self.next_state == "record_step":
            self.record_step()

        if self.next_state == "play_back":
            self.play_back()

	    #Pick and Grab Checkpoint 3
        if self.next_state == "Pick_and_Grab":
            self.Pick_and_Grab()

        if self.next_state == "detect":
            self.detect()

        #Competition States
        if self.next_state == "event1":
            self.event1()
        if self.next_state == "event2":
            self.event2()
        if self.next_state == "event3":
            self.event3()
        if self.next_state == "event4":
            self.event4()
        if self.next_state == "event5":
            self.event5()

    """Functions run for each state"""

    #teach and repeat
    def record_step(self):
        if (self.stop_teaching_flag == False):
            self.current_state = "record_step"
            self.teach_point.append((self.rxarm.get_positions(),self.rxarm.gripper_state))
            self.next_state = "idle"


    def play_back(self):
        self.current_status = "play_back"
        self.rxarm.enable_torque()
        print("The Joint angles are...")
        print(self.teach_point)
        k=0
        for k in range(10):
            k=k+1
            for teach_point in self.teach_point:
                self.rxarm.set_positions(teach_point[0])
                if teach_point[1] == True:
                    self.rxarm.open_gripper()
                else:
                    self.rxarm.close_gripper()
                if self.current_state == "estop":
                    break
                rospy.sleep(1)

        self.next_state = "idle"

    def start_teaching(self):
        self.stop_teaching_flag = False
        self.teach_point = []
        self.current_state = "start_teaching"
        self.next_state = "idle"
        self.rxarm.disable_torque()

    def stop_teaching(self):
        self.current_state = "stop_teaching"
        self.next_state = "idle"
        self.stop_teaching_flag = True


    def manual(self):
        """!
        @brief      Manually control the rxarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        

        

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        self.status_message = "State: Execute - Executing motion plan"
        for j in range(len(self.waypoints)):
            self.rxarm.set_positions(self.waypoints[j])
            rospy.sleep(1)

        self.next_state = "idle"

        
    def calibrate(self):      
        """!
        @brief      Gets the user input to perform the calibration
        """
        detections_xyz_camera = np.zeros((4,4))
        temp = np.zeros((4,1))

        detections_xyz_world = np.array([[-250,-25, self.camera.DepthFrameRaw[-25][-250]],[250,-25,self.camera.DepthFrameRaw[-25][250]],[250,275,self.camera.DepthFrameRaw[275][250]],[-250,275,self.camera.DepthFrameRaw[275][-250]]])
        #detections_xyz_world = np.array([[-250,-25,0],[250,-25,0],[250,275,0],[-250,275,0]])

        msg = self.camera.tag_detections
        for detection in msg.detections:
            temp = np.array([1000*detection.pose.pose.pose.position.x,1000*detection.pose.pose.pose.position.y,1000*detection.pose.pose.pose.position.z,1])
            if temp[0] < 0 and temp[1] > 0:
               detections_xyz_camera[0] = temp 
            if temp[0] > 0 and temp[1] > 0:
               detections_xyz_camera[1] = temp
            if temp[0] > 0 and temp[1] < 0:
               detections_xyz_camera[2] = temp
            if temp[0] < 0 and temp[1] < 0:
               detections_xyz_camera[3] = temp       
        Xw = np.array([detections_xyz_world[0,0],detections_xyz_world[1,0],detections_xyz_world[2,0],detections_xyz_world[3,0]])
        Yw = np.array([detections_xyz_world[0,1],detections_xyz_world[1,1],detections_xyz_world[2,1],detections_xyz_world[3,1]])
        Zw = np.array([detections_xyz_world[0,2],detections_xyz_world[1,2],detections_xyz_world[2,2],detections_xyz_world[3,2]])
        H1 = np.matmul(np.linalg.inv(detections_xyz_camera),np.transpose(Xw))
        H2 = np.matmul(np.linalg.inv(detections_xyz_camera),np.transpose(Yw))
        H3 = np.matmul(np.linalg.inv(detections_xyz_camera),np.transpose(Zw))
        H4 = np.array([0,0,0,1])
        
        self.calibrate_ = True
        H = np.array([H1,H2,H3,H4])

        self.H = H 

        self.next_state = "idle"

        return H

    def event1(self):
        """Pick and sort: We will place blocks (cubes) in front of the arm in the positive half-plane (in front of the arm)
        Level 1 - 3 large blocks (R,G and B), not stacked.
        Level 2 - 6 blocks, 3 of each size, random colors (ROYGBV), not stacked.
        Level 3 - 9 blocks, random sizes, random colors (ROYGBV), possibly stacked
        
        Notes: 
        - Choose one level.
        -The team will have 180s to complete the task.
        -Drop small blocks to the left and big ones to the right of the arm in the negative half-plane.
        +30 pts for each correct block, +(10*level) points for completing the task in the time allotted, 50% deduction for using to click and place."""

        #Computer Vision processing
        self.status_message = "State: event 1"  
        aLim = 20
        sum_s = 1
        sum_l = 1    
        while True: 
            x = 0
            y = 0
            z = 0
            k = 0
            r = 0
            area_ = 0
            for i in range(5):
                c, blockCM, blockRot, isSmall, area = self.camera.detectBlocksInDepthImage() 
                blockDat = []
                
                for i in range(len(blockRot)):
                    if area[i] > aLim:
                        tempBlockDat = [blockCM[i], blockRot[i], area[i]]   
                        blockDat.append(tempBlockDat) 
                if len(blockDat) == 0: 
                    break
                
                for j in range(len(blockDat)):
                    for i in range(len(blockDat) - 1):
                        if blockDat[i][0][0] < blockDat[i+1][0][0]:
                            temp = blockDat[i]
                            blockDat[i] = blockDat[i+1]
                            blockDat[i+1] = temp
            
                nextb_L = blockDat[0]
                lBx = nextb_L[0][0] 
                lBy = nextb_L[0][1]
                lBz = nextb_L[0][2]
                
                lBRot = int(nextb_L[1])*pi/180 
                
                x = lBx + x
                y = lBy + y
                z = lBz + z
                r = lBRot + r
                area_ = area_ + nextb_L[2]
                k = k + 1
                               
            if k == 0:
                return 0
            
            u = x/k
            v = y/k
            d = z/k
            r = r/k
            area_average = area_/k

            if area_average > 700:
                print("Large")

                K = np.array([[904.5715942382812, 0.0, 635.9815063476562],[0.0, 905.2954711914062, 353.06036376953125], [0.0,               0.0,               1.0               ]],dtype=np.float64)
                pixel = np.array([[u],[v], [1]],dtype=np.float64)
                camera_coordinate = d*np.matmul(np.linalg.inv(K),pixel)
                camera_coordinate_homogenuous = np.concatenate([camera_coordinate,np.ones([1,1])])
                translation_matrix = np.array([[1,0,0,-30],[0,1,0,340],[0,0,1,995],[0,0,0,1]],dtype=np.float64)
                angle1 = np.pi - 0.2 
                rotation_matrix_x = np.array([[1,0 ,0,0  ],[0,np.cos(angle1),-np.sin(angle1),0  ], [0,np.sin(angle1)   ,np.cos(angle1)    ,0  ],[0,0                ,0                 ,1  ]],dtype=np.float64)
                H = np.matmul(translation_matrix,rotation_matrix_x)
                world_coordinate = np.matmul(H,camera_coordinate_homogenuous)
                wx = int(world_coordinate[0,0])
                wy = int(world_coordinate[1,0])
                wz = int(world_coordinate[2,0])

               
    

                self.rxarm.open_gripper()
                temp = [wx, wy,wz+150,-np.pi/2,0]
                twist = [wx, wy,wz+150,-np.pi/2,0]
                blockPose = [wx, wy, wz+10,-np.pi/2,0]
                dest_temp = [200,-100,wz+150,-np.pi/2,0]
                dest = [200,-100,36*sum_l + 10 ,-np.pi/2,0]

                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)

                angle_1,angle_2 = kinematics.IK_geometric(blockPose)
                self.rxarm.set_positions(angle_2)
                time.sleep(3)
                self.rxarm.close_gripper()
                
            

                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)



                angle_1,angle_2 = kinematics.IK_geometric(dest_temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)

                angle_1,angle_2 = kinematics.IK_geometric(dest) 
                self.rxarm.set_positions(angle_2)
                time.sleep(2)
                self.rxarm.open_gripper()

                angle_1,angle_2 = kinematics.IK_geometric(dest_temp) 
                self.rxarm.set_positions(angle_2)
                time.sleep(1)
                sum_l = sum_l + 1


            if area_average <700:
                print("Small")

                K = np.array([[904.5715942382812, 0.0, 635.9815063476562],[0.0, 905.2954711914062, 353.06036376953125], [0.0,               0.0,               1.0               ]],dtype=np.float64)
                pixel = np.array([[u],[v], [1]],dtype=np.float64)
                camera_coordinate = d*np.matmul(np.linalg.inv(K),pixel)
                camera_coordinate_homogenuous = np.concatenate([camera_coordinate,np.ones([1,1])])
                translation_matrix = np.array([[1,0,0,-30],[0,1,0,340],[0,0,1,995],[0,0,0,1]],dtype=np.float64)
                angle1 = np.pi - 0.2 
                rotation_matrix_x = np.array([[1,0 ,0,0  ],[0,np.cos(angle1),-np.sin(angle1),0  ], [0,np.sin(angle1)   ,np.cos(angle1)    ,0  ],[0,0                ,0                 ,1  ]],dtype=np.float64)
                H = np.matmul(translation_matrix,rotation_matrix_x)
                world_coordinate = np.matmul(self.H,camera_coordinate_homogenuous)
                wx = int(world_coordinate[0,0])
                wy = int(world_coordinate[1,0])
                wz = int(world_coordinate[2,0])
    

                self.rxarm.open_gripper()
                temp = [wx, wy,wz+150,-np.pi/2,0]
                twist = [wx, wy,wz+150,-np.pi/2,0]
                blockPose = [wx, wy, wz+10,-np.pi/2,0]
                dest_temp = [-200,-100,wz+150,-np.pi/2,0]
                dest = [-200,-100,25*sum_s + 10,-np.pi/2,0]

                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)



                angle_1,angle_2 = kinematics.IK_geometric(blockPose)
                self.rxarm.set_positions(angle_2)
                time.sleep(3)
                self.rxarm.close_gripper()
                
            

                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)



                angle_1,angle_2 = kinematics.IK_geometric(dest_temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)

                angle_1,angle_2 = kinematics.IK_geometric(dest) 
                self.rxarm.set_positions(angle_2)
                time.sleep(2)
                self.rxarm.open_gripper()

                angle_1,angle_2 = kinematics.IK_geometric(dest_temp) 
                self.rxarm.set_positions(angle_2)
                time.sleep(1)
 
                sum_s = sum_s + 1             

            self.next_state = "idle"



    
    def event2(self):
        """Pick and stack: We will place blocks (cubes) in front of the arm in the positive half-plane (in front of the arm)
        Level 1 - 3 large blocks (R,G and B), not stacked.
        Level 2 - 6 blocks, 3 of each size, random colors (ROYGBV), not stacked.
        Level 3 - 9 blocks, random sizes, random colors (ROYGBV), possibly stacked two high
        
        Notes: 
        - Choose one level.
        -The team will have 360s to complete the task.
        -Stack all blocks three tall to the left or right of the arm in the negative half-plane.
        +30 pts for each correct block, +(10*level) points for completing the task in the time allotted, 50% deduction for using to click and place."""
        self.status_message = "State: event 2"  
        aLim = 20
        sum_s = 1
        sum_l = 1    
        while True: 
            x = 0
            y = 0
            z = 0
            k = 0
            r = 0
            area_ = 0
            for i in range(5):
                c, blockCM, blockRot, isSmall, area = self.camera.detectBlocksInDepthImage() 

                blockDat = []

                
                for i in range(len(blockRot)):
                    if area[i] > aLim:
                        tempBlockDat = [blockCM[i], blockRot[i], area[i]]   
                        blockDat.append(tempBlockDat) 
                if len(blockDat) == 0: 
                    break
                
                for j in range(len(blockDat)):
                    for i in range(len(blockDat) - 1):
                        if blockDat[i][0][0] < blockDat[i+1][0][0]:
                            temp = blockDat[i]
                            blockDat[i] = blockDat[i+1]
                            blockDat[i+1] = temp
            
                nextb_L = blockDat[0]
                lBx = nextb_L[0][0] 
                lBy = nextb_L[0][1]
                lBz = nextb_L[0][2]
                
                lBRot = int(nextb_L[1])
                lBRot = lBRot*pi/180
                 
                
                x = lBx + x
                y = lBy + y
                z = lBz + z
                r = lBRot + r
                area_ = area_ + nextb_L[2]
                k = k + 1
                               
            if k == 0:
                return 0
            
            u = x/k
            v = y/k
            d = z/k
            r = r/k
            area_average = area_/k


            if area_average > 750:
                print("Large")


                K = np.array([[904.5715942382812, 0.0, 635.9815063476562],[0.0, 905.2954711914062, 353.06036376953125], [0.0,               0.0,               1.0               ]],dtype=np.float64)
                pixel = np.array([[u],[v], [1]],dtype=np.float64)
                camera_coordinate = d*np.matmul(np.linalg.inv(K),pixel)
                camera_coordinate_homogenuous = np.concatenate([camera_coordinate,np.ones([1,1])])
                translation_matrix = np.array([[1,0,0,-30],[0,1,0,340],[0,0,1,995],[0,0,0,1]],dtype=np.float64)
                angle1 = np.pi - 0.2 
                rotation_matrix_x = np.array([[1,0 ,0,0  ],[0,np.cos(angle1),-np.sin(angle1),0  ], [0,np.sin(angle1)   ,np.cos(angle1)    ,0  ],[0,0                ,0                 ,1  ]],dtype=np.float64)
                H = np.matmul(translation_matrix,rotation_matrix_x)
                world_coordinate = np.matmul(self.H,camera_coordinate_homogenuous)
                wx = int(world_coordinate[0,0])
                wy = int(world_coordinate[1,0])
                wz = int(world_coordinate[2,0])
    

                self.rxarm.open_gripper()
                temp = [wx, wy,wz+150,-np.pi/2,0]
                twist = [wx, wy,wz+150,-np.pi/2,0]
                blockPose = [wx, wy, wz+10,-np.pi/2,0]
                dest_temp = [200,-100,wz+150,-np.pi/2,0]
                dest = [200,-100,36*sum_l ,-np.pi/2,0]

                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)

   
                angle_1,angle_2 = kinematics.IK_geometric(blockPose)
                self.rxarm.set_positions(angle_2)
                time.sleep(2.5)
                self.rxarm.close_gripper()
                

                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(1)

                angle_1,angle_2 = kinematics.IK_geometric(dest_temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)

                angle_1,angle_2 = kinematics.IK_geometric(dest) 
                self.rxarm.set_positions(angle_2)
                time.sleep(3)
                self.rxarm.open_gripper()

                angle_1,angle_2 = kinematics.IK_geometric(dest_temp) 
                self.rxarm.set_positions(angle_2)
                time.sleep(1)
                sum_l = sum_l + 1


            if area_average <750:
                print("Small")


                K = np.array([[904.5715942382812, 0.0, 635.9815063476562],[0.0, 905.2954711914062, 353.06036376953125], [0.0,               0.0,               1.0               ]],dtype=np.float64)
                pixel = np.array([[u],[v], [1]],dtype=np.float64)
                camera_coordinate = d*np.matmul(np.linalg.inv(K),pixel)
                camera_coordinate_homogenuous = np.concatenate([camera_coordinate,np.ones([1,1])])
                translation_matrix = np.array([[1,0,0,-30],[0,1,0,340],[0,0,1,995],[0,0,0,1]],dtype=np.float64)
                angle1 = np.pi - 0.2 
                rotation_matrix_x = np.array([[1,0 ,0,0  ],[0,np.cos(angle1),-np.sin(angle1),0  ], [0,np.sin(angle1)   ,np.cos(angle1)    ,0  ],[0,0                ,0                 ,1  ]],dtype=np.float64)
                H = np.matmul(translation_matrix,rotation_matrix_x)
                world_coordinate = np.matmul(self.H,camera_coordinate_homogenuous)
                wx = int(world_coordinate[0,0])
                wy = int(world_coordinate[1,0])
                wz = int(world_coordinate[2,0])
    

                self.rxarm.open_gripper()
                temp = [wx, wy,wz+150,-np.pi/2,0]
                twist = [wx, wy,wz+150,-np.pi/2,0]
                blockPose = [wx, wy, wz+10,-np.pi/2,0]
                dest_temp = [-200,-100,wz+150,-np.pi/2,0]
                dest = [-200,-100,25*sum_s ,-np.pi/2,0]

                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)

                angle_1,angle_2 = kinematics.IK_geometric(blockPose)
                self.rxarm.set_positions(angle_2)
                time.sleep(2.5)
                self.rxarm.close_gripper()
                
                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(1)



                angle_1,angle_2 = kinematics.IK_geometric(dest_temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)

                angle_1,angle_2 = kinematics.IK_geometric(dest) 
                self.rxarm.set_positions(angle_2)
                time.sleep(3)
                self.rxarm.open_gripper()

                angle_1,angle_2 = kinematics.IK_geometric(dest_temp) 
                self.rxarm.set_positions(angle_2)
                time.sleep(1)
 
                sum_s = sum_s + 1             

            self.next_state = "idle"
        


    def event3(self):
        """Line em up:"""

        #Computer Vision processing
        self.status_message = "State: event 3"  
        aLim = 20
        sum_s = 1
        sum_l = 1  


        RL = False
        OL = False
        YL = False
        GL = False
        BL = False
        VL = False

        while True: 
            x = 0
            y = 0
            z = 0
            k = 0
            r = 0
            area_ = 0
            for i in range(10):
                c, blockCM, blockRot, isSmall, area = self.camera.detectBlocksInDepthImage() 
                print(c)
                blockDat = []



                
                for i in range(len(blockRot)):
                    if area[i] > aLim:
                        tempBlockDat = [blockCM[i], blockRot[i], area[i],c[i]]    
                        blockDat.append(tempBlockDat) 
                if len(blockDat) == 0:   
                    break
                
                for j in range(len(blockDat)):
                    for i in range(len(blockDat) - 1):
                        if blockDat[i][0][0] < blockDat[i+1][0][0]:
                            temp = blockDat[i]
                            blockDat[i] = blockDat[i+1]
                            blockDat[i+1] = temp
            
                nextb_L = blockDat[0]
                lBx = nextb_L[0][0] 
                lBy = nextb_L[0][1]
                lBz = nextb_L[0][2]

                

                lBRot = int(nextb_L[1])
                lBRot = lBRot*pi/180
                 
                
                x = lBx + x
                y = lBy + y
                z = lBz + z
                r = lBRot + r
                area_ = area_ + nextb_L[2]
                k = k + 1
                               
            if k == 0:
                return 0
            
            u = x/k
            v = y/k
            d = z/k
            r = r/k
            area_average = area_/k

            print(area_average)

            if area_average > 700:
                print("Large")


                K = np.array([[904.5715942382812, 0.0, 635.9815063476562],[0.0, 905.2954711914062, 353.06036376953125], [0.0,               0.0,               1.0               ]],dtype=np.float64)
                pixel = np.array([[u],[v], [1]],dtype=np.float64)
                camera_coordinate = d*np.matmul(np.linalg.inv(K),pixel)
                camera_coordinate_homogenuous = np.concatenate([camera_coordinate,np.ones([1,1])])
                translation_matrix = np.array([[1,0,0,-30],[0,1,0,340],[0,0,1,995],[0,0,0,1]],dtype=np.float64)
                angle1 = np.pi - 0.2 
                rotation_matrix_x = np.array([[1,0 ,0,0  ],[0,np.cos(angle1),-np.sin(angle1),0  ], [0,np.sin(angle1)   ,np.cos(angle1)    ,0  ],[0,0                ,0                 ,1  ]],dtype=np.float64)
                H = np.matmul(translation_matrix,rotation_matrix_x)
                world_coordinate = np.matmul(H,camera_coordinate_homogenuous)
                wx = int(world_coordinate[0,0])
                wy = int(world_coordinate[1,0])
                wz = int(world_coordinate[2,0])
    

                self.rxarm.open_gripper()
                temp = [wx, wy,wz+150,-np.pi/2,0]
                twist = [wx, wy,wz+150,-np.pi/2,0]
                blockPose = [wx, wy, wz+10,-np.pi/2,0]


                #Color logic
                if nextb_L[3][0] == 'red':
                    dest_temp = [200,-100,wz+150,-np.pi/2,0]
                    dest = [200,-100,wz ,-np.pi/2,0]
                if nextb_L[3][0] == 'orange':
                    dest_temp = [200,-75,wz+150,-np.pi/2,0]
                    dest = [200,-85,wz ,-np.pi/2,0]    
                if nextb_L[3][0] == 'yellow':
                    dest_temp = [200,-50,wz+150,-np.pi/2,0]
                    dest = [200,-65,wz ,-np.pi/2,0]
                if nextb_L[3][0] == 'green':
                    dest_temp = [200,-25,wz+150,-np.pi/2,0]
                    dest = [200,-45,wz ,-np.pi/2,0]
                if nextb_L[3][0] == 'blue':
                    dest_temp = [200,-0,wz+150,-np.pi/2,0]
                    dest = [200,-25,wz ,-np.pi/2,0]
                if nextb_L[3][0] == 'violet':
                    dest_temp = [200,25,wz+150,-np.pi/2,0]
                    dest = [200,-5,wz ,-np.pi/2,0]



                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(3)

                angle_1,angle_2 = kinematics.IK_geometric(twist)
                self.rxarm.set_positions(angle_2)
                time.sleep(3)



                angle_1,angle_2 = kinematics.IK_geometric(blockPose)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)
                self.rxarm.close_gripper()
                
            

                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)



                angle_1,angle_2 = kinematics.IK_geometric(dest_temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)

                angle_1,angle_2 = kinematics.IK_geometric(dest) 
                self.rxarm.set_positions(angle_2)
                time.sleep(2)
                self.rxarm.open_gripper()

                angle_1,angle_2 = kinematics.IK_geometric(dest_temp) 
                self.rxarm.set_positions(angle_2)
                time.sleep(2)
                sum_l = sum_l + 1


            if area_average <700:
                print("Small")


                K = np.array([[904.5715942382812, 0.0, 635.9815063476562],[0.0, 905.2954711914062, 353.06036376953125], [0.0,               0.0,               1.0               ]],dtype=np.float64)
                pixel = np.array([[u],[v], [1]],dtype=np.float64)
                camera_coordinate = d*np.matmul(np.linalg.inv(K),pixel)
                camera_coordinate_homogenuous = np.concatenate([camera_coordinate,np.ones([1,1])])
                translation_matrix = np.array([[1,0,0,-30],[0,1,0,340],[0,0,1,995],[0,0,0,1]],dtype=np.float64)
                angle1 = np.pi - 0.2 
                rotation_matrix_x = np.array([[1,0 ,0,0  ],[0,np.cos(angle1),-np.sin(angle1),0  ], [0,np.sin(angle1)   ,np.cos(angle1)    ,0  ],[0,0                ,0                 ,1  ]],dtype=np.float64)
                H = np.matmul(translation_matrix,rotation_matrix_x)
                world_coordinate = np.matmul(H,camera_coordinate_homogenuous)
                wx = int(world_coordinate[0,0])
                wy = int(world_coordinate[1,0])
                wz = int(world_coordinate[2,0])
    

                self.rxarm.open_gripper()
                temp = [wx, wy,wz+150,-np.pi/2,0]
                twist = [wx, wy,wz+150,-np.pi/2,0]
                blockPose = [wx, wy, wz+10,-np.pi/2,0]
                #Color logic

                if nextb_L[3][0] == 'red':
                    dest_temp = [-200,-100,wz+150,-np.pi/2,0]
                    dest = [-200,-100,wz ,-np.pi/2,0]
                if nextb_L[3][0] == 'orange':
                    dest_temp = [-200,-75,wz+150,-np.pi/2,0]
                    dest = [-200,-85,wz ,-np.pi/2,0]    
                if nextb_L[3][0] == 'yellow':
                    dest_temp = [-200,-50,wz+150,-np.pi/2,0]
                    dest = [-200,-65,wz ,-np.pi/2,0]
                if nextb_L[3][0] == 'green':
                    dest_temp = [-200,-25,wz+150,-np.pi/2,0]
                    dest = [-200,-45,wz ,-np.pi/2,0]
                if nextb_L[3][0] == 'blue':
                    dest_temp = [-200,0,wz+150,-np.pi/2,0]
                    dest = [-200,-25,wz ,-np.pi/2,0]
                if nextb_L[3][0] == 'violet':
                    dest_temp = [-200,25,wz+150,-np.pi/2,0]
                    dest = [-200,-5,wz ,-np.pi/2,0]

                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(3)

                angle_1,angle_2 = kinematics.IK_geometric(twist)
                self.rxarm.set_positions(angle_2)
                time.sleep(3)



                angle_1,angle_2 = kinematics.IK_geometric(blockPose)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)
                self.rxarm.close_gripper()
                
            

                angle_1,angle_2 = kinematics.IK_geometric(temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)



                angle_1,angle_2 = kinematics.IK_geometric(dest_temp)
                self.rxarm.set_positions(angle_2)
                time.sleep(2)

                angle_1,angle_2 = kinematics.IK_geometric(dest) 
                self.rxarm.set_positions(angle_2)
                time.sleep(2)
                self.rxarm.open_gripper()

                angle_1,angle_2 = kinematics.IK_geometric(dest_temp) 
                self.rxarm.set_positions(angle_2)
                time.sleep(2)
 
                sum_s = sum_s + 1  
        
            self.next_state = "idle"

    

    def event4(self):
        """Stack em high"""
        #Level one only small blocks" stack in rainbow color order
        #Computer Vision processing
        self.status_message = "State: event 1"  
        aLim = 20
        sum_s = 1
        sum_l = 1    
        while True: 
            x = 0
            y = 0
            z = 0
            k = 0
            r = 0
            area_ = 0
            for i in range(5):
                c, blockCM, blockRot, isSmall, area = self.camera.detectBlocksInDepthImage() 
                blockDat = []
                

                redS = False
                orangeS = False
                yellowS = False
                greenS = False
                blueS = False
                violetS = False

                for i in range(len(blockRot)):
                    if area[i] > aLim:
                        if redS == False and c[i][0] =='red':
                            tempBlockDat = [blockCM[i], blockRot[i], area[i]]   
                            blockDat.append(tempBlockDat)
                            redS = True

                        if redS == True and orangeS == False and c[i][0]  =='orange':
                            tempBlockDat = [blockCM[i], blockRot[i], area[i]] 
                            blockDat.append(tempBlockDat)
                            orangeS = True

                        if redS == True and orangeS == True and yellowS == False and c[i][0]  =='yellow':
                            tempBlockDat = [blockCM[i], blockRot[i], area[i]]
                            blockDat.append(tempBlockDat)
                            yellowS = True

                        if redS == True and orangeS == True and yellowS == True and greenS == False and c[i][0]  =='green':
                            tempBlockDat = [blockCM[i], blockRot[i], area[i]] 
                            blockDat.append(tempBlockDat)
                            greenS = True

                        if redS == True and orangeS == True and yellowS == True and greenS == True and blueS == False and c[i][0]  =='blue':
                            tempBlockDat = [blockCM[i], blockRot[i], area[i]] 
                            blockDat.append(tempBlockDat)
                            blueS = True

                        if redS == True and orangeS == True and yellowS == True and greenS == True and blueS == True and violetS ==False and c[i][0]  =='violet':
                            tempBlockDat = [blockCM[i], blockRot[i], area[i]] 
                            blockDat.append(tempBlockDat)
                            violetS = True

                         
                if len(blockDat) == 0: 
                    break
                
                for j in range(len(blockDat)):
                    for i in range(len(blockDat) - 1):
                        if blockDat[i][0][0] < blockDat[i+1][0][0]:
                            temp = blockDat[i]
                            blockDat[i] = blockDat[i+1]
                            blockDat[i+1] = temp
            
                nextb_L = blockDat[0]
                lBx = nextb_L[0][0] 
                lBy = nextb_L[0][1]
                lBz = nextb_L[0][2]
                
                lBRot = int(nextb_L[1])*pi/180 
                
                x = lBx + x
                y = lBy + y
                z = lBz + z
                r = lBRot + r
                area_ = area_ + nextb_L[2]
                k = k + 1
                               
            if k == 0:
                return 0
            
            u = x/k
            v = y/k
            d = z/k
            r = r/k
            area_average = area_/k

            K = np.array([[904.5715942382812, 0.0, 635.9815063476562],[0.0, 905.2954711914062, 353.06036376953125], [0.0,               0.0,               1.0               ]],dtype=np.float64)
            pixel = np.array([[u],[v], [1]],dtype=np.float64)
            camera_coordinate = d*np.matmul(np.linalg.inv(K),pixel)
            camera_coordinate_homogenuous = np.concatenate([camera_coordinate,np.ones([1,1])])
            translation_matrix = np.array([[1,0,0,-30],[0,1,0,340],[0,0,1,995],[0,0,0,1]],dtype=np.float64)
            angle1 = np.pi - 0.2 
            rotation_matrix_x = np.array([[1,0 ,0,0  ],[0,np.cos(angle1),-np.sin(angle1),0  ], [0,np.sin(angle1)   ,np.cos(angle1)    ,0  ],[0,0                ,0                 ,1  ]],dtype=np.float64)
            H = np.matmul(translation_matrix,rotation_matrix_x)
            world_coordinate = np.matmul(self.H,camera_coordinate_homogenuous)
            wx = int(world_coordinate[0,0])
            wy = int(world_coordinate[1,0])
            wz = int(world_coordinate[2,0])
    

            self.rxarm.open_gripper()
            temp = [wx, wy,wz+150,-np.pi/2,0]
            twist = [wx, wy,wz+150,-np.pi/2,0]
            blockPose = [wx, wy, wz+10,-np.pi/2,0]
            dest_temp = [-200,-100,wz+150,-np.pi/2,0]
            dest = [-200,-100,25*sum_s + 10,-np.pi/2,0]

            angle_1,angle_2 = kinematics.IK_geometric(temp)
            self.rxarm.set_positions(angle_2)
            time.sleep(2)



            angle_1,angle_2 = kinematics.IK_geometric(blockPose)
            self.rxarm.set_positions(angle_2)
            time.sleep(3)
            self.rxarm.close_gripper()
                
            

            angle_1,angle_2 = kinematics.IK_geometric(temp)
            self.rxarm.set_positions(angle_2)
            time.sleep(2)



            angle_1,angle_2 = kinematics.IK_geometric(dest_temp)
            self.rxarm.set_positions(angle_2)
            time.sleep(2)

            angle_1,angle_2 = kinematics.IK_geometric(dest) 
            self.rxarm.set_positions(angle_2)
            time.sleep(2)
            self.rxarm.open_gripper()

            angle_1,angle_2 = kinematics.IK_geometric(dest_temp) 
            self.rxarm.set_positions(angle_2)
            time.sleep(1)
 
            sum_s = sum_s + 1             

            self.next_state = "idle"

      
    

    def event5(self):   
        """Bonus event: To the sky!"""
        self.event1()

        self.next_state = "idle"

       



        

    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        _, _, _, _, _ = self.camera.detectBlocksInDepthImage()
        #rospy.sleep(1)
        #self.next_state = "idle"

    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            rospy.sleep(5)
        self.next_state = "idle"


    def Pick_and_Grab(self):
        self.current_state = 'Pick_and_Grab'
        self.grab_state = True
        self.next_state = "idle"





class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)
    
    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            rospy.sleep(0.05)
