#!/usr/bin/python
"""!
Main GUI for Arm lab
"""
import os
script_path = os.path.dirname(os.path.realpath(__file__))
import copy
import argparse
import sys
import cv2
import numpy as np
from numpy import matmul
import rospy
import time
from functools import partial

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel,
                         QMainWindow, QCursor, QFileDialog)

from ui import Ui_MainWindow
from rxarm import RXArm, RXArmThread
from camera import Camera, VideoThread
from state_machine import StateMachine, StateMachineThread
from kinematics import IK_geometric
""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi


class Gui(QMainWindow):
    """!
    Main GUI Class

    Contains the main function and interfaces between the GUI and functions.
    """
    def __init__(self, parent=None, dh_config_file=None):
        QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        ######

        #Define mouse clicks for calibration
        #self.mousePos #Can also send this as an array I like single use
        self.calibrate_point = []
        self.calCount = 0
        self.clickMat = []
        self.lastVal = 0
        self.H = 0

        """ Groups of ui commonents """
        self.joint_readouts = [
            self.ui.rdoutBaseJC,
            self.ui.rdoutShoulderJC,
            self.ui.rdoutElbowJC,
            self.ui.rdoutWristAJC,
            self.ui.rdoutWristRJC,
        ]
        self.joint_slider_rdouts = [
            self.ui.rdoutBase,
            self.ui.rdoutShoulder,
            self.ui.rdoutElbow,
            self.ui.rdoutWristA,
            self.ui.rdoutWristR,
        ]
        self.joint_sliders = [
            self.ui.sldrBase,
            self.ui.sldrShoulder,
            self.ui.sldrElbow,
            self.ui.sldrWristA,
            self.ui.sldrWristR,
        ]
        """Objects Using Other Classes"""
        self.camera = Camera()
        print("Creating rx arm...")
        if (dh_config_file is not None):
            self.rxarm = RXArm(dh_config_file=dh_config_file)
        else:
            self.rxarm = RXArm()
        print("Done creating rx arm instance.")
        self.sm = StateMachine(self.rxarm, self.camera) 
        """
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
       
        self.ui.videoDisplay.setMouseTracking(True)
        self.ui.videoDisplay.mouseMoveEvent = self.trackMouse

    
            
        # Buttons
        nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state)
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btn_init_arm.clicked.connect(self.initRxarm)
        self.ui.btn_torq_off.clicked.connect(
            lambda: self.rxarm.disable_torque())
        self.ui.btn_torq_on.clicked.connect(lambda: self.rxarm.enable_torque())
        self.ui.btn_sleep_arm.clicked.connect(lambda: self.rxarm.sleep())
        ##################################################################################################
        #teach and repeat                                                                                
        def open_gripper_teach():                                                                       
            self.rxarm.enable_torque()                                                                   
            self.rxarm.open_gripper()                                                                    
            self.rxarm.gripper_state = True                                                                                                                           #
            self.rxarm.disable_torque()                                                                  
                                                                                                         
        def close_gripper_teach():                                                                       
            self.rxarm.enable_torque()                                                                   
            self.rxarm.close_gripper()                                                                   
            self.rxarm.gripper_state = False                                                             
            self.rxarm.disable_torque()                                                                 
        #User Buttons
        self.ui.btnUser1.setText("Calibrate")
        self.ui.btnUser1.clicked.connect(partial(nxt_if_arm_init, 'calibrate'))
        self.ui.btnUser2.setText('Open Gripper')
        self.ui.btnUser2.clicked.connect(lambda: self.rxarm.open_gripper())
        self.ui.btnUser3.setText('Close Gripper')
        self.ui.btnUser3.clicked.connect(lambda: self.rxarm.close_gripper())
        self.ui.btnUser4.setText('Execute')
        self.ui.btnUser4.clicked.connect(partial(nxt_if_arm_init, 'execute'))


	    #Add buttons for teach and follow                                                                
        self.ui.btnUser5.setText('record_step')                                                          
        self.ui.btnUser5.clicked.connect(partial(nxt_if_arm_init, 'record_step'))                        
                                                                                                         
        self.ui.btnUser6.setText('start teaching')                                                       
        self.ui.btnUser6.clicked.connect(partial(nxt_if_arm_init, 'start_teaching'))                     
                                                                                                         
        self.ui.btnUser7.setText('stop teaching')                                                        
        self.ui.btnUser7.clicked.connect(partial(nxt_if_arm_init, 'stop_teaching'))                      
                                                                                                         
        self.ui.btnUser8.setText('play_back')                                                            
        self.ui.btnUser8.clicked.connect(partial(nxt_if_arm_init, 'play_back'))                          
                                                                                                         
       
        self.ui.btnUser10.setText('Open Gripper in Teaching')
        self.ui.btnUser10.clicked.connect(lambda: open_gripper_teach())
        
        self.ui.btnUser9.setText('Close Gripper in Teaching')
        self.ui.btnUser9.clicked.connect(lambda: close_gripper_teach())

        self.ui.btnUser11.setText('Detect')
        self.ui.btnUser11.clicked.connect(partial(nxt_if_arm_init, 'detect'))

        
        ### Final Competition Buttons.
        #Event 1: Pick and Sort
        self.ui.btn_task1.setText('Event 1: Pick n sort!')
        self.ui.btn_task1.clicked.connect(partial(nxt_if_arm_init, 'event1'))
        
        #Event 2: Pick and Stack
        self.ui.btn_task2.setText('Event 2: Pick n stack!')
        self.ui.btn_task2.clicked.connect(partial(nxt_if_arm_init, 'event2'))

        #Event 3: Line 'em up
        self.ui.btn_task3.setText('Event 3: Line em up!')
        self.ui.btn_task3.clicked.connect(partial(nxt_if_arm_init, 'event3'))

        #Event 4: Stack 'em high!
        self.ui.btn_task4.setText('Event 4: Stack em high!')
        self.ui.btn_task4.clicked.connect(partial(nxt_if_arm_init, 'event4'))

        #Event 5: Bonous event
        self.ui.btn_task5.setText('Event 5: To the Sky!')
        self.ui.btn_task5.clicked.connect(partial(nxt_if_arm_init, 'event5'))

	    ##############################################

                                                     
        self.ui.videoDisplay.mousePressEvent = self.ClicktoGrab 



        # Sliders
        for sldr in self.joint_sliders:
            sldr.valueChanged.connect(self.sliderChange)
        self.ui.sldrMoveTime.valueChanged.connect(self.sliderChange)
        self.ui.sldrAccelTime.valueChanged.connect(self.sliderChange)
        # Direct Control
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        # Status
        self.ui.rdoutStatus.setText("Waiting for input")
        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)
        """Setup Threads"""

        # State machine
        self.StateMachineThread = StateMachineThread(self.sm)
        self.StateMachineThread.updateStatusMessage.connect(
            self.updateStatusMessage)
        self.StateMachineThread.start()
        self.VideoThread = VideoThread(self.camera)

        self.VideoThread.updateFrame.connect(self.setImage)

        self.VideoThread.start()
        self.ArmThread = RXArmThread(self.rxarm)
        self.ArmThread.updateJointReadout.connect(self.updateJointReadout)
        self.ArmThread.updateEndEffectorReadout.connect(
            self.updateEndEffectorReadout)
        self.ArmThread.start()

    """ Slots attach callback functions to signals emitted from threads"""

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        for rdout, joint in zip(self.joint_readouts, joints):
            rdout.setText(str('%+.2f' % (joint * R2D)))

    ### TODO: output the rest of the orientation according to the convention chosen
    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f mm" % (1000 *pos[0])))
        self.ui.rdoutY.setText(str("%+.2f mm" % (1000 *pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f mm" % (1000 *pos[2])))
        self.ui.rdoutPhi.setText(str("%+.2f rad" % (pos[3])))
        self.ui.rdoutTheta.setText(str("%+.2f" % (pos[4])))
        self.ui.rdoutPsi.setText(str("%+.2f" % (pos[5])))

    @pyqtSlot(QImage, QImage, QImage)
    def setImage(self, rgb_image, depth_image, tag_image):
        """!
        @brief      Display the images from the camera.

        @param      rgb_image    The rgb image
        @param      depth_image  The depth image
        """
        if (self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if (self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
        if (self.ui.radioUsr1.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(tag_image))
            self.camera.detectState = True
        else:
            self.camera.detectState = False

    """ Other callback functions attached to GUI elements"""

    def estop(self):
        self.rxarm.disable_torque()
        self.sm.set_next_state('estop')

    def sliderChange(self):
        """!
        @brief Slider changed

        Function to change the slider labels when sliders are moved and to command the arm to the given position
        """
        for rdout, sldr in zip(self.joint_slider_rdouts, self.joint_sliders):
            rdout.setText(str(sldr.value()))

        self.ui.rdoutMoveTime.setText(
            str(self.ui.sldrMoveTime.value() / 10.0) + "s")
        self.ui.rdoutAccelTime.setText(
            str(self.ui.sldrAccelTime.value() / 20.0) + "s")
        self.rxarm.set_moving_time(self.ui.sldrMoveTime.value() / 10.0) 
        self.rxarm.set_accel_time(self.ui.sldrAccelTime.value() / 20.0)

        # Do nothing if the rxarm is not initialized
        if self.rxarm.initialized:
            joint_positions = np.array(
                [sldr.value() * D2R for sldr in self.joint_sliders])
            # Only send the joints that the rxarm has
            self.rxarm.set_positions(joint_positions[0:self.rxarm.num_joints])

    def calibrateMousePress(self, mouse_event): 
        """!
        @brief Record mouse click positions for calibration

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """
        """ Get mouse posiiton """
        pt = mouse_event.pos()
        self.camera.last_click[0] = pt.x()
        self.camera.last_click[1] = pt.y()
        self.camera.new_click = True

    
    def directControlChk(self, state):
        """!
        @brief      Changes to direct control mode

                    Will only work if the rxarm is initialized.

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.rxarm.initialized:
            # Go to manual and enable sliders
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            # Lock sliders and go to idle
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)
            self.ui.chk_directcontrol.setChecked(False)

    #################################################################################################
    #Manual calibration
    def trackMouse(self, mouse_event):
        """!
        @brief      Show the mouse position in GUI

                    TODO: after implementing workspace calibration display the world coordinates the mouse points to in the RGB
                    video image.

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """
        
        pt = mouse_event.pos()
        if self.camera.DepthFrameRaw.any() != 0:
            z = self.camera.DepthFrameRaw[pt.y()][pt.x()]  
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" %
                                             (pt.x(), pt.y(), z))           
            u = pt.x()
            v = pt.y()
            d = z
            K = np.array([[904.5715942382812, 0.0,               635.9815063476562],
             [0.0,               905.2954711914062, 353.06036376953125], 
             [0.0,               0.0,               1.0               ]],dtype=np.float64)
            pixel = np.array([[u],
                  [v], 
                  [1]],dtype=np.float64)
            camera_coordinate = d*np.matmul(np.linalg.inv(K),pixel)
            camera_coordinate_homogenuous = np.concatenate([camera_coordinate,np.ones([1,1])])
            translation_matrix = np.array([[1,0,0,-30],
                                           [0,1,0,340], 
                                           [0,0,1,995],
                                           [0,0,0,1]],dtype=np.float64)

            angle1 = np.pi - 0.2 
            rotation_matrix_x = np.array([[1,0                ,0                 ,0  ],
                                          [0,np.cos(angle1)   ,-np.sin(angle1)   ,0  ], 
                                          [0,np.sin(angle1)   ,np.cos(angle1)    ,0  ],
                                          [0,0                ,0                 ,1  ]],dtype=np.float64)


            
            H = np.matmul(translation_matrix,rotation_matrix_x)


            world_coordinate = np.matmul(H,camera_coordinate_homogenuous) 
            wx = int(world_coordinate[0,0])
            wy = int(world_coordinate[1,0])
            wz = int(world_coordinate[2,0])
            
            self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" %
                                             (wx, wy, wz))

            if self.sm.calibrate_ == True:
                print('This registered')
                H_At = self.sm.calibrate()
                world_coordinate_At = np.matmul(H_At,camera_coordinate_homogenuous)
                self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" % (wx,wy,wz))
                print(world_coordinate_At[0], world_coordinate_At[1], world_coordinate_At[2])





    def initRxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.ui.SliderFrame.setEnabled(False)
        self.ui.chk_directcontrol.setChecked(False)
        self.rxarm.enable_torque()
        self.sm.set_next_state('initialize_rxarm')


    ########################################################################################################
    #grab and drop
    def ClicktoGrab(self,mouse_event):

        wx,wy,wz = self.camera.PixeltoWorld(mouse_event) 

        self.rxarm.open_gripper()
        temp = [wx, wy,wz+150,-np.pi/2,0]
        blockPose = [wx, wy, wz+10,-np.pi/2,0]

        angle_1,angle_2 = IK_geometric(temp)
        self.rxarm.set_positions(angle_2)
        time.sleep(2)

        angle_1,angle_2 = IK_geometric(blockPose)
        self.rxarm.set_positions(angle_2)
        time.sleep(2.5)
        self.rxarm.close_gripper()
        
        angle_1,angle_2 = IK_geometric(temp)
        self.rxarm.set_positions(angle_2)
    
        self.ui.videoDisplay.mousePressEvent = self.ClicktoDrop
        print("grab")

    def ClicktoDrop(self,mouse_event):
        self.rxarm.close_gripper()
        wx,wy,wz = self.camera.PixeltoWorld(mouse_event)

        dest_temp = [wx,wy,wz+150,-np.pi/2,0]
        dest = [wx,wy,wz+40,-np.pi/2,0]

        angle_1,angle_2 = IK_geometric(dest_temp)
        self.rxarm.set_positions(angle_2)
        time.sleep(2)

        angle_1,angle_2 = IK_geometric(dest) 
        self.rxarm.set_positions(angle_2)
        time.sleep(2)
        self.rxarm.open_gripper()

        angle_1,angle_2 = IK_geometric(dest_temp) 
        self.rxarm.set_positions(angle_2)
        

        print("drop")


        self.ui.videoDisplay.mousePressEvent = self.ClicktoGrab
    ######################################################


### TODO: Add ability to parse POX config file as well
def main(args=None):
    """!
    @brief      Starts the GUI
    """
    app = QApplication(sys.argv)
    app_window = Gui(dh_config_file=args['dhconfig'])
    app_window.show()
    sys.exit(app.exec_())




# Run main if this file is being run directly
### TODO: Add ability to parse POX config file as well
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-c",
                    "--dhconfig",
                    required=False,
                    help="path to DH parameters csv file")
    main(args=vars(ap.parse_args()))







