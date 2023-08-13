"""!
Class to represent the camera.
"""

import cv2
import time
import numpy as np
from PyQt4.QtGui import QImage
from PyQt4.QtCore import QThread, pyqtSignal, QTimer
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from apriltag_ros.msg import *
from cv_bridge import CvBridge, CvBridgeError
import copy
import argparse


class Camera():
    """!
    @brief      This class describes a camera.
    """
    def __init__(self):
        """!
        @brief      Construcfalsets a new instance.
        """
        self.VideoFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.GridFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((720, 1280)).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.array([])

        # mouse clicks & calibration variables
        self.cameraCalibrated = False
        self.intrinsic_matrix = np.array([])
        self.extrinsic_matrix = np.array([])
        self.last_click = np.array([0, 0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5, 2), int)
        self.depth_click_points = np.zeros((5, 2), int)
        self.grid_x_points = np.arange(-450, 500, 50)
        self.grid_y_points = np.arange(-175, 525, 50)
        self.grid_points = np.array(np.meshgrid(self.grid_x_points, self.grid_y_points))
        self.tag_detections = np.array([])
        self.tag_locations = [[-250, -25], [250, -25], [250, 275]]

        self.detectState = True 
        """ block info """ 
        self.color_labels = list((
            {'id': 'red', 'color': (10, 10, 127)},
            {'id': 'orange', 'color': (30, 75, 150)},
            {'id': 'yellow', 'color': (30, 150, 200)},
            {'id': 'green', 'color': (20, 60, 20)},
            {'id': 'blue', 'color': (100, 50, 0)},
            {'id': 'violet', 'color': (100, 40, 80)})
        )

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        
        self.block_contours = np.array([])
        self.block_detections = np.array([])

        self.pts1 = np.array([-540, 455, 503, 503, 550, -177, -520, -220]).reshape((4,2))
        self.pts2 = np.array([0, 0, 0, 1200, 1280, 1280, 1280, 0]).reshape((4, 2))


    def processVideoFrame(self): 
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(self.VideoFrame, self.block_contours, -1,
                         (255, 0, 255), 3)

    def ColorizeDepthFrame(self): 
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[..., 0] = self.DepthFrameRaw >> 1
        self.DepthFrameHSV[..., 1] = 0xFF
        self.DepthFrameHSV[..., 2] = 0x9F
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,
                                          cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self): 
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png", cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB)

        pass

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",
                                        0).astype(np.uint16)

        pass

    def convertQtVideoFrame(self): 
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:

            Per =  self.getAffineTransform()
            

            #Uncomment line below to apply transform to the RGB image (in general either to distorted or black screen)
            #self.VideoFrame = cv2.warpPerspective(self.VideoFrame, Per, (self.VideoFrame.shape[1], self.VideoFrame.shape[0]))


            self.VideoFrame = cv2.resize(self.VideoFrame, (1280, 720)) #works

            

            img = QImage(self.VideoFrame, self.VideoFrame.shape[1], self.VideoFrame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None
        

    def convertQtGridFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.GridFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None


    def convertQtDepthFrame(self): 
        """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
        try:
            img = QImage(self.DepthFrameRGB, self.DepthFrameRGB.shape[1],
                         self.DepthFrameRGB.shape[0], QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtTagImageFrame(self): 
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.TagImageFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def getAffineTransform(self):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

        @param      coord1  Points in coordinate frame 1
        @param      coord2  Points in coordinate frame 2

        @return     Affine transform between coordinates.
        """
        detections_xyz_camera = np.zeros((4,4))
        temp = np.zeros((4,1))


        detections_xyz_world = np.array([[-250,275,self.DepthFrameRaw[275][-250]],
                                         [250,275,self.DepthFrameRaw[275][250]],
                                         [250,-25,self.DepthFrameRaw[-25][250]],
                                         [-250,-25, self.DepthFrameRaw[-25][-250]]
            ])

        
        #Does not produce a stable image: Retain for possible future fixes
        msg = self.tag_detections
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


        worldtemp = np.concatenate((Xw,Yw),axis=0)
        world = np.concatenate((worldtemp,Zw), axis =0)
        world = np.reshape(world,(3,4))
        world = np.transpose(world)

        detCamOut = detections_xyz_camera[:,:3]

        H = cv2.findHomography(detections_xyz_world,detCamOut)[0]

        return H 


    def PixeltoWorld(self, mouse_event): 
            pt = mouse_event.pos()
            if self.DepthFrameRaw.any() != 0:
                z = self.DepthFrameRaw[pt.y()][pt.x()]

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
                return wx,wy,wz
            
    

    def loadCameraCalibration(self, file): #Defined at top of class
        """!
        @brief      Load camera intrinsic matrix from file.

                    TODO: use this to load in any calibration files you need to

        @param      file  The file
        """
        pass

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        pass

    
    
    def retrieve_area_color(self, data, contour): 
        labels = self.color_labels
        mask = np.zeros(data.shape[:2], dtype="uint8")
        cv2.drawContours(mask, [contour], -1, 255, -1)
        mean = cv2.mean(data, mask=mask)[:3]
        min_dist = (np.inf, None)
        
        for label in labels:
            d = np.linalg.norm(label["color"] - np.array(mean))
            if d < min_dist[0]:
                min_dist = (d, label["id"])
        color = min_dist[1]     
        return color
    
    def retrieve_area_color_hsv(self,img, contour):
        mask = np.zeros(img.shape[:2], dtype="uint8")
        cv2.drawContours(mask, [contour], -1, 255, -1)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        masked_img = cv2.bitwise_and(hsv, hsv, mask=mask)
        h, s, v = cv2.split(masked_img)
        h_hist = cv2.calcHist([h], [0], None, [180], [0,180])
        h_value = np.argmax(h_hist[1:])
        color_h_value = np.array([50, 5, 15, 90, 110, 120]) #or was 170, v=15, b=25
        color_dict = ['green', 'violet', 'blue','yellow',  'orange', 'red']
        dist = np.absolute(h_value - color_h_value)

        # compute area of a contour
        area = cv2.contourArea(contour)
        #print(area)
        if area > 18:
            block_size = 'L'
        else:
            block_size = 'S'
        
        return color_dict[np.argmin(dist)], h_value, block_size,area
    

    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    #TODO: Implement a blob detector to find blocks in the depth image"""
        
        
        font = cv2.FONT_HERSHEY_SIMPLEX 

        cv_depthIm = self.DepthFrameRaw
        
        mask = np.zeros_like(cv_depthIm, dtype=np.uint8)
    
        cv2.rectangle(self.VideoFrame, (220,65),(1050,480), (255, 0, 0), 2) 
        cv2.rectangle(mask, (220,65),(1050,480), 255, cv2.FILLED) 
        

        cv2.rectangle(mask, (575,344),(760,720), 0, cv2.FILLED) 
        cv2.rectangle(self.VideoFrame, (575,344),(760,720), (255, 0, 0), 2) # Arm reigon
        
        #thresh = cv2.bitwise_and(cv2.inRange(cv_depthIm, lower, upper), mask)#958 995
        hsv = cv2.cvtColor(self.VideoFrame, cv2.COLOR_BGR2HSV)
        hsvMask = cv2.inRange(hsv, np.array([1,150,40]),np.array([249,249,245]))
       
        thresh = cv2.bitwise_and(hsvMask, mask)

        #Get contours
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #New CHAIN_APPROX_SIMPLE
        self.blocks_detections = contours

        #Get the block colors, centers, angles, sizes and thresh
        colors = []
        blockCenters = []
        blockAngles = []
        blockSizes = []
        areas = []

        for i in range(len(self.blocks_detections)):
          
            contour = self.blocks_detections[i]

            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cz = self.DepthFrameRaw[cy][cx]   

            blockAngle = cv2.minAreaRect(contour)[2]  
            blockAngle = int(blockAngle)

            color, ind, blockSize,area = self.retrieve_area_color_hsv(self.VideoFrame, contour) 


            #Display
            cv2.putText(self.VideoFrame, color, (cx-30, cy+40), font, 1.0, (0,0,0), thickness=1) 
            #cv2.putText(self.VideoFrame, str(blockAngle)+blockSize, (cx, cy), font, 0.5, (255,255,255), thickness=2)
            cv2.drawContours(self.VideoFrame, contours, -1, (0,255,255), thickness = 1)

            #Prep. out data
            colors.append([color])
            blockCenters.append([cx, cy,cz])
            blockAngles.append(blockAngle)
            blockSizes.append(blockSize)
            areas.append(area)

            #cv2.imwrite('contour.png', self.VideoFrame) #Takes a frame image
       

        return colors, blockCenters, blockAngles, blockSizes, areas
    

    def projectGridInRGBImage(self):
        """!
        @brief      projects

                    TODO: Use the intrinsic and extrinsic matricies to project the gridpoints 
                    on the board into pixel coordinates. copy self.VideoFrame to self.GridFrame and
                    and draw on self.GridFrame the grid intersection points from self.grid_points
                    (hint: use the cv2.circle function to draw circles on the image)
        """
        pass
        

class ImageListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
           
        except CvBridgeError as e:
            print(e)
        self.camera.VideoFrame = cv_image



class TagImageListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge() 
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
        self.camera.TagImageFrame = cv_image
        if self.camera.detectState:
            self.camera.detectBlocksInDepthImage()
            rospy.sleep(0.005) 

class TagDetectionListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, AprilTagDetectionArray,
                                        self.callback)
        self.camera = camera

    def callback(self, data):
        self.camera.tag_detections = data
        #print(self.camera.tag_detections )
        #for detection in data.detections:
        #print(detection.id[0])
        #print(detection.pose.pose.pose.position)


class CameraInfoListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, CameraInfo, self.callback)
        self.camera = camera

    def callback(self, data):
        self.camera.K = np.reshape(data.K, (3, 3))
        #print(self.camera.intrinsic_matrix)


class DepthListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
            #cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)
        self.camera.DepthFrameRaw = cv_depth
        #self.camera.DepthFrameRaw = self.camera.DepthFrameRaw/2
        self.camera.ColorizeDepthFrame()


class VideoThread(QThread):

    updateFrame = pyqtSignal(QImage, QImage, QImage,QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        tag_image_topic = "/tag_detections_image"
        tag_detection_topic = "/tag_detections"
        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        tag_image_listener = TagImageListener(tag_image_topic, self.camera)
        camera_info_listener = CameraInfoListener(camera_info_topic,
                                                  self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic,
                                                      self.camera)
        


    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Grid window", cv2.WINDOW_NORMAL)

          

            time.sleep(0.5)
        while True:
            rgb_frame = self.camera.convertQtVideoFrame()
            depth_frame = self.camera.convertQtDepthFrame()
            tag_frame = self.camera.convertQtTagImageFrame()
            self.camera.projectGridInRGBImage()
            grid_frame = self.camera.convertQtGridFrame()


            if ((rgb_frame != None) & (depth_frame != None)):
                self.updateFrame.emit(rgb_frame, depth_frame, tag_frame, grid_frame)
            time.sleep(0.03)
            if __name__ == '__main__':

                cv2.imshow(
                    "Image window",
                    cv2.cvtColor(self.camera.VideoFrame, cv2.COLOR_RGB2BGR))
                    
                cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                cv2.imshow(
                    "Tag window",
                    cv2.cvtColor(self.camera.TagImageFrame, cv2.COLOR_RGB2BGR))
                cv2.imshow("Grid window",
                    cv2.cvtColor(self.camera.GridFrame, cv2.COLOR_RGB2BGR))

                cv2.waitKey(3)
                time.sleep(0.03)
    

if __name__ == '__main__':
    camera = Camera()
    videoThread = VideoThread(camera)
    videoThread.start()
    rospy.init_node('realsense_viewer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
