
#!/usr/bin/env python3
 
import rospy
from std_msgs.msg import Float32, Float64, Int32, Bool
from sensor_msgs.msg import BatteryState,Image

import os
import cv2
import numpy as np
from cv_bridge import CvBridge
#import pyrealsense2
#from realsense_depth import *
from typing import Tuple

# Information for DIstanceMesureing
imageWidth = 640
imageHeight = 480

heightDistanceMessureing = imageHeight * (3 / 4)
posMid = imageWidth / 2
posRight = 2 * imageWidth / 3
posLeft = imageWidth / 3
textSpace = 10

pointMid = (int(posMid), int(heightDistanceMessureing))
pointRight = (int(posRight), int(heightDistanceMessureing))
pointLeft = (int(posLeft), int(heightDistanceMessureing))
pointFRONT = (int(imageWidth / 2), int(imageHeight / 2))


# SensorBox Postions
boxSpace = 20
background = (57, 58, 54)
Rim = (0, 0, 0)

SensorBox_width = 120
SensorBox_height = 45
SensorBoxColum_Right = boxSpace
SensorBoxColum_Left: int = imageWidth - SensorBox_width - boxSpace

SensorBox_Level_1: int = imageHeight / 2 - 180
SensorBox_Level_2: int = imageHeight / 2 - 60
SensorBox_Level_3: int = imageHeight / 2 + 60
SensorBox_Level_4: int = imageHeight / 2 + 120

# SensorBoxes Information

textType = cv2.FONT_HERSHEY_SIMPLEX
red = (0, 0, 225)
textThikness = 2
texFont_Size = 0.5



class ImageProcess:


    def __init__(self):
        self.o2_data = 0
        #self.env_Temp_data = 0
        #self.envPressure_data = 0
        #self.envHumid_data = 0
        self.objTemp_data = 0
        self.ambientTemp_data = 0
        self.battery_data = 0

        #absFilePath_Haarcadcade= os.path.abspath('haarcascade.xml')

        self.casePath = "/home/rembomaster/catkin_ws/src/ARES/ares_interface/src/haarcascade.xml"
        #self.casePath = "haarcascade.xml"

        self.faceCascade: object = cv2.CascadeClassifier(self.casePath)
        #self.img
        #self.depth_img
        self.distance_mid = 0
        self.distance_right = 0
        self.distance_left = 0
        self.distance_FRONT = 0
        self.bridge = CvBridge()
        self.emergencStop_data = 0

        self.img_pub= rospy.Publisher("/image", Image, queue_size=10)

        self.o2_sub = rospy.Subscriber("o2_concentration", Float32,self.o2_sensor_callback)
        self.battery_sub = rospy.Subscriber("battery_state", BatteryState, self.batteryState_callback)
        self.ambient_temp_sub = rospy.Subscriber("ambient_temp", Float64, self.ambient_temp_sensor_callback)
        self.obj_temp_sub = rospy.Subscriber("object_temp", Float64, self.obj_temp_sensor_callback)
        self.emergency_sub = rospy.Subscriber("/emergency_button_state", Bool, self.emergency_button_state_callback)
        self.color_frame_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_color_callback) #DepthCamera()
        self.depth_frame_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.camera_depth_callback) #DepthCamera()



    def o2_sensor_callback(self,o2_msg):
        self.o2_data=o2_msg.data

    def batteryState_callback(self,battery_msg):
        self.battery_data = battery_msg.voltage


    def ambient_temp_sensor_callback(self,ambientTemp_msg):
        self.ambientTemp_data=ambientTemp_msg.data

    def obj_temp_sensor_callback(self,objTemp_msg):
        self.objTemp_data=objTemp_msg.data

    def emergency_button_state_callback(self,emergencStop_msg):
        self.emergencStop_data=emergencStop_msg.data


    def camera_color_callback(self, color_msg):
        self.img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        self.img = self.interfaceDrawing(self.img)
        self.img = self.peopleRecnognition(self.img)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, encoding='passthrough'))



    def camera_depth_callback(self, depth_msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        self.distance_mid = round((self.depth_img[pointMid[1], pointMid[0]]) / 10)
        self.distance_right = round((self.depth_img[pointRight[1], pointRight[0]]) / 10)
        self.distance_left = round((self.depth_img[pointLeft[1], pointLeft[0]]) / 10)
        self.distance_FRONT = round((self.depth_img[pointFRONT[1], pointFRONT[0]]) / 10)


    def peopleRecnognition(self, img):

        # Recognition People

        #img = color_frame
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.faceCascade.detectMultiScale( gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE )

        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 225, 0), 2)

            x_medium = int((x + x + w) / 2)  # finding de center of the face
            y_medium = int((y + y + h) / 2)
       
        return img

    def interfaceDrawing(self, img):

                # Structure of the depth_Messurance
                cv2.circle(img, pointMid, 6, (0, 0, 255), 3)
                cv2.circle(img, pointRight, 6, (0, 0, 255), 3)
                cv2.circle(img, pointLeft, 6, (0, 0, 255), 3)
                

                # print('Right: ' , self.distance_right , '; MID ' , self.distance_mid , '; Left' , self.distance_left)
                cv2.putText(img, str(self.distance_mid) + "cm", (int(posMid + textSpace), int(heightDistanceMessureing)), textType,
                            texFont_Size, red, textThikness)
                cv2.putText(img, str(self.distance_right) + "cm", (int(posRight + textSpace), int(heightDistanceMessureing)),
                            textType, texFont_Size, red, textThikness)
                cv2.putText(img, str(self.distance_left) + "cm", (int(posLeft + textSpace), int(heightDistanceMessureing)), textType,
                            texFont_Size, red, textThikness)

                # Box for TEMP_Front
                cv2.rectangle(img, (SensorBoxColum_Right, int(SensorBox_Level_2)),
                              (SensorBoxColum_Right + SensorBox_width, int(SensorBox_Level_2) + SensorBox_height), Rim, 3, )
                cv2.rectangle(img, (SensorBoxColum_Right, int(SensorBox_Level_2)),
                              (SensorBoxColum_Right + SensorBox_width, int(SensorBox_Level_2) + SensorBox_height),
                              background, -1, )



                cv2.putText(img, str(round(self.objTemp_data,2)),
                            (SensorBoxColum_Right + textSpace, (int(SensorBox_Level_2) + SensorBox_height - textSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for DIstanz_Front
                cv2.rectangle(img, (SensorBoxColum_Right, int(SensorBox_Level_3)),
                              (SensorBoxColum_Right + SensorBox_width, int(SensorBox_Level_3) + SensorBox_height), Rim, 3, )
                cv2.rectangle(img, (SensorBoxColum_Right, int(SensorBox_Level_3)),
                              (SensorBoxColum_Right + SensorBox_width, int(SensorBox_Level_3) + SensorBox_height),
                              background, -1, )

                cv2.circle(img, pointFRONT, 6, (0, 0, 255), 3)

                cv2.putText(img, str(self.distance_FRONT) + "cm",
                            (SensorBoxColum_Right + textSpace, (int(SensorBox_Level_3) + SensorBox_height - textSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for O2 Robot
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_2)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_2) + SensorBox_height), Rim, 3, )
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_2)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_2) + SensorBox_height), background,
                              -1, )

                cv2.putText(img, str(round(self.o2_data,2)),
                            (SensorBoxColum_Left, (int(SensorBox_Level_2) + SensorBox_height - textSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for TEMP Robot
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_3)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_3) + SensorBox_height), Rim, 3, )
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_3)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_3) + SensorBox_height), background,
                              -1, )

                cv2.putText(img, str(round(self.ambientTemp_data,2)),
                            (SensorBoxColum_Left, (int(SensorBox_Level_3) + SensorBox_height - textSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)


                # Box for Batterie
                #cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_4)),(SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_4) + SensorBox_height), Rim, 3, )
                #cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_4)),(SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_4) + SensorBox_height), background,-1, )

                #cv2.putText(img, str(round(self.battery_data,2)) + " Volt", (SensorBoxColum_Left, (int(SensorBox_Level_4) + SensorBox_height - textSpace)), cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                #Warning EmergencyStop

                #if self.emergencStop_data == True:
                    #cv2.rectangle(self.img, (SensorBoxColum_Left, int(SensorBox_Level_1)),(SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_1) + SensorBox_height), background,-1, )
                #    cv2.putText(img, str("Emergency-STOP"), (150 , int(imageHeight / 2 + 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0), 3)




                # Box for Signalstärke

                # LOGO ARES
                #LogoAres = cv2.imread('/home/rembomaster/catkin_ws/src/ARES/ares_interface/src/ARESLOGO_Black_tiny.jpg')

                ##LogoAres = cv2.imread('ARESLOGO_Black_tiny.jpg')
                #if LogoAres is None:
                #    rospy.loginfo("error")
                #h_LoGo = LogoAres.shape[0]
                #w_logo = LogoAres.shape[1]

                #h_LoGo, w_logo, _ = LogoAres.shape
                #h_roi: int = (boxSpace + int(h_LoGo))
                #w_roi: int = (boxSpace + int(w_logo))

                #roi = img[boxSpace: h_roi, boxSpace: w_roi]
                #combine = cv2.addWeighted(roi, 1, LogoAres, 0.5, 0)
                #img[boxSpace: h_roi, boxSpace: w_roi] = combine

                return img



    def run(self):

        while not rospy.is_shutdown():
            #ret, depth_frame, color_frame = self.image.get_frame()

            #img = color_frame
            self.peopleRecnognition(self.img)
            self.img = self.interfaceDrawing(self.img)

            self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, desired_encoding='passthrough'))

            cv2.imshow("Color frame", img)
            key = cv2.waitKey(1)
            if key == 27:  # esc key ends process
                break



if __name__ == '__main__':
    try:
        rospy.init_node('imageProcess', anonymous=True)

        imageProcess = ImageProcess()
        rospy.spin()
        #imageProcess.run()

    except rospy.ROSInterruptException:
        pass
    #finally:
        #

