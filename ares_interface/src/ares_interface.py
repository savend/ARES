#!/usr/bin/env python3
 
import rospy
from std_msgs.msg import Float32, Float64, Int32, Bool
from sensor_msgs.msg import BatteryState,Image


import cv2
import numpy as np
from cv_bridge import CvBridge
#import pyrealsense2
#from realsense_depth import *
from typing import Tuple

# Information for DIstanceMesureing
imageWidth = 640
imageHeight = 480

heightDistanceMessureing = imageHeight * (3/4)
posMid = imageWidth / 2
posRight = 2 * imageWidth / 3
posLeft = imageWidth / 3
TextSpace = 10

pointMid = (int(posMid), int(heightDistanceMessureing))
pointRight = (int(posRight), int(heightDistanceMessureing))
pointLeft = (int(posLeft), int(heightDistanceMessureing))
pointFRONT = (int(imageWidth / 2), int(imageHeight / 2))


# SensorBox Postions
boxSpace = 20
backGround = (57, 58, 54)
rim = (0, 0, 0)

sensorBoxWidth = 120
sensorBoxHeight = 45
sensorBoxColumRight = boxSpace
sensorBoxColumLeft: int = imageWidth - sensorBoxWidth - boxSpace

sensorBoxLevel1: int = imageHeight / 2 - 180
sensorBoxLevel2: int = imageHeight / 2 - 60
sensorBoxLevel3: int = imageHeight / 2 + 60
sensorBoxLevel4: int = imageHeight / 2 + 180

# SensorBoxes Information

textType = cv2.FONT_HERSHEY_SIMPLEX
red = (0, 0, 225)
textThikness = 2
texFontSize = 0.5



class ImageProcess:


    def __init__(self):
        self.o2_data = 0
        #self.env_Temp_data = 0
        #self.envPressure_data = 0
        #self.envHumid_data = 0
        self.objTemp_data = 0
        self.ambientTemp_data = 0
        self.battery_data = 0
        self.casePath = "haarcascade.xml"
        self.faceCascade: object = cv2.CascadeClassifier(self.casePath)
        #self.img
        #self.depth_img
        self.distanceMid = 0
        self.distanceRight = 0
        self.distanceLeft = 0
        self.distanceFRONT = 0
        self.bridge = CvBridge()

        self.img_pub= rospy.Publisher("/image", Image, queue_size=10)

        self.o2_sub = rospy.Subscriber("o2_concentration", Float32,self.o2_sensor_callback)
        self.battery_sub = rospy.Subscriber("battery_state", BatteryState, self.batteryState_callback)
        #self.env_temp_sub = rospy.Subscriber("env_temp", Float32, self.env_temp_sensor_callback)
        #rospy.Subscriber("env_pres", Int32, self.env_pressure_sensor_callback)
        #rospy.Subscriber("env_hum", Float32, self.env_humid_sensor_callback)
        self.ambient_temp_sub = rospy.Subscriber("ambient_temp", Float64, self.ambient_temp_sensor_callback)
        self.obj_temp_sub = rospy.Subscriber("object_temp", Float64, self.obj_temp_sensor_callback)
        self.emergency_sub = rospy.Subscriber("emergency_button_state", Bool, self.emergency_button_state_callback)
        self.color_frame_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_color_callback) #DepthCamera()
        self.depth_frame_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.camera_depth_callback) #DepthCamera()



    def o2_sensor_callback(self,o2_msg):
        self.o2_data=o2_msg.data

    def batteryState_callback(self,battery_msg):
        self.battery_data = battery_msg.voltage

    #def env_temp_sensor_callback(self,envTemp_msg):
    #   env_Temp_data=envTemp_msg.data

    #def env_pressure_sensor_callback(self,envPressure_msg):
    #    envPressure_data=envPressure_msg.data

    #def env_humid_sensor_callback(self,envHumid_msg):
    #    envHumid_data=envHumid_msg.data

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

        self.distanceMid = round((self.depth_img[pointMid[1], pointMid[0]]) / 10)
        self.distanceRight = round((self.depth_img[pointRight[1], pointRight[0]]) / 10)
        self.distanceLeft = round((self.depth_img[pointLeft[1], pointLeft[0]]) / 10)
        self.distanceFRONT = round((self.depth_img[pointFRONT[1], pointFRONT[0]]) / 10)


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
                cv2.putText(img, str(self.distanceMid) + "cm", (int(posMid + TextSpace), int(heightDistanceMessureing)), textType,
                            texFontSize, red, textThikness)
                cv2.putText(img, str(self.distanceRight) + "cm", (int(posRight + TextSpace), int(heightDistanceMessureing)),
                            textType, texFontSize, red, textThikness)
                cv2.putText(img, str(self.distanceLeft) + "cm", (int(posLeft + TextSpace), int(heightDistanceMessureing)), textType,
                            texFontSize, red, textThikness)

                # Box for TEMP_Front
                cv2.rectangle(img, (sensorBoxColumRight, int(sensorBoxLevel2)),
                              (sensorBoxColumRight + sensorBoxWidth, int(sensorBoxLevel2) + sensorBoxHeight), rim, 3, )
                cv2.rectangle(img, (sensorBoxColumRight, int(sensorBoxLevel2)),
                              (sensorBoxColumRight + sensorBoxWidth, int(sensorBoxLevel2) + sensorBoxHeight),
                              backGround, -1, )

                cv2.putText(img, str(self.objTemp_data),
                            (sensorBoxColumRight + TextSpace, (int(sensorBoxLevel2) + sensorBoxHeight - TextSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFontSize, red, textThikness)

                # Box for DIstanz_Front
                cv2.rectangle(img, (sensorBoxColumRight, int(sensorBoxLevel3)),
                              (sensorBoxColumRight + sensorBoxWidth, int(sensorBoxLevel3) + sensorBoxHeight), rim, 3, )
                cv2.rectangle(img, (sensorBoxColumRight, int(sensorBoxLevel3)),
                              (sensorBoxColumRight + sensorBoxWidth, int(sensorBoxLevel3) + sensorBoxHeight),
                              backGround, -1, )

                cv2.circle(img, pointFRONT, 6, (0, 0, 255), 3)

                cv2.putText(img, str(self.distanceFRONT) + "cm",
                            (sensorBoxColumRight + TextSpace, (int(sensorBoxLevel3) + sensorBoxHeight - TextSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFontSize, red, textThikness)

                # Box for O2 Robot
                cv2.rectangle(img, (sensorBoxColumLeft, int(sensorBoxLevel2)),
                              (sensorBoxColumLeft + sensorBoxWidth, int(sensorBoxLevel2) + sensorBoxHeight), rim, 3, )
                cv2.rectangle(img, (sensorBoxColumLeft, int(sensorBoxLevel2)),
                              (sensorBoxColumLeft + sensorBoxWidth, int(sensorBoxLevel2) + sensorBoxHeight), backGround,
                              -1, )

                cv2.putText(img, str(self.o2_data),
                            (sensorBoxColumLeft, (int(sensorBoxLevel2) + sensorBoxHeight - TextSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFontSize, red, textThikness)

                # Box for TEMP Robot
                cv2.rectangle(img, (sensorBoxColumLeft, int(sensorBoxLevel3)),
                              (sensorBoxColumLeft + sensorBoxWidth, int(sensorBoxLevel3) + sensorBoxHeight), rim, 3, )
                cv2.rectangle(img, (sensorBoxColumLeft, int(sensorBoxLevel3)),
                              (sensorBoxColumLeft + sensorBoxWidth, int(sensorBoxLevel3) + sensorBoxHeight), backGround,
                              -1, )

                cv2.putText(img, str(self.ambientTemp_data),
                            (sensorBoxColumLeft, (int(sensorBoxLevel3) + sensorBoxHeight - TextSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFontSize, red, textThikness)

                # Box for Humidity Robot
                #cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_1)),
                #              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_1) + SensorBox_height), Rim, 3, )
                #cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_1)),
                #              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_1) + SensorBox_height), background,
                #              -1, )
                #cv2.putText(img, str(self.envHumid_data),
                #            (SensorBoxColum_Left, (int(SensorBox_Level_1) + SensorBox_height - TextSpace)),
                #            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for Pressure Robot
                #cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_4)),
                #              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_4) + SensorBox_height), Rim, 3, )
                #cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_4)),
                #              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_4) + SensorBox_height), background,
                #              -1, )

                #cv2.putText(img, str(self.envPressure_data),
                #            (SensorBoxColum_Left, (int(SensorBox_Level_4) + SensorBox_height - TextSpace)),
                #            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for Batterie
                #cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_4)),(SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_4) + SensorBox_height), Rim, 3, )
                #cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_4)),(SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_4) + SensorBox_height), background,-1, )

                #cv2.putText(img, str(self.envPressure_data),(SensorBoxColum_Left, (int(SensorBox_Level_4) + SensorBox_height - TextSpace)),cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

 

                # Box for Signalstärke

                # LOGO ARES
                LogoAres = cv2.imread('ARESLOGO_Black.jpg')
                if LogoAres is None:
                    rospy.loginfo("error")
                h_LoGo = LogoAres.shape[0]
                w_logo = LogoAres.shape[1]
                #h_LoGo, w_logo, _ = LogoAres.shape
                h_roi: int = (boxSpace + int(h_LoGo))
                w_roi: int = (boxSpace + int(w_logo))

                roi = img[boxSpace: h_roi, boxSpace: w_roi]
                combine = cv2.addWeighted(roi, 1, LogoAres, 0.5, 0)
                img[boxSpace: h_roi, boxSpace: w_roi] = combine

                return img

                # Liitle ARES
                #    p1 = (1140, 40)
                #    p2 = (1140, 70)
                #    p3 = (1130, 55)
                #    p4 = (1130, 65)
                #    p5 = (1220, 70)
                #    p6 = (1220, 40)
                #    p7 = (1230, 55)
                #    p8 = (1230, 65)
                #    p9 = (1160, 70)
                #    p10 =(1200, 70)
                #    cv2.rectangle(color_frame, p1, p5, (0, 0, 225), cv2.FILLED)
                #    cv2.rectangle(color_frame, p1, p5, (0, 0, 0), 3, )
                #    cv2.circle(color_frame, p9,10, (0, 0, 255),cv2.FILLED)
                #    cv2.circle(color_frame, p10,10, (0, 0, 255), cv2.FILLED)
                #    cv2.circle(color_frame, (p9), 10, (0, 0, 0), 5)
                #    cv2.circle(color_frame, (p10), 10, (0, 0, 0), 5)
                #    cv2.line(color_frame, p1, p3, (0, 0, 0), 3)
                #    cv2.line(color_frame, p3, p4, (0, 0, 0), 3)
                #    cv2.line(color_frame, p4, p2, (0, 0, 0), 3)
                #    cv2.line(color_frame, p6, p7, (0, 0, 0), 3)
                #    cv2.line(color_frame, p7, p8, (0, 0, 0), 3)
                #    cv2.line(color_frame, p8, p5, (0, 0, 0), 3)

                # Battery DIsplay

                # cv2.imshow("depth frame", depth_frame)






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
