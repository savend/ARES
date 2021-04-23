#!/usr/bin/env python3
 
import rospy
from std_msgs.msg import Float32, Float64, Int32, Bool
from sensor_msgs.msg import BatteryState,Image


import cv2
import numpy as np
import pyrealsense2
from realsense_depth import *
from typing import Tuple

# Information for DIstanceMesureing
image_width = 1280
image_height = 720

heightDistanceMessureing = 550
POS_Mid: int = image_width / 2
POS_Right: int = 2 * image_width / 3
POS_Left: int = image_width / 3
TextSpace = 10

# SensorBox Postions
BoxSpace = 20
background = (57, 58, 54)
Rim = (0, 0, 0)

SensorBox_width = 120
SensorBox_height = 60
SensorBoxColum_Right = BoxSpace
SensorBoxColum_Left: int = image_width - SensorBox_width - BoxSpace

SensorBox_Level_1: int = image_height / 2 - 180
SensorBox_Level_2: int = image_height / 2 - 60
SensorBox_Level_3: int = image_height / 2 + 60
SensorBox_Level_4: int = image_height / 2 + 180

# SensorBoxes Information

textType = cv2.FONT_HERSHEY_SIMPLEX
red = (0, 0, 225)
textThikness = 3
texFont_Size = 0.8


class ImageProcess:


    def __init__(self):
        self.o2_data = 0
        #self.env_Temp_data = 0
        #self.envPressure_data = 0
        #self.envHumid_data = 0
        self.objTemp_data = 0
        self.ambientTemp_data = 0
        self.battery_data = 0
        self.image = DepthCamera()
        self.casePath = "haarcascade.xml"
        self.faceCascade: object = cv2.CascadeClassifier(self.casePath)

        self.img_pub= rospy.Publisher("/image", Image,queue_size=10)

        self.o2_sub = rospy.Subscriber("o2_concentration", Float32,self.o2_sensor_callback)
        self.battery_sub = rospy.Subscriber("battery_state", BatteryState, self.batteryState_callback)
        #self.env_temp_sub = rospy.Subscriber("env_temp", Float32, self.env_temp_sensor_callback)
        #rospy.Subscriber("env_pres", Int32, self.env_pressure_sensor_callback)
        #rospy.Subscriber("env_hum", Float32, self.env_humid_sensor_callback)
        self.ambient_temp_sub = rospy.Subscriber("ambient_temp", Float64, self.ambient_temp_sensor_callback)
        self.obj_temp_sub = rospy.Subscriber("object_temp", Float64, self.obj_temp_sensor_callback)
        self.emergency_sub = rospy.Subscriber("emergency_button_state", Bool, self.emergency_button_state_callback)



    def o2_sensor_callback(self,o2_msg):
        o2_data=o2_msg.data

    def batteryState_callback(self,battery_msg):
        battery_data = battery_msg.data

    #def env_temp_sensor_callback(self,envTemp_msg):
    #   env_Temp_data=envTemp_msg.data

    #def env_pressure_sensor_callback(self,envPressure_msg):
    #    envPressure_data=envPressure_msg.data

    #def env_humid_sensor_callback(self,envHumid_msg):
    #    envHumid_data=envHumid_msg.data

    def ambient_temp_sensor_callback(self,ambientTemp_msg):
        ambientTemp_data=ambientTemp_msg.data

    def obj_temp_sensor_callback(self,objTemp_msg):
        objTemp_data=objTemp_msg.data

    def emergency_button_state_callback(self,emergencStop_msg):
        emergencStop_data=emergencStop_msg.data

    def peopleRecnognition(self,img):

                # Recognition People

                # img = color_frame
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                faces = self.faceCascade.detectMultiScale(
                    gray,
                    scaleFactor=1.1,
                    minNeighbors=5,
                    minSize=(30, 30),
                    flags=cv2.CASCADE_SCALE_IMAGE  # .cv.CV_HAAR_SCALE_IMAGE has been removed
                )

                for (x, y, w, h) in faces:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 225, 0), 2)

                    x_medium = int((x + x + w) / 2)  # finding de center of the face
                    y_medium = int((y + y + h) / 2)

    def interfaceDrawing(self,img, depth_img):

                # Structure of the depth_Messurance
                point_mid = (int(POS_Mid), heightDistanceMessureing)
                point_right = (int(POS_Right), heightDistanceMessureing)
                point_left = (int(POS_Left), heightDistanceMessureing)
                cv2.circle(img, point_mid, 6, (0, 0, 255), 3)
                cv2.circle(img, point_right, 6, (0, 0, 255), 3)
                cv2.circle(img, point_left, 6, (0, 0, 255), 3)
                distance_mid = round((depth_img[point_mid[1], point_mid[0]]) / 10)
                distance_right = round((depth_img[point_right[1], point_right[0]]) / 10)
                distance_left = round((depth_img[point_left[1], point_left[0]]) / 10)

                # print('Right: ' , distance_right , '; MID ' , distance_mid , '; Left' , distance_left)
                cv2.putText(img, str(distance_mid) + "cm", (int(POS_Mid + TextSpace), heightDistanceMessureing), textType,
                            texFont_Size, red, textThikness)
                cv2.putText(img, str(distance_right) + "cm", (int(POS_Right + TextSpace), heightDistanceMessureing),
                            textType, texFont_Size, red, textThikness)
                cv2.putText(img, str(distance_left) + "cm", (int(POS_Left + TextSpace), heightDistanceMessureing), textType,
                            texFont_Size, red, textThikness)

                # Box for TEMP_Front
                cv2.rectangle(img, (SensorBoxColum_Right, int(SensorBox_Level_2)),
                              (SensorBoxColum_Right + SensorBox_width, int(SensorBox_Level_2) + SensorBox_height), Rim, 3, )
                cv2.rectangle(img, (SensorBoxColum_Right, int(SensorBox_Level_2)),
                              (SensorBoxColum_Right + SensorBox_width, int(SensorBox_Level_2) + SensorBox_height),
                              background, -1, )

                cv2.putText(img, str(self.objTemp_data),
                            (SensorBoxColum_Right + TextSpace, (int(SensorBox_Level_2) + SensorBox_height - TextSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for DIstanz_Front
                cv2.rectangle(img, (SensorBoxColum_Right, int(SensorBox_Level_3)),
                              (SensorBoxColum_Right + SensorBox_width, int(SensorBox_Level_3) + SensorBox_height), Rim, 3, )
                cv2.rectangle(img, (SensorBoxColum_Right, int(SensorBox_Level_3)),
                              (SensorBoxColum_Right + SensorBox_width, int(SensorBox_Level_3) + SensorBox_height),
                              background, -1, )

                point_FRONT = (640, 360)
                cv2.circle(img, point_FRONT, 6, (0, 0, 255), 3)
                distance_FRONT = round((depth_img[point_FRONT[1], point_FRONT[0]]) / 10)
                cv2.putText(img, str(distance_FRONT) + "cm",
                            (SensorBoxColum_Right + TextSpace, (int(SensorBox_Level_3) + SensorBox_height - TextSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for O2 Robot
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_2)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_2) + SensorBox_height), Rim, 3, )
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_2)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_2) + SensorBox_height), background,
                              -1, )

                cv2.putText(img, str(self.o2_data),
                            (SensorBoxColum_Left, (int(SensorBox_Level_2) + SensorBox_height - TextSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for TEMP Robot
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_3)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_3) + SensorBox_height), Rim, 3, )
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_3)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_3) + SensorBox_height), background,
                              -1, )

                cv2.putText(img, str(self.ambientTemp_data),
                            (SensorBoxColum_Left, (int(SensorBox_Level_3) + SensorBox_height - TextSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for Humidity Robot
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_1)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_1) + SensorBox_height), Rim, 3, )
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_1)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_1) + SensorBox_height), background,
                              -1, )

                cv2.putText(img, str(self.envHumid_data),
                            (SensorBoxColum_Left, (int(SensorBox_Level_1) + SensorBox_height - TextSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for Pressure Robot
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_4)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_4) + SensorBox_height), Rim, 3, )
                cv2.rectangle(img, (SensorBoxColum_Left, int(SensorBox_Level_4)),
                              (SensorBoxColum_Left + SensorBox_width, int(SensorBox_Level_4) + SensorBox_height), background,
                              -1, )

                cv2.putText(img, str(self.envPressure_data),
                            (SensorBoxColum_Left, (int(SensorBox_Level_4) + SensorBox_height - TextSpace)),
                            cv2.FONT_HERSHEY_SIMPLEX, texFont_Size, red, textThikness)

                # Box for Batterie

                # Box for Signalstärke

                # LOGO ARES
                LogoAres = cv2.imread('ARESLOGO_Black.jpg')
                h_LoGo, w_logo, _ = LogoAres.shape
                h_roi: int = (BoxSpace + int(h_LoGo))
                w_roi: int = (BoxSpace + int(w_logo))

                roi = img[BoxSpace: h_roi, BoxSpace: w_roi]
                combine = cv2.addWeighted(roi, 1, LogoAres, 0.5, 0)
                img[BoxSpace: h_roi, BoxSpace: w_roi] = combine

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
            ret, depth_frame, color_frame = self.image.get_frame()

            img = color_frame
            self.peopleRecnognition(img)
            self.interfaceDrawing(img, depth_frame)

            self.img_pub.publish(img)

            #cv2.imshow("Color frame", img)
            #key = cv2.waitKey(1)
            #if key == 27:  # esc key ends process
            #    break



if __name__ == '__main__':
    try:
        rospy.init_node('imageProcess', anonymous=True)

        imageProcess = ImageProcess()
        imageProcess.run()

    except rospy.ROSInterruptException:
        pass
    #finally:
        #
