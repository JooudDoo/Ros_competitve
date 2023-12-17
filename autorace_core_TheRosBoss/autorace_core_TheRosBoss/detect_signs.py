from typing import Literal
from collections import deque
import os

import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory

import cv2
import math
import numpy as np

from module.config import (
    DEBUG_LEVEL, 
    TASK_LEVEL,
    FOLLOW_ROAD_MODE
    )

# Если потерял линию то стараться повернуть к ней?
# или наоборот держаться той линии что осталась, но на каком-то растоянии? (среднем за предыдущие время от этой линии)
# Что-то сделать со скоростями, PID регулятор?


class Detect_Signs_Node(Node):

    def __init__(self):
        super().__init__("Detect_Signs_Node")

        self._missions_array = []
        self._ready_missions = []
        self._intersection_checker = 0
        self._find_sign_sub = self.create_subscription(Image, "/color/image", self._callback_finder, 3)
        self._publish_command = self.create_publisher(String, '/sign', 1)
        self._cv_bridge = CvBridge()

        pkg_project_path = get_package_share_directory("autorace_core_TheRosBoss")
        signs_path_folder = f"{pkg_project_path}/signs"

        self.pedestrian_crossing_image = cv2.imread(f"{signs_path_folder}/pedestrian_crossing_sign.png")
        self.traffic_construction_image = cv2.imread(f"{signs_path_folder}/traffic_construction.png") 
        self.traffic_intersection_image = cv2.imread(f"{signs_path_folder}/traffic_intersection.png") 
        self.traffic_parking_image = cv2.imread(f"{signs_path_folder}/traffic_parking.png")
        self.tunnel_image = cv2.imread(f"{signs_path_folder}/tunnel.png")

        self.traffic_left_image = self.__reduce_brightness(cv2.imread(f"{signs_path_folder}/traffic_left.png")[:250, :])
        self.traffic_right_image = self.__reduce_brightness(cv2.imread(f"{signs_path_folder}/traffic_right.png")[:250, :])

        self.switch_camera = False

    def __angle3pt(self, a, b, c):
        """
        Расчет угла между тремя точками
        a.shape: [x,y]
        """

        ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
        return ang + 360 if ang < 0 else ang

    def __reduce_brightness(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(img)
        v = np.clip(v, 0, 210)
        img = cv2.merge((h,s,v))
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        return img

    def __detect_sign(self, query_img, train_img, min_matches, my_color, text): 
        """
        Детекция на фото объектов, совпадающих с заданным.

        query_img - что ищем на фото
        train_img - фото, на котором ищем
        min_matches - минимальное необходимое кол-во совпадений между первыми двумя

        Возвращает:
        - 0/1 - есть на фото что-то или нет
        - len(good) - количество совпадений между фото
        - train_img - исходное фото с обведенным объектом 
        - s - площадь обводки
        - angle - угол между прямыми, выделяющими найденный объект
        """

        flag = 0

        while (flag!=1):
            # обнаружение на фото ключевых точек и вычисление для них дескриптора =ORB
            orb = cv2.ORB_create(nfeatures=15000)

            features1, des1 = orb.detectAndCompute(query_img, None)
            features2, des2 = orb.detectAndCompute(train_img, None)
            
            # сопоставить точки шаблона с точками изображения через Brute-Force Matching
            bf = cv2.BFMatcher(cv2.NORM_HAMMING)

            try:
                matches = bf.knnMatch(des1, des2, k = 2)
            except cv2.error:
                return (0, 0, train_img, 0,0)       

            good = []    
            good_without_lists = []    
            matches = [match for match in matches if len(match) == 2] 

            for m, n in matches:
                if m.distance < 0.82 * n.distance:
                    good.append([m])
                    good_without_lists.append(m)

            # параметры для отображения надписей
            font = cv2.FONT_HERSHEY_SIMPLEX 
            org = (50, 50) 
            fontScale = 0.7
            color = (255, 0, 0) 
            thickness = 2

            if len(good) >= min_matches:
                src_pts = np.float32([features1[m.queryIdx].pt for m in good_without_lists]).reshape(-1, 1, 2)
                dst_pts = np.float32([features2[m.trainIdx].pt for m in good_without_lists]).reshape(-1, 1, 2)

                # найти гомографию используя алгоритм RANSAC, выделить шаблон на изображение рамкой
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,6)

                h, w = query_img.shape[:2]
                pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
                try:
                    dst = cv2.perspectiveTransform(pts, M)
                except cv2.error:
                    return (0, 0, train_img, 0,0)  

                m = np.sort(np.reshape(dst.ravel(),(4,2)), axis = 0)
                train_img = cv2.polylines(train_img, [np.int32(dst)], True, my_color, 2, cv2.LINE_AA)
                
                angle = self.__angle3pt(np.int32(dst)[0][0],np.int32(dst)[1][0],np.int32(dst)[2][0])
                s = abs(tuple(np.int32(m[3]))[0]-tuple(np.int32(m[0]))[0])*abs(tuple(np.int32(m[3]))[1]-tuple(np.int32(m[0]))[1])

                train_img  = cv2.putText(train_img , text, tuple(np.int32(m[0])), font, fontScale, color, thickness, cv2.LINE_AA) 
                return (1, len(good), train_img, s, angle) 
            else:
                flag = 1

        return (0, 0, train_img, 0,0)     

    def __check_signs(self, train_img, pedestrian_crossing_image, traffic_construction_image, traffic_intersection_image, traffic_parking_image, tunnel_image):
        """
        Ищем, какой из знаков на кадре и выводим номер миссии
        """

        detecting_result1 = self.__detect_sign(pedestrian_crossing_image[10:200, :], train_img.copy(), 45, (255,242,0), "pedestrian_crossing")
        detecting_result2 = self.__detect_sign(traffic_construction_image[:250, :], train_img.copy(),  45, (255,242,0), "traffic_construction")
        detecting_result3 = self.__detect_sign(traffic_intersection_image[:260, :], train_img.copy(), 45, (255,242,0), "traffic_intersection")
        detecting_result4 = self.__detect_sign(traffic_parking_image[:260, :], train_img.copy(), 45, (255,242,0), "traffic_parking")
        detecting_result5 = self.__detect_sign(tunnel_image[50:280, 20:280], train_img.copy(), 45, (255,242,0), "tunnel")

        results = [detecting_result1, detecting_result2, detecting_result3, detecting_result4, detecting_result5]

        if sum([i[1] for i in results]) == 0:
            return (0, train_img)
        else:
            # выбираем ту миссию, у которой больше мэтчей, площадь выделенной формы и углы формы +- прямые
            best_result = results[np.argmax([i[1] for i in results])]
            if best_result[3] > 10000 and best_result[4] >= 80 and best_result[4] <= 100:
                return (np.argmax([i[1] for i in results]) + 1, best_result[2])
            return (0, train_img)
        
    def __check_direction(self, train_img, traffic_left_image, traffic_right_image):
        """
        Ищем, какой из знаков на кадре и выводим номер миссии
        """

        detecting_result1 = self.__detect_sign(traffic_left_image, train_img.copy(), 45, (255,242,0), "left")
        detecting_result2 = self.__detect_sign(traffic_right_image, train_img.copy(),  45, (255,242,0), "right")

        results = [detecting_result1, detecting_result2]

        if sum([i[1] for i in results]) == 0:
            return (0, train_img)
        else:
            # выбираем ту миссию, у которой больше мэтчей, площадь выделенной формы и углы формы +- прямые
            best_result = results[np.argmax([i[1] for i in results])]
            #if best_result[3] > 10000 and best_result[4] >= 80 and best_result[4] <= 100:
            return (np.argmax([i[1] for i in results]) + 1, best_result[2])
            #return (0, train_img)
    
    def _callback_finder(self, msg : Image):
        cvImg = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
        cvImg = cv2.cvtColor(cvImg, cv2.COLOR_BGR2RGB)

        if not (cvImg is None):
            ru_missions = ["OnlyDriving", "PedestrianCrossing", "TrafficConstruction", "TrafficIntersection","TrafficParking","Tunnel"]
            ru_intersect_missions = ["looking for sign", "turn left", "turn right"]

            if self.switch_camera:
                mission, train_img = self.__check_signs(cvImg[:240, :200], self.pedestrian_crossing_image, self.traffic_construction_image, self.traffic_intersection_image, self.traffic_parking_image, self.tunnel_image)
            else:
                mission, train_img = self.__check_signs(cvImg[:240, 400:], self.pedestrian_crossing_image, self.traffic_construction_image, self.traffic_intersection_image, self.traffic_parking_image, self.tunnel_image)
                
            if mission != 0 and ru_missions[mission] not in self._ready_missions:
                self._missions_array.append(ru_missions[mission])
                
            if mission == 0 and len(self._missions_array) != 0 and (self._missions_array.count(self._missions_array[0]) >= 2 or (self._missions_array.count(self._missions_array[0]) == 1 and self._missions_array[0] == "TrafficIntersection")):
                if self._missions_array[0] in self._ready_missions:
                    self._missions_array = []
                else:
                    msg = String()
                    msg.data = self._missions_array[0]
                    if msg.data == "PedestrianCrossing":
                        self.switch_camera = True
                    self._publish_command.publish(msg)

                    self._ready_missions.append(self._missions_array[0])
                    self._missions_array = [] 

            # self.get_logger().info(f"ready missions: {self._ready_missions}")
            
            # if DEBUG_LEVEL >= 1:
            #     self.get_logger().info(f"Миссии: {self._ready_missions}")
            #     if mission != 0 and ru_missions[mission] not in self._ready_missions:
            #         self.get_logger().info(f"Mission: {ru_missions[mission]}")

            #     print("Mission: ", ru_missions[mission])
            #     print("-------------")
            # self.get_logger().info(f"Миссии: {self._ready_missions}")
            # self.get_logger().info(f"missions arr: {self._missions_array}")  
            cv2.imshow("detect_signs", train_img)
            cv2.waitKey(1)

def main():
    rclpy.init()

    DSN = Detect_Signs_Node()

    rclpy.spin(DSN)

    DSN.destroy_node()

    rclpy.shutdown()
