from typing import Literal
from collections import deque
import os

import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


import cv2
import math
import numpy as np


"""
Уровни дебага
0: нет его
1: выводим данные с камеры
"""
DEBUG_LEVEL : Literal[0, 1] = 1

# Если потерял линию то стараться повернуть к ней?
# или наоборот держаться той линии что осталась, но на каком-то растоянии? (среднем за предыдущие время от этой линии)
# Что-то сделать со скоростями, PID регулятор?


class Detect_Signs_Node(Node):

    def __init__(self):
        super().__init__("Detect_Signs_Node")

        self._find_sign_sub = self.create_subscription(Image, "/color/image", self._callback_finder, 3)
        self._cv_bridge = CvBridge()


    def __angle3pt(self, a, b, c):
        """
        Расчет угла между тремя точками
        a.shape: [x,y]
        """

        ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
        return ang + 360 if ang < 0 else ang


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

        flag, founded = 0, 0
        train_img_copy = train_img.copy()

        while (flag!=1):
            # обнаружение на фото ключевых точек и вычисление для них дескриптора =ORB
            orb = cv2.ORB_create(nfeatures=15000)

            features1, des1 = orb.detectAndCompute(query_img, None)
            features2, des2 = orb.detectAndCompute(train_img_copy, None)
            
            # сопоставить точки шаблона с точками изображения через Brute-Force Matching
            bf = cv2.BFMatcher(cv2.NORM_HAMMING)

            try:
                matches = bf.knnMatch(des1, des2, k = 2)
            except cv2.error:
                return (0, 0, train_img,train_img_copy)     

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
                founded = 1

                src_pts = np.float32([features1[m.queryIdx].pt for m in good_without_lists]).reshape(-1, 1, 2)
                dst_pts = np.float32([features2[m.trainIdx].pt for m in good_without_lists]).reshape(-1, 1, 2)

                # найти гомографию используя алгоритм RANSAC, выделить шаблон на изображение рамкой
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,6)

                h, w = query_img.shape[:2]
                pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
                try:
                    dst = cv2.perspectiveTransform(pts, M)
                except cv2.error:
                    return (0, 0, train_img,train_img_copy)

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
        
    def _callback_finder(self, msg : Image):
        cvImg = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
        cvImg = cv2.cvtColor(cvImg, cv2.COLOR_BGR2RGB)

        if not (cvImg is None):
            pedestrian_crossing_image = cv2.imread("/home/alincnl/template_ws/src/follow_path_code/signs/pedestrian_crossing_sign.png")
            traffic_construction_image = cv2.imread("/home/alincnl/template_ws/src/follow_path_code/signs/traffic_construction.png") 
            traffic_intersection_image = cv2.imread("/home/alincnl/template_ws/src/follow_path_code/signs/traffic_intersection.png") 
            traffic_parking_image = cv2.imread("/home/alincnl/template_ws/src/follow_path_code/signs/traffic_parking.png")
            tunnel_image = cv2.imread("/home/alincnl/template_ws/src/follow_path_code/signs/tunnel.png")

            mission, train_img = self.__check_signs(cvImg[:240, 400:], pedestrian_crossing_image, traffic_construction_image, traffic_intersection_image, traffic_parking_image, tunnel_image)

            if DEBUG_LEVEL == 1:
                print("Миссия: ", mission)
                print("-------------")
                cv2.imshow("detect_signs", train_img)
                cv2.waitKey(1)

def main():
    rclpy.init()

    DSN = Detect_Signs_Node()

    rclpy.spin(DSN)

    DSN.destroy_node()

    rclpy.shutdown()
