from typing import Literal
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

import cv2
import numpy as np

OFFSET_BTW_CENTERS = 5
"""
Уровни дебага
0: нет его
1: выводим данные с перспективы
2: выводим данные с камеры
3: выводим маски слоев
4: машинка не поедет никуда
"""
DEBUG_LEVEL : Literal[0, 1, 2, 3, 4] = 3

# Если потерял линию то стараться повернуть к ней?
# или наоборот держаться той линии что осталась, но на каком-то растоянии? (среднем за предыдущие время от этой линии)
# Что-то сделать со скоростями, PID регулятор?

class Follow_Trace_Node(Node):

    def __init__(self, linear_speed = 0.1, angular_speed=0.2, linear_slow_speed=None):
        super().__init__("Follow_Trace_Node")

        self._robot_Ccamera_sub = self.create_subscription(Image, "/color/image", self._callback_Ccamera, 3)
        self._robot_cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._cv_bridge = CvBridge()

        self._linear_speed = linear_speed
        self.angular_speed = angular_speed
        self._linear_slow_speed = linear_slow_speed

        if self._linear_slow_speed is None:
            self._linear_slow_speed = self._linear_speed / 5

        self.__yellow_prevs = deque(maxlen=10)
        self.__white_prevs  = deque(maxlen=10)
        self.__yellow_prevs.append(0)
        self.__white_prevs.append(0)

    def __warpPerspective(self, cvImg):
        h, w, _ = cvImg.shape
        top_x_offset = 50

        pts1 = np.float32([[0, 480], [w, 480], [top_x_offset, 300], [w-top_x_offset, 300]])
        result_img_width = np.int32(abs(pts1[0][0] - pts1[1][0])) # 760
        result_img_height = np.int32(abs(pts1[0][1] - pts1[2][0])) # 300

        pts2 = np.float32([[0, 0], [result_img_width,0], [0, result_img_height], [result_img_width, result_img_height]])

        M = cv2.getPerspectiveTransform(pts1, pts2)
        dst = cv2.warpPerspective(cvImg, M, (result_img_width, result_img_height))

        if(DEBUG_LEVEL >= 2):
            for pt in pts1:
                cvImg = cv2.rectangle(cvImg, np.int32(pt), np.int32(pt), (255, 0, 0), 5)
            cv2.imshow("orig", cvImg)
        
        return cv2.flip(dst, 0)
    

    def _find_yellow_line(self, perspectiveImg):
        h, w, _ = perspectiveImg.shape
        middle_h = h // 2

        yellow_mask = cv2.inRange(perspectiveImg, (0, 240, 255), (0, 255, 255))
        yellow_mask = cv2.dilate(yellow_mask, np.ones((2, 2)), iterations=4)

        middle_row = yellow_mask[middle_h]
        try:
            first_notYellow = np.int32(np.where(middle_row == 255))[0][-1]
            self.__yellow_prevs.append(first_notYellow)
        except: # Если не смогли найти линию пользуемся последними данными о ней, в надежде что починится само
            first_notYellow = sum(self.__yellow_prevs)//len(self.__yellow_prevs)

        if DEBUG_LEVEL>=3:
            a = cv2.rectangle(yellow_mask, (first_notYellow+5, middle_h), (w, middle_h), 255, 10) # рисуем линию откуда идет конец желтой полосы
            cv2.imshow("img_y", a)
            cv2.waitKey(1)

        return (first_notYellow, middle_h)

    def _find_white_line(self, perspectiveImg):
        h, w, _ = perspectiveImg.shape
        middle_h = h // 2

        white_mask = cv2.inRange(perspectiveImg, (250, 250, 250), (255, 255, 255))

        middle_row = white_mask[middle_h]
        try:
            first_white = np.int32(np.where(middle_row == 255))[0][0]
            self.__white_prevs.append(first_white)
        except: # Если не смогли найти линию пользуемся последними данными о ней, в надежде что починится само
            first_white = sum(self.__white_prevs)//len(self.__white_prevs)
            

        if DEBUG_LEVEL>=3:
            a = cv2.rectangle(white_mask, (first_white-5, middle_h), (0, middle_h), 255, 10) # рисуем линию откуда идет конец белой полосы
            cv2.imshow("img_w", a)
            cv2.waitKey(1)

        return (first_white, middle_h)

    def _callback_Ccamera(self, msg : Image):
        emptyTwist = Twist()
        emptyTwist.linear.x = self._linear_speed

        # обрабатываем изо с камеры
        cvImg = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
        cvImg = cv2.cvtColor(cvImg, cv2.COLOR_RGB2BGR)

        # получаем изо перед колесами
        persective = self.__warpPerspective(cvImg)

        # получаем координаты края желтой линии и белой
        endYellow, hYellow  = self._find_yellow_line(persective)
        startWhite, hWhite = self._find_white_line(persective)

        h, w, _ = persective.shape

        middle_btw_lines = (startWhite + endYellow) // 2

        center_crds = (w//2, hYellow)
        lines_center_crds = (middle_btw_lines, hYellow)

        # Выравниваем наш корабль
        if(abs(center_crds[0] - lines_center_crds[0]) > OFFSET_BTW_CENTERS): # если центры расходятся больше чем нужно
            if DEBUG_LEVEL >= 1:
                self.get_logger().info(f"Rotating dist: {abs(center_crds[0] - lines_center_crds[0])}")
            direction = center_crds[0] - lines_center_crds[0] # центр справа - положительно, центр слева - отрицательно

            emptyTwist.angular.z = self.angular_speed
            emptyTwist.linear.x = self._linear_slow_speed

            if(direction < 0):
                emptyTwist.angular.z = -emptyTwist.angular.z


        if DEBUG_LEVEL >= 1:
            # рисуем точки 
            persective_drawed = cv2.rectangle(persective, center_crds, center_crds, (0, 255, 0), 5) # Центр изо 
            persective_drawed = cv2.rectangle(persective_drawed, lines_center_crds, lines_center_crds, (0, 0, 255), 5) # центр точки между линиями
            # по сути пытаемся соединить центр изо с центром между линиями, т.е. поставить синюю точку на зеленую
            cv2.imshow("img", persective_drawed)
            cv2.waitKey(1)
        
        if DEBUG_LEVEL < 4:
            self._robot_cmd_vel_pub.publish(emptyTwist)



def main():
    rclpy.init()

    FTN = Follow_Trace_Node()

    rclpy.spin(FTN)

    FTN.destroy_node()

    rclpy.shutdown()