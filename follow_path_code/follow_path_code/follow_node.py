from queue import Queue
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

import cv2
import numpy as np

OFFSET_BTW_CENTERS = 5

class Follow_Trace_Node(Node):

    def __init__(self):
        super().__init__("Follow_Trace_Node")
        input()

        self._robot_Ccamera_sub = self.create_subscription(Image, "/color/image", self._callback_Ccamera, 3)
        self._robot_cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._cv_bridge = CvBridge()

        self.__yellow_prevs = deque(maxlen=10)
        self.__white_prevs  = deque(maxlen=10)

        

    def __warpPerspective(self, cvImg):
        pts1 = np.float32([[0, 480], [761, 480], [180, 300], [615, 300]])
        result_img_width = np.int32(abs(pts1[0][0] - pts1[1][0])) # 760
        result_img_height = np.int32(abs(pts1[0][1] - pts1[2][0])) # 300

        pts2 = np.float32([[0, 0], [result_img_width,0], [0, result_img_height], [result_img_width, result_img_height]])

        M = cv2.getPerspectiveTransform(pts1, pts2)
        dst = cv2.warpPerspective(cvImg, M, (result_img_width, result_img_height))

        return cv2.flip(dst, 0)
    

    def _find_yellow_line(self, perspectiveImg, showIt=False):
        h, w, _ = perspectiveImg.shape
        middle_h = h // 2

        yellow_mask = cv2.inRange(perspectiveImg, (0, 240, 255), (0, 255, 255))
        yellow_mask = cv2.dilate(yellow_mask, np.ones((2, 2)), iterations=4)

        middle_row = yellow_mask[middle_h]
        try:
            first_notYellow = np.int32(np.where(middle_row == 255))[0][-1]
            self.__yellow_prevs.append(first_notYellow)
        except:
            first_notYellow = sum(self.__yellow_prevs)//len(self.__yellow_prevs)

        if showIt:
            # print((first_notYellow, middle_h))
            a = cv2.rectangle(yellow_mask, (first_notYellow+5, middle_h), (w, middle_h), 255, 10) # рисуем линию откуда идет конец желтой полосы
            cv2.imshow("img_y", a)
            cv2.waitKey(1)

        return (first_notYellow, middle_h)

    def _find_white_line(self, perspectiveImg, showIt=False):
        h, w, _ = perspectiveImg.shape
        middle_h = h // 2

        white_mask = cv2.inRange(perspectiveImg, (250, 250, 250), (255, 255, 255))

        middle_row = white_mask[middle_h]
        try:
            first_white = np.int32(np.where(middle_row == 255))[0][0]
            self.__white_prevs.append(first_white)
        except:
            first_white = sum(self.__white_prevs)//len(self.__white_prevs)
            

        if showIt:
            # print((first_white, middle_h))
            a = cv2.rectangle(white_mask, (first_white-5, middle_h), (0, middle_h), 255, 10) # рисуем линию откуда идет конец белой полосы
            cv2.imshow("img_w", a)
            cv2.waitKey(1)

        return (first_white, middle_h)

    def _callback_Ccamera(self, msg : Image):
        emptyTwist = Twist()
        emptyTwist.linear.x = 0.05

        cvImg = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
        cvImg = cv2.cvtColor(cvImg, cv2.COLOR_RGB2BGR)

        persective = self.__warpPerspective(cvImg)

        endYellow, hYellow  = self._find_yellow_line(persective, showIt=True)
        startWhite, hWhite = self._find_white_line(persective, showIt=True)

        h, w, _ = persective.shape

        middle_btw_lines = (startWhite + endYellow) // 2

        center_crds = (w//2, hYellow)
        lines_center_crds = (middle_btw_lines, hYellow)

        persective_drawed = cv2.rectangle(persective, center_crds, center_crds, (0, 255, 0), 5) # Центр изо
        persective_drawed = cv2.rectangle(persective_drawed, lines_center_crds, lines_center_crds, (0, 0, 255), 5) # центр точки между линиями

        if(abs(center_crds[0] - lines_center_crds[0]) > OFFSET_BTW_CENTERS): # если центры расходятся больше чем нужно
            self.get_logger().info(f"Rotating dist: {abs(center_crds[0] - lines_center_crds[0])}")
            direction = center_crds[0] - lines_center_crds[0] # центр справа - положительно, центр слева - отрицательно

            twMsg = Twist()
            # twMsg.linear.x = 0.1
            emptyTwist.angular.z = 0.5

            if(direction < 0):
                emptyTwist.angular.z = -twMsg.angular.z

            # self._robot_cmd_vel_pub.publish(twMsg)


        cv2.imshow("img", persective_drawed)
        cv2.waitKey(1)
        self._robot_cmd_vel_pub.publish(emptyTwist)



def main():
    rclpy.init()

    FTN = Follow_Trace_Node()

    rclpy.spin(FTN)

    FTN.destroy_node()

    rclpy.shutdown()