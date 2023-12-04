import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class ImageCompensation(Node):
    def __init__(self):
        super().__init__('image_compensation')
        self.declare_parameter('clip_hist_percent', 1.0)

        self.sub_image_type = "raw"  # "compressed" / "raw"
        self.pub_image_type = "raw"  # "compressed" / "raw"

        if self.sub_image_type == "compressed":
            # subscribes compressed image 
            self.sub_image_original = self.create_subscription(CompressedImage, '/color/image/compressed', self.cbImageCompensation, 1)
        elif self.sub_image_type == "raw":
            # subscribes raw image 
            self.sub_image_original = self.create_subscription(Image, '/color/image', self.cbImageCompensation, 1)

        if self.pub_image_type == "compressed":
            # publishes compensated image in compressed type 
            self.pub_image_compensated = self.create_publisher(CompressedImage, '/color/image_output/compressed', 1)
        elif self.pub_image_type == "raw":
            # publishes compensated image in raw type
            self.pub_image_compensated = self.create_publisher(Image, '/color/image_output', 1)

        self.cvBridge = CvBridge()

    def cbImageCompensation(self, msg_img):
        self.clip_hist_percent = self.get_parameter("clip_hist_percent").get_parameter_value().double_value

        if self.sub_image_type == "compressed":
            # converts compressed image to opencv image
            np_image_original = np.frombuffer(msg_img.data, np.uint8)
            cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            # converts raw image to opencv image
            cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")

        cv_image_compensated = np.copy(cv_image_original)

        ## Image compensation based on pseudo histogram equalization
        clip_hist_percent = self.clip_hist_percent
        
        hist_size = 256
        min_gray = 0
        max_gray = 0
        alpha = 0
        beta = 0

        gray = cv2.cvtColor(cv_image_compensated, cv2.COLOR_BGR2GRAY)

        # histogram calculation
        if clip_hist_percent == 0.0:
            min_gray, max_gray, _, _ = cv2.minMaxLoc(gray)
        else:
            hist = cv2.calcHist([gray], [0], None, [hist_size], [0, hist_size])

            accumulator = np.cumsum(hist)

            max = accumulator[hist_size - 1]

            clip_hist_percent *= (max / 100.)
            clip_hist_percent /= 2.

            min_gray = 0
            while accumulator[min_gray] < clip_hist_percent:
                min_gray += 1
            
            max_gray = hist_size - 1
            while accumulator[max_gray] >= (max - clip_hist_percent):
                max_gray -= 1

        input_range = max_gray - min_gray

        alpha = (hist_size - 1) / input_range
        beta = -min_gray * alpha

        cv_image_compensated = cv2.convertScaleAbs(cv_image_compensated, -1, alpha, beta)

        if self.pub_image_type == "compressed":
            # publishes compensated image in compressed type
            self.pub_image_compensated.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_compensated, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes compensated image in raw type
            self.pub_image_compensated.publish(self.cvBridge.cv2_to_imgmsg(cv_image_compensated, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ImageCompensation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
