import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class ImageProjection(Node):
    def __init__(self):
        super().__init__('image_projection')

        self.declare_parameters(
            namespace='',
            parameters=[
            ('top_x', 72),
            ('top_y', 4),
            ('bottom_x', 115),
            ('bottom_y', 120),
            ('is_calibrating', False)
        ])

        self.sub_image_type = "raw"        # "compressed" / "raw"
        self.pub_image_type = "raw"        # "compressed" / "raw"

        if self.sub_image_type == "compressed":
            # subscribes compressed image 
            self.sub_image_original = self.create_subscription(CompressedImage, '/color/image/compressed', self.cbImageProjection, 1)
        elif self.sub_image_type == "raw":
            # subscribes raw image 
            self.sub_image_original = self.create_subscription(Image, '/color/image', self.cbImageProjection, 1)

        if self.pub_image_type == "compressed":
            # publishes ground-project image in compressed type 
            self.pub_image_projected = self.create_publisher(CompressedImage, '/color/image_output/compressed', 1)
        elif self.pub_image_type == "raw":
            # publishes ground-project image in raw type 
            self.pub_image_projected = self.create_publisher(Image, '/color/image_output', 1)

        self.calib = self.get_parameter("is_calibrating").get_parameter_value().bool_value
        if self.calib == True:
            if self.pub_image_type == "compressed":
                # publishes calibration image in compressed type 
                self.pub_image_calib = self.create_publisher(CompressedImage, '/color/image_calib/compressed', 1)
            elif self.pub_image_type == "raw":
                # publishes calibration image in raw type 
                self.pub_image_calib = self.create_publisher(Image, '/color/image_calib', 1)

        self.cvBridge = CvBridge()

    def cbImageProjection(self, msg_img):
        self.top_x = self.get_parameter("top_x").get_parameter_value().integer_value
        self.top_y = self.get_parameter("top_y").get_parameter_value().integer_value
        self.bottom_x = self.get_parameter("bottom_x").get_parameter_value().integer_value
        self.bottom_y = self.get_parameter("bottom_y").get_parameter_value().integer_value

        if self.sub_image_type == "compressed":
            # converts compressed image to opencv image
            np_image_original = np.frombuffer(msg_img.data, np.uint8)
            cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            # converts raw image to opencv image
            cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")

        # setting homography variables
        top_x = self.top_x
        top_y = self.top_y
        bottom_x = self.bottom_x
        bottom_y = self.bottom_y

        if self.calib == True:
            # copy original image to use for cablibration
            cv_image_calib = np.copy(cv_image_original)

            # draw lines to help setting homography variables
            cv_image_calib = cv2.line(cv_image_calib, (424 - top_x, 240 - top_y), (424 + top_x, 240 - top_y), (0, 0, 255), 1)
            cv_image_calib = cv2.line(cv_image_calib, (424 - bottom_x, 240 + bottom_y), (424 + bottom_x, 240 + bottom_y), (0, 0, 255), 1)
            cv_image_calib = cv2.line(cv_image_calib, (424 + bottom_x, 240 + bottom_y), (424 + top_x, 240 - top_y), (0, 0, 255), 1)
            cv_image_calib = cv2.line(cv_image_calib, (424 - bottom_x, 240 + bottom_y), (424 - top_x, 240 - top_y), (0, 0, 255), 1)

            if self.pub_image_type == "compressed":
                # publishes calibration image in compressed type
                self.pub_image_calib.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_calib, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes calibration image in raw type
                self.pub_image_calib.publish(self.cvBridge.cv2_to_imgmsg(cv_image_calib, "bgr8"))

        # adding Gaussian blur to the image of original
        cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        ## homography transform process
        # selecting 4 points from the original image
        pts_src = np.array([[424 - top_x, 240 - top_y], [424 + top_x, 240 - top_y], [424 + bottom_x, 240 + bottom_y], [424 - bottom_x, 240 + bottom_y]])

        # selecting 4 points from image that will be transformed
        pts_dst = np.array([[148, 0], [600, 0], [600, 480], [148, 480]])

        # finding homography matrix
        h, status = cv2.findHomography(pts_src, pts_dst)

        # homography process
        cv_image_homography = cv2.warpPerspective(cv_image_original, h, (848, 480))

        # fill the empty space with black triangles on left and right side of bottom
        # triangle1 = np.array([[0, 479], [0, 260], [148, 479]], np.int32)
        # triangle2 = np.array([[847, 479], [847, 260], [647, 479]], np.int32)
        # black = (0, 0, 0)
        # white = (255, 255, 255)
        # cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)

        if self.pub_image_type == "compressed":
            # publishes ground-project image in compressed type
            self.pub_image_projected.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_homography, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes ground-project image in raw type
            self.pub_image_projected.publish(self.cvBridge.cv2_to_imgmsg(cv_image_homography, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = ImageProjection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
