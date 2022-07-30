import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from interface_doc.msg import Coordinates

# create processor class
class Processor(Node):
    def __init__(self, name):
        super().__init__(name)
        # subscribe video frame
        self.image_subscriber = self.create_subscription(Image, 'image', self.callback, 10)
        # publish coordinates
        self.coordinate_publisher = self.create_publisher(Coordinates, 'coordinates', 10)

        self.cv_bridge = CvBridge()

    

    # find the contour of the image
    #return: the HSV contours 
    def findContour(self, frame):
        # read the color ranges 
        red_min = np.array([0, 128, 46])
        red_max = np.array([5, 255, 255])
        red2_min = np.array([156, 128, 46])
        red2_max = np.array([180, 255, 255])
        yellow_min = np.array([15, 128, 46])
        yellow_max = np.array([50, 255, 255])

        # get the colors in range
        x,y = frame.shape[0:2]
        src = cv2.resize(frame, (int(y / 2), int(x / 2)))
        res = src.copy()
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        mask_red1 = cv2.inRange(hsv, red_min, red_max)
        mask_red2 = cv2.inRange(hsv, red2_min, red2_max)
        mask_yellow = cv2.inRange(hsv, yellow_min, yellow_max)
        mask = cv2.bitwise_or(mask_red1, mask_red2)
        mask = cv2.bitwise_or(mask, mask_yellow)
        res = cv2.bitwise_and(src, src, mask=mask)

        # process the image to reduce noise
        h, w = res.shape[:2]
        blured = cv2.blur(res, (5, 5))
        ret, bright = cv2.threshold(blured, 10, 255, cv2.THRESH_BINARY)
        gray = cv2.cvtColor(bright, cv2.COLOR_BGR2GRAY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        opened = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

        # find the contour
        contours, hierarchy = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    
    # point the centers
    # retrn: the average x and y coordinates of the detected contour
    def findCenter(self, contours):
        centersX = np.array([])
        centersY = np.array([])
        for contour in contours:
            if contour.shape[0] < 150:
                continue

        (x, y, w, h) = cv2.boundingRect(contour)
        centersX = np.append(centersX, int(x + w / 2))
        centersY = np.append(centersY, int(y + h / 2))

        meanX = np.mean(centersX)
        meanY = np.mean(centersY)
        return (meanX, meanY)

    # callback function for receiving topic
    def callback(self, image):
        frame = self.cv_bridge.imgmsg_to_cv2(image, 'bgr8')
        coordinates = self.findCenter(self.findContour(frame))
        msg = Coordinates()
        msg.x = int(coordinates[0])
        msg.y = int(coordinates[1])
        self.coordinate_publisher.publish(msg)  # publish topic to serial


def main(args=None):
    rclpy.init(args=args)
    processor_node = Processor("processor")
    rclpy.spin(processor_node)
    processor_node.destroy_node()
    rclpy.shutdown()
