# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# create camera class
class Camera(Node):

    # constructor
    def __init__(self, name, interval):
        super().__init__(name)
        self.cap = cv2.VideoCapture(0)  # create video camera object
        self.publisher = self.create_publisher(Image, 'image', 10)  # create image publisher
        self.timer = self.create_timer(interval, self.receive_image) # call receive_image at given interval
        self.cv_bridge = CvBridge()

    # receive video frame
    def receive_image(self):
        ret,frame = self.cap.read()
        if ret:
            self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(frame, "bgr8"))




def main(args=None):
    rclpy.init(args=args)
    camera_node = Camera("camera", 0.2)
    rclpy.spin(camera_node)
    camera_node.destroy_node
    rclpy.shutdown()
