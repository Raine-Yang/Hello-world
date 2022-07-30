import rclpy
from rclpy.node import Node
import numpy as np
import serial
import time
from interface_doc.msg import Coordinates

# create a serial controller class
class SerialController(Node):


    # constructor
    def __init__(self, name, port = '/dev/ttyUSB0', baud = 38400, bytesize = 8, stopbits = 1, parity = 'N'):
        super().__init__(name)
        # create serial object
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.bytesize = bytesize
        self.ser.stopbits = stopbits
        self.ser.parity = parity
        self.ser.open()
        # create subscription of Coordinates
        self.sub = self.create_subscription(Coordinates, 'coordinates', self.callback, 10)
        # save the previous center coordinates
        self.oldCenter = None

    
    # send serial message
    def callback(self, msg):
        center = (msg.x, msg.y)
        # initialize variable oldCenter
        if oldCenter == None:
            oldCenter = center
        # detect displacement to send serial message
        try:
            if center[0] < oldCenter[0]:
                self.ser.write("3".encode())
            if center[0] > oldCenter[0]:
                self.ser.write("4".encode())
            if center[1] > oldCenter[1]:
                self.ser.write("1".encode())
            if center[1] < oldCenter[1]:
                self.ser.write("2".encode())
            oldCenter = center
        except TypeError:   # no valid coordinates in center
            pass
                
        time.sleep(1)
        self.ser.write("5".encode())
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialController("serial")
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()