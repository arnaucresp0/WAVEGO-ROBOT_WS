#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import json

ser = serial.Serial("/dev/ttyS0", 115200, timeout = 1)
dataCMD = json.dumps({'var': "", 'val': 0, 'ip': ""})

# Move forward:
def forward(speed=100):
    dataCMD = json.dumps({'var': "move", 'val': 1})
    ser.write(dataCMD.encode())
    print('robot-forward')

# Move backward:
def backward(speed=100):
    dataCMD = json.dumps({'var': "move", 'val': 5})
    ser.write(dataCMD.encode())
    print('robot-backward')

# Turn left:
def left(speed=100):
    dataCMD = json.dumps({'var': "move", 'val': 2})
    ser.write(dataCMD.encode())
    print('robot-left')

# Turn right:
def right(speed=100):
    dataCMD = json.dumps({'var': "move", 'val': 4})
    ser.write(dataCMD.encode())
    print('robot-right')

# Stop left-right turn:
def stopLR():
    dataCMD = json.dumps({'var': "move", 'val': 6})
    ser.write(dataCMD.encode())
    print('robot-stop')

# Stop forward_backward move
def stopFB():
    dataCMD = json.dumps({'var': "move", 'val': 3})
    ser.write(dataCMD.encode())
    print('robot-stop')


class TeleopRpiNode(Node):
    def __init__(self):
        super().__init__("teleop_rpi_node")
        self.subscriber = self.create_subscription(String, "teleop_commands", self.callback, 10)
        self.quit = False

    def callback(self, msg):
        # Received teleop command from PC, send it to Arduino via serial
        command = msg.data
        self.get_logger().info(command)
        if command == 'w':
            # Move forward:
            forward()
        elif command == 's':
            # Move backward:
            backward()
        elif command == 'a':
            # Turn left:
            left()
        elif command == 'd':
            # Turn right:
            right()
        elif command == 'e':
            # Stop left/right moves:
            stopLR()
        elif command == 'r':
            # Stop back and forward moves
            stopFB()
        elif command == 'q':
            # Exit the program:
            print("Closing the teleop python script.")    
            self.destroy_node()
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopRpiNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
