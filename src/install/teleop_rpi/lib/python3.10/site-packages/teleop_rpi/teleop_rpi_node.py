#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json
import logging



# Constants for command values
MOVE_FORWARD = 1
MOVE_LEFT = 2
STOP_FB = 3
MOVE_BACKWARD = 5
MOVE_RIGHT = 4
STOP_LR = 6

class TeleopRpiNode(Node):
    def __init__(self):
        super().__init__("teleop_rpi_node")
        self.subscriber = self.create_subscription(String, "teleop_commands", self.callback, 10)
        self.logger = logging.getLogger("teleop_rpi_node")
        self.logger.info("Robot is ready to receive commands.")
        self.quit = False

        # Open serial connection
        with serial.Serial("/dev/ttyS0", 115200, timeout=1) as ser:
            self.ser = ser
            self.dataCMD = json.dumps({'var': "", 'val': 0, 'ip': ""})

            rclpy.spin(self)

    def callback(self, msg):
        # Received teleop command from PC, send it to Arduino via serial
        command = msg.data
        self.logger.info(command)
        if command == 'w':
            # Move forward:
            self.send_command(MOVE_FORWARD)
        elif command == 's':
            # Move backward:
            self.send_command(MOVE_BACKWARD)
        elif command == 'a':
            # Turn left:
            self.send_command(MOVE_LEFT)
        elif command == 'd':
            # Turn right:
            self.send_command(MOVE_RIGHT)
        elif command == 'e':
            # Stop left/right moves:
            self.send_command(STOP_LR)
        elif command == 'r':
            # Stop back and forward moves
            self.send_command(STOP_FB)
        elif command == 'q':
            # Exit the program:
            self.logger.info("Closing the teleop python script.")
            self.destroy_node()

    def send_command(self, val):
        dataCMD = json.dumps({'var': "move", 'val': val})
        self.ser.write(dataCMD.encode())
        self.logger.info(f"Sent command: {dataCMD}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopRpiNode()
    rclpy.shutdown()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
