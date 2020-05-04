import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import os
import os.path
from datetime import datetime

# add dependencies and config setup.py

class laser_node(Node):
    def __init__(self):

        # inherit Node properties
        super().__init__('scan_distribution')

        # sub to /scan
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.laser_range = None

        self.timer = self.create_timer(0.1, self.timer_callback)

        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        print("Current Time =", current_time)

        # Setup log file path
        script_dir = os.path.dirname(__file__)
        rel_path = "../logs/"
        self.abs_file_path = os.path.join(script_dir, rel_path) + "laser_log_" + current_time + ".txt"

    def timer_callback(self):
        if self.laser_range != None:
            self.write_to_file()

    def scan_callback(self, data):
        # Store scan data in front on callback
        self.laser_range = data.ranges[0]

    def write_to_file(self):
        # Write to terminal and log file
        self.get_logger().info("\n Object is %f units in front\n " % self.laser_range)
        self.write(str(self.laser_range) +" ")


    def write(self, msg):
        with open(self.abs_file_path, 'a') as myFile:
            myFile.write(msg)

def main(args=None):
    rclpy.init(args=args)

    # Create logs folder if it does not exist yet
    script_dir = os.path.dirname(__file__)
    rel_path = "../logs/"
    abs_file_path = os.path.join(script_dir, rel_path)
    if not os.path.exists(abs_file_path):
        os.mkdir(abs_file_path)

    node = laser_node()
    rclpy.spin(node)

    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()