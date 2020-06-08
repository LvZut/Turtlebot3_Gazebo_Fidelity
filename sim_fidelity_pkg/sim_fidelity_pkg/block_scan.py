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
        self.laser_range90 = None
        self.laser_range180 = None
        self.laser_range270 = None

        self.laser_int = None

        self.timer = self.create_timer(0.1, self.timer_callback)

        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        self.get_logger().info("Current Time = %s" % current_time)

        # Setup log file path
        script_dir = os.path.dirname(__file__)
        rel_path = "../logs/"
        self.abs_file_path = os.path.join(script_dir, rel_path) + "laser_log_" + current_time + ".txt"

        self.write("0_degrees, 90_degrees, 180_degrees, 270_degrees, intensity")

    def timer_callback(self):
        if self.laser_range != None:
            self.write_to_file()

    def scan_callback(self, data):
        # Store scan data in front on callback
        self.laser_range = data.ranges[0]
        self.laser_range90 = data.ranges[90]
        self.laser_range180 = data.ranges[180]
        self.laser_range270 = data.ranges[270]

        self.laser_int = data.intensities[0]
        self.get_logger().info("----------------\n0: %f\n90: %f\n180: %f\n270: %f\n-----------------" % (data.ranges[0], data.ranges[90], data.ranges[180], data.ranges[270]))
        self.get_logger().info("----------------\nIntensity @ 0 degrees: %s\n-----------------" % str(data.intensities[0]))

    def write_to_file(self):
        # Write scan range to log file with corresponding angle
        # self.get_logger().info("\n Object is %f units in front\n " % self.laser_range)
        write_list = [self.laser_range90, self.laser_range180, self.laser_range270, self.laser_int]
        write_str = "\n" + str(self.laser_range)
        for lr in write_list:
            write_str += ", " + str(lr)

        self.write(write_str)


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