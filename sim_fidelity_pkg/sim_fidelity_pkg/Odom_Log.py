import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
import numpy as np
import os
import os.path
from datetime import datetime

''' Log the odometry to log file while running. '''

class laser_node(Node):
    def __init__(self):

        # inherit Node properties
        super().__init__('odom_logger')

        # sub to /odom
        qos = QoSProfile(depth=10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos)

        # Init self vars
        self.x_pos = None
        self.y_pos = None
        self.z_pos = None

        # Log refresh rate
        refresh_rate = 0.25
        self.timer = self.create_timer(refresh_rate, self.timer_callback)

        # Get and print current time (for log file name)
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        self.get_logger().info("Current Time = %s" % current_time)

        # Setup log file path
        script_dir = os.path.dirname(__file__)
        rel_path = "../logs/"
        self.abs_file_path = os.path.join(script_dir, rel_path) + "odom_log_" + current_time + ".txt"

        # Write column names
        self.write("x, y, z")

    def timer_callback(self):
        if self.x_pos != None:
            self.write_to_file()

    def odom_callback(self, data):
        # Store odom to self vars on callback
        self.x_pos = data.pose.pose.position.x
        self.y_pos = data.pose.pose.position.y
        self.z_pos = data.pose.pose.position.z

    def write_to_file(self):
        # Write odom data to log file
        self.get_logger().info("Position X %f, Y %f, Z %f" % (self.x_pos, self.y_pos, self.z_pos))
        write_list = [self.y_pos, self.z_pos]
        write_str = "\n" + str(self.x_pos)
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