import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time

import numpy as np
import os
import os.path
from datetime import datetime

class Block_node(Node):
    def __init__(self):
        super().__init__('distance_driver')

        # Speed at which to drive
        self.max_speed = 0.22
        self.acceleration = 0.001
        self.deceleration = 0.0015
        self.reverse = 0
        self.vel_msg = Twist()

        # Only driving forward (x-axis)
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0

        # Set up publishers and subscribers
        qos = QoSProfile(depth=10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.input_sub = self.create_subscription(Float64MultiArray, 'bd_input', self.input_callback, qos)
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.front_scan = None

        # Get current time
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        self.get_logger().info("Current Time = %s" % current_time)

        # Setup log file path
        script_dir = os.path.dirname(__file__)
        rel_path = "../logs/"
        self.abs_file_path = os.path.join(script_dir, rel_path) + "drive_log_" + current_time + ".txt"
        self.write("max_speed, distance, reverse, acceleration, deceleration, odom_posdif, lidar_posdif, true_speed, drive_time")

        self.init_vars()

    def odom_callback(self, data):
        self.current_position = (data.pose.pose.position.x, data.pose.pose.position.y)
        if self.starting_position == None:
            self.starting_position = self.current_position

    def scan_callback(self, data):
        self.front_scan = data.ranges[0]
        if self.initial_scan == None:
            self.initial_scan = self.front_scan

    def input_callback(self, data):
        self.max_speed = data.data[0]
        self.distance = data.data[1]
        self.reverse = data.data[2]
        self.acceleration = data.data[3]
        self.deceleration = data.data[4]

    def timer_callback(self):
        if self.starting_position != None and self.distance != -1:
            if self.start_total_time == None:
                self.start_total_time = time.time()
            self.drive()

    def drive(self):
        # Make sure callback has been done atleast once
        if self.current_position == None:
            return

        if self.current_distance < self.distance:

            if self.vel_msg.linear.x <= self.max_speed and self.distance - self.current_distance > 0.2:
                # Accelerate
                self.vel_msg.linear.x += self.acceleration
            elif self.distance - self.current_distance < 0.4 and self.vel_msg.linear.x > 0.05:

                # Stop odom_measure timer if necessary and not yet done
                if self.start_time != None and self.end_time == None:
                    self.end_time = time.time()
                    self.odom_measure_stop = self.current_position

                # Decelerate
                self.vel_msg.linear.x -= self.deceleration

            # Start timer if max speed is reached
            elif self.vel_msg.linear.x >= self.max_speed:
                if self.start_time == None:
                    self.start_time = time.time()
                    self.odom_measure_start = self.current_position

            #Publish the velocity
            if self.reverse:
                self.vel_msg.linear.x *= -1
                self.velocity_pub.publish(self.vel_msg)
                self.vel_msg.linear.x *= -1
            else:
                self.velocity_pub.publish(self.vel_msg)
            
            # Update distance travelled
            pos_dif = (self.current_position[0] - self.starting_position[0], self.current_position[1] - self.starting_position[1])
            self.current_distance = (pos_dif[0]**2 + pos_dif[1]**2)**0.5
        else:
            # Robot has arrived
            self.vel_msg.linear.x = 0.0

            #Force the robot to stop
            self.velocity_pub.publish(self.vel_msg)
            self.end_total_time = time.time()
            self.get_logger().info("\nRobot has arrived\n")

            # Output info
            pos_dif = (self.current_position[0] - self.starting_position[0], self.current_position[1] - self.starting_position[1])

            self.odom_posdif = (pos_dif[0]**2 + pos_dif[1]**2)**0.5
            self.lidar_posdif = self.initial_scan - self.front_scan

            self.get_logger().info("Scan data:\nStarted at %f\nStopped at %f\nDistance: %f\n" % 
            (self.initial_scan, self.front_scan, self.lidar_posdif))
            self.get_logger().info("Odometry data:\nStarted at %f, %f\nStopped at %f, %f\nDistance: %f\n" % 
            (self.starting_position[0], self.starting_position[1], self.current_position[0], self.current_position[1], self.odom_posdif))

            if self.end_time != None:
                pos_dif_2 = (self.odom_measure_stop[0] - self.odom_measure_start[0], self.odom_measure_stop[1] - self.odom_measure_start[1])
                distance_2 = (pos_dif_2[0]**2 + pos_dif_2[1]**2)**0.5
                drive_time = self.end_time - self.start_time
                self.true_speed = distance_2 / drive_time
                self.get_logger().info("Speed data:\nDrove %f in %f seconds\nSpeed was %f m/s\n\n" % (distance_2, drive_time, self.true_speed))

            self.drive_time = self.end_total_time - self.start_total_time
            self.get_logger().info("Drove a total of %f seconds" % self.drive_time)

            self.write_to_file()

            self.init_vars()

    def init_vars(self):
        # reinit vars
        self.current_position = None
        self.starting_position = None

        self.current_distance = 0
        self.distance = -1
        self.initial_scan = None

        self.odom_measure_start = None
        self.odom_measure_stop = None
        self.odom_posdif = None
        self.lidar_posdif = None

        self.start_time = None
        self.end_time = None
        self.true_speed = None

        self.start_total_time = None
        self.end_total_time = None
        self.drive_time = None

    def write_to_file(self):
        # Write data to log file

        write_list = [self.distance, self.reverse, self.acceleration, self.deceleration, self.odom_posdif, self.lidar_posdif, self.true_speed, self.drive_time]

        write_str = "\n" + str(self.max_speed)
        for col in write_list:
            write_str += ", " + str(col)
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

    node = Block_node()
    rclpy.spin(node)

    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()