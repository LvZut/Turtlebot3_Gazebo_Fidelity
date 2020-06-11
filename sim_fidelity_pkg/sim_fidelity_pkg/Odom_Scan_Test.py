import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import os

class Block_node(Node):
    def __init__(self):
        super().__init__('distance_driver')

        # Speed at which to drive
        self.max_speed = 0.22
        self.vel_msg = Twist()

        # Only driving forward (x-axis)
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0

        qos = QoSProfile(depth=10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.front_scan = None

        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)

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

    def timer_callback(self):
        if self.distance == -1:
            self.distance = float(input("Input a distance:"))
            self.reverse = bool("Reverse: ")
        if self.starting_position != None:
            self.drive()

    def drive(self):
        # Make sure callback has been done atleast once
        if self.current_position == None:
            return

        if self.current_distance < self.distance:

            if self.vel_msg.linear.x < self.max_speed and self.distance - self.current_distance > 0.2:
                # Accelerate
                self.vel_msg.linear.x += 0.001
            elif self.distance - self.current_distance < 0.4 and self.vel_msg.linear.x > 0.05:
                # Deccelerate
                self.vel_msg.linear.x -= 0.001

            #Publish the velocity
            self.velocity_pub.publish(self.vel_msg)
            
            # Update distance travelled
            pos_dif = (self.current_position[0] - self.starting_position[0], self.current_position[1] - self.starting_position[1])
            self.current_distance = (pos_dif[0]**2 + pos_dif[1]**2)**0.5
        else:
            # Robot has arrived
            self.vel_msg.linear.x = 0.0

            #Force the robot to stop
            self.velocity_pub.publish(self.vel_msg)
            print("Robot has arrived")

            # Print info
            pos_dif = (self.current_position[0] - self.starting_position[0], self.current_position[1] - self.starting_position[1])

            print("Scan data:\nStarted at %f\nStopped at %f\nDistance: %f" % 
            (self.initial_scan, self.front_scan, self.initial_scan - self.front_scan))
            print("Odometry data:\nStarted at %f, %f\nStopped at %f, %f\nDistance: %f" % 
            (self.starting_position[0], self.starting_position[1], self.current_position[0], self.current_position[1],
            (pos_dif[0]**2 + pos_dif[1]**2)**0.5))
            self.init_vars()

    def init_vars(self):
        # reinit vars
        self.current_position = None
        self.starting_position = None
        self.current_distance = 0
        self.distance = 0

        self.current_position = None
        self.starting_position = None

        self.current_distance = 0
        self.distance = -1
        self.initial_scan = None

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