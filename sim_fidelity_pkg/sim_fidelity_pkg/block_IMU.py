import rclpy
from rclpy.node import Node
import os
import os.path
import numpy as np
import math
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


class imu_node(Node):
    def __init__(self):

        # inherit Node properties
        super().__init__('angler')

        self.orientation = None
        self.starting_orientation = None
        self.rot_dest = None
        self.angular_dest = None
        self.current_rotation = 0

        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile_sensor_data)
        self.rot_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.005, self.timer_callback)
        
        self.rot_speed = 2.0 # 2.84 max
        self.cur_speed = 0.0
        self.rem_thresh = 0.05
        self.vel_msg = Twist()

        # Only rotating horizontally (z-axis)
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0
        
        
        print("everything is set up")


    def timer_callback(self):
        if self.orientation != None:
            if self.rot_dest == None:
                rotation = float(input("Input rotation [-180, 180]:"))
                self.get_destination(rotation)
            self.rotate_robot()



    def rotate_robot(self):
            self.euler_coords = self.euler_from_quaternion(self.orientation)
            remainder = max(float((self.euler_coords)[2]) - self.rot_dest, self.rot_dest - float((self.euler_coords)[2]))

            if remainder > self.rem_thresh:
                if self.cur_speed < self.rot_speed and remainder > 0.5:
                    self.cur_speed += 0.0005
                elif self.cur_speed > 0.5 and remainder <= 0.5:
                    self.cur_speed -= 0.0005

                if self.direction == 1:
                    self.vel_msg.angular.z = self.cur_speed
                else:
                    self.vel_msg.angular.z = -self.cur_speed

                self.rot_pub.publish(self.vel_msg)

            else:
                print("rotation achieved")
                self.vel_msg.angular.z = 0.0
                self.rot_pub.publish(self.vel_msg)
                self.rot_dest = None
                self.angular_dest = None
                self.starting_orientation = None
                self.rem_thresh = 0.1

    def imu_callback(self, data):
        self.orientation = data.orientation
        if self.starting_orientation == None:
            self.starting_orientation = (self.euler_from_quaternion(self.orientation))[2]


    def get_destination(self, destination):
        self.angular_dest = math.radians(destination)
        self.rot_dest = self.starting_orientation + self.angular_dest
        if self.rot_dest > math.pi:
            self.rot_dest -= 2*math.pi
        elif self.rot_dest < -math.pi:
            self.rot_dest += 2*math.pi
        print("from %f to %f" % (self.starting_orientation, self.rot_dest))

        # Increase threshold if dest is close to pi
        if self.rot_dest > 3 or self.rot_dest < -3:
            self.rem_thresh = 0.1

        if self.angular_dest > 0:
            self.direction = 1
        else:
            self.direction = -1

    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w*y - z*x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)

    node = imu_node()
    rclpy.spin(node)

    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





