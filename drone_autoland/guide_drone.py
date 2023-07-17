# SPDX-FileCopyrightText: 2023 Aso Hidetoshi asouhide2002@gmail.com
# SPDX-License-Identifier: BSD-3-Clause

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
from tello_msgs.srv import TelloAction
import tello_msgs.msg
from ros2_aruco_interfaces.msg import ArucoMarkers
import rclpy
from rclpy.node import Node

class MarkerListener(Node):

    def __init__(self):
        super().__init__('marker_listener')

        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.marker_detect,
            10)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_input,
            10)
        self.timer = self.create_timer(0.1, self.guide)

        self.client = self.create_client(TelloAction, 'tello_action')
        self.rcpub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.height = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.joydata = Twist()
        self.joydata.linear.x = 0.0
        self.joydata.linear.y = 0.0
        self.joydata.linear.z = 0.0
        self.joydata.angular.z = 0.0

        self.x_sumerr = 0.0
        self.y_sumerr = 0.0
        self.yaw_sumerr = 0.0
        self.useguide = False

        self.kp_xy = 10.0
        self.ki_xy = 0.01
        self.kp_yaw = 10.0
        self.ki_yaw = 0.01
        self.maxI = 100
    
    def marker_detect(self, msg):
        readid = 0
        for marker_id in msg.marker_ids:
            marker_pos = msg.poses[readid]
            if marker_id == 1:
                self.height = marker_pos.position.z
                self.x = marker_pos.position.x
                self.y = marker_pos.position.y
                self.yaw = marker_pos.orientation.z
            readid += 1
    
    def LimitMax(self,sum,max):
        if sum > max:
            sum = max
        elif sum < -max:
            sum = -max
        return sum

    def guide(self):
        if self.useguide:
            x_err = self.x
            y_err = self.y
            yaw_err = 0.0 - self.yaw

            self.x_sumerr += x_err
            self.y_sumerr += y_err
            self.yaw_sumerr += yaw_err

            self.x_sumerr = self.LimitMax(self.x_sumerr, self.maxI)
            self.y_sumerr = self.LimitMax(self.y_sumerr, self.maxI)
            self.yaw_sumerr = self.LimitMax(self.yaw_sumerr, self.maxI)

            self.joydata.linear.x = self.LimitMax(self.kp_xy * x_err + self.ki_xy * self.x_sumerr, 1.0)
            self.joydata.linear.y = self.LimitMax(self.kp_xy * y_err + self.ki_xy * self.y_sumerr, 1.0)

            if self.height > 0.3:
                self.joydata.linear.z = -0.5
            else:
                self.joydata.linear.z = 0.0

            self.joydata.angular.z = self.LimitMax(self.kp_yaw * yaw_err + self.ki_yaw * self.yaw_sumerr, 1.0)


            self.get_logger().info('Xerr: "%f"' % x_err)
            self.get_logger().info('Yerr: "%f"' % y_err)
            self.get_logger().info('Yawerr: "%f"' % yaw_err)
        else:
            self.x_sumerr = 0.0
            self.y_sumerr = 0.0
            self.yaw_sumerr = 0.0

        self.rcpub.publish(self.joydata)

        #self.get_logger().info('Joydata: "%f"' % self.joydata.linear.x)
        #self.get_logger().info('Joydata: "%f"' % self.joydata.linear.y)
        #self.get_logger().info('Joydata: "%f"' % self.joydata.linear.z)
        #self.get_logger().info('Joydata: "%f"' % self.joydata.angular.z)
    
    def joy_input(self,joy_msg):
        if joy_msg.buttons[1]:
            request = TelloAction.Request()
            request.cmd = 'takeoff'
            self.client.call_async(request)
        elif joy_msg.buttons[0]:
            request = TelloAction.Request()
            request.cmd = 'land'
            self.client.call_async(request)
        elif joy_msg.buttons[2]:
            self.useguide = True
        elif joy_msg.buttons[3]:
            self.useguide = False
        elif joy_msg.buttons[5]:
            request = TelloAction.Request()
            request.cmd = 'emergency'
            self.client.call_async(request)
        else:
            self.joydata.linear.x = joy_msg.axes[1]
            self.joydata.linear.y = joy_msg.axes[3]
            self.joydata.linear.z = joy_msg.axes[4]
            self.joydata.angular.z = joy_msg.axes[0]

def main():
    rclpy.init()
    node = MarkerListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
