from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
from tello_msgs.srv import TelloAction
import tello_msgs.msg
from tf2_ros import TransformListener,Buffer
import rclpy
from rclpy.node import Node
import math

class MarkerListener(Node):

    def __init__(self):
        super().__init__('marker_listener')

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_input,
            10)
        

        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer,self)
        self.timer = self.create_timer(0.01, self.guide)

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
        self.x_pasterr = 0.0
        self.y_pasterr = 0.0
        self.yaw_pasterr = 0.0
        self.useguide = False

        self.kp_xy = 0.8
        self.ki_xy = 0.0
        self.kd_xy = 0.5
        self.update = False

        self.kp_yaw = 1.0
        self.ki_yaw = 0.01
        self.kd_yaw = 0.1
        self.maxI = 100
    
    def LimitMax(self,sum,max):
        if sum > max:
            sum = max
        elif sum < -max:
            sum = -max
        return sum

    def guide(self):
        try:
            t = self.tfBuffer.lookup_transform('camera_pos','aruco_1',rclpy.time.Time())
            if self.height != t.transform.translation.z:
                self.update = True
            self.x = t.transform.translation.x
            self.y = t.transform.translation.y
            self.height = t.transform.translation.z
            self.yaw = t.transform.rotation.y
        except:
            self.update = False
            pass
            
        #self.get_logger().info('Height: %f' % self.height)
        #self.get_logger().info('x: %f, y: %f, yaw: %f' % (self.x, self.y, self.yaw))
        if self.useguide:

            x_err = -self.x
            y_err = self.y
            yaw_err = self.yaw

            self.x_sumerr += x_err
            self.y_sumerr += y_err
            self.yaw_sumerr += yaw_err

            self.x_sumerr = self.LimitMax(self.x_sumerr, self.maxI)
            self.y_sumerr = self.LimitMax(self.y_sumerr, self.maxI)
            self.yaw_sumerr = self.LimitMax(self.yaw_sumerr, self.maxI)

            real_x = self.LimitMax(self.kp_xy * x_err + self.ki_xy * self.x_sumerr + self.kd_xy * (x_err - self.x_pasterr), 1.0)
            real_y = self.LimitMax(self.kp_xy * y_err + self.ki_xy * self.y_sumerr + self.kd_xy * (y_err - self.y_pasterr), 1.0)
            real_yaw = self.LimitMax(self.kp_yaw * yaw_err + self.ki_yaw * self.yaw_sumerr - self.kd_yaw * (yaw_err - self.yaw_pasterr), 1.0)

            self.x_pasterr = x_err
            self.y_pasterr = y_err
            self.yaw_pasterr = yaw_err

            #self.joydata.linear.x = real_y*math.cos(self.yaw) + real_x*math.sin(self.yaw)
            #self.joydata.linear.y = real_x*math.cos(self.yaw) - real_y*math.sin(self.yaw)
            self.joydata.linear.x = real_y
            self.joydata.linear.y = real_x
            #self.get_logger().info('outx: %f, outy: %f' % (self.joydata.linear.x, self.joydata.linear.y))

            if self.update is not True:
                self.joydata.linear.z = 0.2
            elif self.height > 0.3:
                self.joydata.linear.z = -0.3
            else:
                request = TelloAction.Request()
                request.cmd = 'emergency'
                self.client.call_async(request)

            self.joydata.angular.z = -real_yaw

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
