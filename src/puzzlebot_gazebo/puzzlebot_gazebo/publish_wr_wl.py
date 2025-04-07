#! /usr/bin/env python3
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data



class PublishWheelSpeeds(Node):

  def __init__(self):
    super().__init__('publish_wr_wl')
    self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
    # Create wr publisher
    self.pub_wr = self.create_publisher(Float32, 'VelocityEncR', qos_profile_sensor_data)
    # Create wr publisher
    self.pub_wl = self.create_publisher(Float32, 'VelocityEncL', qos_profile_sensor_data)

    #Robot parameters
    self.L = 0.191 # wheel separation [m]
    self.r = 0.050 # radius of the wheels in [m]
    self.wr = Float32() # right wheel speed in [rad/s]
    self.wl = Float32() # left wheel speed in [rad/s]
    self.v = 0.0 # linear velocity in [m/s]
    self.w = 0.0 # angular velocity in [rad/s]

    # Call timer_cb function every 0.04 seconds (25Hz)
    self.dt= 0.04
    self.timer = self.create_timer(self.dt, self.timer_cb)

  def timer_cb(self):
    [self.wr.data, self.wl.data] = self.get_wheel_speeds(self.v, self.w)
    self.pub_wr.publish(self.wr)
    self.pub_wl.publish(self.wl)

  def cmd_vel_cb(self, msg):
    #Get the velocity 
    self.get_logger().info('Received cmd_vel: linear.x=%.2f, angular.z=%.2f' % (msg.linear.x, msg.angular.z))
    self.v = msg.linear.x
    self.w = msg.angular.z
    

  def get_wheel_speeds(self, v, w):
    #Get the wheel speeds from robot velocity
    wr = (2*v + w * self.L)/(2*self.r)
    wl = (2*v - w * self.L)/(2*self.r)
    return [wr, wl]



def main(args=None): 
  rclpy.init(args=args) 
  f_p=PublishWheelSpeeds() 
  rclpy.spin(f_p) 
  f_p.destroy_node() 
  rclpy.shutdown() 

     

if __name__ == '__main__': 
    main() 