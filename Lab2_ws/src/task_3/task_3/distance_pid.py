#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('kp', 0.2)
        self.declare_parameter('ki', 0.00001)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('target_distance', 0.35)
        self.declare_parameter('max_speed', 0.15)
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('cut_off_frequency_Hz', 5.0)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.target_distance = self.get_parameter('target_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value


        self.distance = 0.0
        self.previous_distance = 0.0
        self.filtered_distance = 0.0
        self.integral = 0.0
        self.sample_rate = 10.0
        self.cut_off_frequency_Hz = self.get_parameter('cut_off_frequency_Hz').value
        dt = 1.0 / self.sample_rate  
        self.alpha = (2 * np.pi * self.cut_off_frequency_Hz * dt) / (2 * np.pi * self.cut_off_frequency_Hz * dt + 1)  
        self.previous_error = 0.0
        self.max_integral = 0.05
        self.cmd_vel = 0.0

        self.create_timer(1.0, self.update_parameters)
        self.create_timer(dt, self.control_loop)

        self.laser_ranges = []

    def update_parameters(self):
        kp= self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        target_distance = self.get_parameter('target_distance').value
        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        if kp != self.kp or ki != self.ki or kd != self.kd or target_distance != self.target_distance or max_speed != self.max_speed or min_speed != self.min_speed:
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.target_distance = target_distance
            self.max_speed = max_speed
            self.min_speed = min_speed


  

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        self.distance = self.laser_ranges[0]

    def reset_integral(self):
        return 0.0

    def control_loop(self):
        if not self.laser_ranges:
            return 
        self.filtered_distance = self.filtered_distance + self.alpha * (self.distance - self.filtered_distance)
        error = self.filtered_distance - self.target_distance
        integral = np.clip(self.max_integral, -self.max_integral, self.integral + error)
        derivative = (error - self.previous_error)/self.sample_rate
        if self.cmd_vel == self.max_speed and error > 0:
            integral = self.reset_integral()
        output = self.kp * error + self.ki * integral + self.kd * derivative
        output = np.clip(output, self.min_speed, self.max_speed)
        self.cmd_vel = output
        twist = Twist()
        twist.linear.x = output  
        self.cmd_vel_publisher.publish(twist)
        self.previous_error = error

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    try:
        rclpy.spin(pid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        pid_controller.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
