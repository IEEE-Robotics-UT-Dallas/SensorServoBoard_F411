#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float32MultiArray
import math

class SensorServoNode(Node):
    def __init__(self):
        super().__init__('sensor2servo_node')
        
        self.mag_sub = self.create_subscription(
            MagneticField,
            '/imu/mag',
            self.mag_callback,
            10
        )
        self.telemetry_sub = self.create_subscription(
            Float32MultiArray,
            '/telemetry',
            self.telemetry_callback,
            10
        )
        
        self.servo_pub = self.create_publisher(
            Float32MultiArray,
            '/servo_cmd',
            10
        )
        
        # Current inferred state
        self.current_heading_deg = 90.0
        self.current_lux = 0.0
        
        # Timer to publish servo commands at 20Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Sensor to Servo node started.')

    def mag_callback(self, msg):
        # Calculate heading from x and y magnetic field
        # Assuming typical orientation, this is arctan2(y, x)
        x = msg.magnetic_field.x
        y = msg.magnetic_field.y
        heading_rad = math.atan2(y, x)
        heading_deg = math.degrees(heading_rad)
        
        # Map [-180, 180] to [0, 180] for Servo 0 target angle
        mapped_angle = (heading_deg + 180.0) / 2.0
        self.current_heading_deg = max(0.0, min(180.0, mapped_angle))

    def telemetry_callback(self, msg):
        # According to SESSION_HANDOFF.md, index 8 is ambient light
        if len(msg.data) >= 9:
            self.current_lux = msg.data[8]

    def timer_callback(self):
        msg = Float32MultiArray()
        # Create an array of 5 floats for the servos
        # Servo 0 tracks magnetometer heading (0-180)
        servo0 = self.current_heading_deg
        
        # Servo 1 reacts to light intensity
        # Map 0 lux to 90 deg, and 500+ lux to 180 deg
        servo1 = 90.0 + (self.current_lux / 500.0) * 90.0
        servo1 = max(0.0, min(180.0, servo1))
        
        # Servos 2, 3, 4 stay at 90.0 (neutral) for now
        msg.data = [float(servo0), float(servo1), 90.0, 90.0, 90.0]
        
        self.servo_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
