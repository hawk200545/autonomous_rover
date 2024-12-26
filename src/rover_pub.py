#/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import serial
import time
import math

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        # Initialize serial communication
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        # ROS publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # ROS subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.wheel_base = 0.29

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities from the message
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate left and right wheel speeds
        left_wheel_speed = linear_velocity - (angular_velocity * self.wheel_base / 2)
        right_wheel_speed = linear_velocity + (angular_velocity * self.wheel_base / 2)

        # Send the wheel speeds to the Arduino
        command = f"SET_VELOCITY,{left_wheel_speed},{right_wheel_speed}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent command: {command.strip()}")
    
    def quaternion_from_euler(self,roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode().strip()
            if line.startswith("ENCODERS"):
                _, left_speed, right_speed = line.split(',')
                self.publish_odometry(float(left_speed), float(right_speed))
            elif line.startswith("IMU"):
                _, gyro_x, gyro_y, gyro_z = line.split(',')
                self.publish_imu(float(gyro_x), float(gyro_y), float(gyro_z))

    def publish_odometry(self, left_speed, right_speed):
        # Update position and heading
        dt = time.time() - self.last_time
        self.last_time = time.time()

        linear_velocity = (left_speed + right_speed) / 2
        angular_velocity = (right_speed - left_speed) / self.wheel_base # Adjust wheelbase

        delta_theta = angular_velocity * dt
        self.theta += delta_theta
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        quat = self.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Twist
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

    def publish_imu(self, gyro_x, gyro_y, gyro_z):
        # Publish IMU data
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'

        imu.angular_velocity.x = gyro_x
        imu.angular_velocity.y = gyro_y
        imu.angular_velocity.z = gyro_z

        self.imu_pub.publish(imu)

    def spin(self):
        while rclpy.ok():
            self.read_serial()
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    node.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
