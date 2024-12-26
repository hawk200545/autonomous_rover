#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
import tf2_ros
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('localization')

        self.wheel_radius = 0.04  # 4 cm radius
        self.wheel_base = 0.29    # 29 cm between wheels
        self.encoder_resolution = 1080  # Encoder ticks per revolution

        # State variables
        self.x, self.y = 0.0, 0.0
        self.theta_encoder = 0.0  # Orientation from encoders
        self.theta_imu = 0.0      # Orientation from IMU
        self.theta = 0.0          # Fused orientation
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # Complementary filter weight (0.0 to 1.0)
        self.alpha = 0.98

        # ROS publishers and subscribers
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Int32MultiArray, '/encoder_counts', self.encoder_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

    def encoder_callback(self, msg):
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        # Calculate delta ticks
        delta_left = left_ticks - self.prev_left_ticks
        delta_right = right_ticks - self.prev_right_ticks

        # Log tick deltas
        self.get_logger().debug(f"Delta ticks - Left: {delta_left}, Right: {delta_right}")

        # Update previous tick counts
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        # Compute wheel distances
        dL = (delta_left / self.encoder_resolution) * (2 * math.pi * self.wheel_radius)
        dR = (delta_right / self.encoder_resolution) * (2 * math.pi * self.wheel_radius)

        # Compute linear and angular displacement
        d = (dL + dR) / 2.0
        d_theta_encoder = (dR - dL) / self.wheel_base

        # Log computed displacements
        self.get_logger().debug(f'dL: {dL}, dR: {dR}, d: {d}, d_theta: {d_theta_encoder}')

        # Update pose using encoders
        self.x += d * math.cos(self.theta + d_theta_encoder / 2.0)
        self.y += d * math.sin(self.theta + d_theta_encoder / 2.0)
        self.theta_encoder += d_theta_encoder
        self.theta_encoder = math.atan2(math.sin(self.theta_encoder), math.cos(self.theta_encoder))  # Normalize

        # Fuse encoder and IMU data for orientation
        self.theta = self.alpha * self.theta_encoder + (1 - self.alpha) * self.theta_imu
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalize

        # Publish odometry
        self.publish_odometry()

    def imu_callback(self, msg):
        # Extract yaw (z-axis rotation) from IMU quaternion
        orientation = msg.orientation
        _, _, yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        self.theta_imu = yaw
        self.get_logger().debug(f"IMU Yaw: {self.theta_imu}")

    def publish_odometry(self):
        
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Log position
        self.get_logger().info(f'Position: {self.x}, {self.y}, {self.theta}')

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Publish odometry message
        self.odom_publisher.publish(odom)

        # Publish TF for odom to base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(transform)

        # Publish TF for left_wheel joint
        left_wheel_transform = TransformStamped()
        left_wheel_transform.header.stamp = self.get_clock().now().to_msg()
        left_wheel_transform.header.frame_id = 'base_link'
        left_wheel_transform.child_frame_id = 'left_wheel'
        left_wheel_transform.transform.translation.x = 0.0  # Adjust based on your robot configuration
        left_wheel_transform.transform.translation.y = 0.165  # Adjust based on your robot configuration
        left_wheel_transform.transform.translation.z = 0.0
        left_wheel_transform.transform.rotation.x = 0.0
        left_wheel_transform.transform.rotation.y = 0.0
        left_wheel_transform.transform.rotation.z = 0.0
        left_wheel_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(left_wheel_transform)

        # Publish TF for right_wheel joint
        right_wheel_transform = TransformStamped()
        right_wheel_transform.header.stamp = self.get_clock().now().to_msg()
        right_wheel_transform.header.frame_id = 'base_link'
        right_wheel_transform.child_frame_id = 'right_wheel'
        right_wheel_transform.transform.translation.x = 0.0  # Adjust based on your robot configuration
        right_wheel_transform.transform.translation.y = -0.165  # Adjust based on your robot configuration
        right_wheel_transform.transform.translation.z = 0.0
        right_wheel_transform.transform.rotation.x = 0.0
        right_wheel_transform.transform.rotation.y = 0.0
        right_wheel_transform.transform.rotation.z = 0.0
        right_wheel_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(right_wheel_transform)


        laser_transform = TransformStamped()
        laser_transform.header.stamp = self.get_clock().now().to_msg()
        laser_transform.header.frame_id = "base_link"  # Replace with your parent frame
        laser_transform.child_frame_id = "laser"
        laser_transform.transform.translation.x = 0.1  # Adjust based on the laser's position
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.2  # Height of the laser
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0


    def euler_from_quaternion(self, x, y, z, w):
        # Convert quaternion to Euler angles
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw  # We only care about yaw

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
