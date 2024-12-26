import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import time
import math
from threading import Thread

class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_odom_node')

        # Serial communication setup
        self.serial_port = None
        self.connect_to_serial()

        # ROS publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

        # Robot parameters
        self.wheel_radius = 0.05  # 5 cm
        self.wheel_base = 0.3     # 30 cm (distance between wheels)

        # Threads for serial communication
        self.read_thread = Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()

        # Timeout handling for `cmd_vel`
        self.last_cmd_time = time.time()
        self.cmd_timeout = 2.0
        self.cmd_thread = Thread(target=self.check_cmd_timeout, daemon=True)
        self.cmd_thread.start()

    def connect_to_serial(self):
        while self.serial_port is None:
            try:
                self.serial_port = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
                time.sleep(2)  # Allow Arduino to initialize
                self.get_logger().info("Connected to Arduino via Serial")
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to connect to Arduino: {e}. Retrying...")
                time.sleep(2)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Convert to left and right motor speeds
        left_speed = linear_velocity - (angular_velocity * self.wheel_base / 2.0)
        right_speed = linear_velocity + (angular_velocity * self.wheel_base / 2.0)

        # Scale speeds for ESC (1000 to 2000 range)
        left_pwm = int(1500 + left_speed * 500)
        right_pwm = int(1500 + right_speed * 500)

        # Send the command to Arduino
        self.send_serial_command(left_pwm, right_pwm)

    def send_serial_command(self, left_pwm, right_pwm):
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial connection is not open. Reconnecting...")
            self.connect_to_serial()
            return

        try:
            command = f"{left_pwm}:{right_pwm}\n"
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Sent command to Arduino: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error sending command to Arduino: {e}")

    def read_serial(self):
        while rclpy.ok():
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode().strip()
                    self.get_logger().info(f"Received from Arduino: {line}")

                    if line.startswith("Encoder"):
                        _, left_ticks, right_ticks = line.split(":")
                        self.publish_odometry(int(left_ticks), int(right_ticks))
                    else:
                        self.get_logger().warn(f"Unexpected serial message: {line}")
            except Exception as e:
                self.get_logger().error(f"Error reading from Arduino: {e}")
                self.connect_to_serial()

    def check_cmd_timeout(self):
        while rclpy.ok():
            if time.time() - self.last_cmd_time > self.cmd_timeout:
                self.get_logger().warn("Command timeout. Stopping motors.")
                self.send_serial_command(1500, 1500)  # Neutral PWM
            time.sleep(0.1)

    def publish_odometry(self, left_ticks, right_ticks):
        # Compute odometry based on encoder ticks
        dt = time.time() - self.last_time
        self.last_time = time.time()

        left_distance = (left_ticks / 4096) * (2 * math.pi * self.wheel_radius)  # Assuming 4096 ticks/rev
        right_distance = (right_ticks / 4096) * (2 * math.pi * self.wheel_radius)

        linear_velocity = (left_distance + right_distance) / (2 * dt)
        angular_velocity = (right_distance - left_distance) / (self.wheel_base * dt)

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
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Twist
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def cleanup(self):
        self.get_logger().info("Shutting down node. Stopping motors and closing serial port.")
        self.send_serial_command(1500, 1500)  # Stop motors
        if self.serial_port:
            self.serial_port.close()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
