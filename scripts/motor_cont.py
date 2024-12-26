#/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial
import time
from threading import Thread
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Initialize Serial Connection
        self.arduino = None
        while self.arduino is None:
            try:
                self.arduino = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=1)
                time.sleep(2)
                self.get_logger().info("Connected to Arduino via Serial")
                self.reset_encoders()  # Reset encoders after connecting to Arduino
            except serial.SerialException as e:
                self.get_logger().warn(f"Arduino not connected. Retrying in 2 seconds... {e}")
                time.sleep(2)

        self.command_subscriber = self.create_subscription(Int32MultiArray, '/cmd_vel_raw', self.process_command_callback, 10)
        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.ultrasonic_publisher = self.create_publisher(Float32, '/ultrasonic_distance', 10)
        self.encoder_publisher = self.create_publisher(Int32MultiArray, '/encoder_counts', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)  # IMU publisher

        # Robot parameters
        self.wheel_base = 0.3  # Distance between wheels in meters
        self.wheel_radius = 0.05  # Wheel radius in meters

        # Thread for reading Arduino responses
        self.read_thread = Thread(target=self.read_from_arduino, daemon=True)
        self.read_thread.start()

        self.last_command_time = time.time()
        self.timeout = 2.0
        self.stopping_thread = Thread(target=self.stop_if_timeout, daemon=True)
        self.stopping_thread.start()

    def reset_encoders(self):
        try:
            self.arduino.write(b"RESET\n")  # Send reset command
            time.sleep(1)  # Allow some time for Arduino to process the command
            self.get_logger().info("Encoders reset successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to reset encoders: {e}")

    def process_command_callback(self, msg):
        self.last_command_time = time.time()
        speed_left = msg.data[0]
        speed_right = msg.data[1]
        self.send_command_to_arduino(speed_left, speed_right)

    def cmd_vel_callback(self, msg):
        self.last_command_time = time.time()
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate left and right wheel speeds
        left_speed = linear_velocity - (angular_velocity * self.wheel_base / 2)
        right_speed = linear_velocity + (angular_velocity * self.wheel_base / 2)

        # Scale speeds for ESC (1000 to 2000 range)
        left_pwm = int(1500 + left_speed * 500)
        right_pwm = int(1500 + right_speed * 500)

        # Send the calculated speeds to Arduino
        self.send_command_to_arduino(left_pwm, right_pwm)

    def send_command_to_arduino(self, speed_left, speed_right):
        if not self.arduino.is_open:
            self.get_logger().error("Serial connection closed. Attempting to reconnect...")
            self.reconnect_serial()
            return

        try:
            command = f"{speed_left}:{speed_right}\n"
            self.arduino.write(command.encode())
            time.sleep(0.1)
        except serial.SerialTimeoutException:
            self.get_logger().error("Serial write timeout occurred.")
            self.reconnect_serial()
        except Exception as e:
            self.get_logger().error(f"Error sending command to Arduino: {e}")
            self.reconnect_serial()

    def read_from_arduino(self):
        while rclpy.ok():
            try:
                if self.arduino.in_waiting > 0:
                    response = self.arduino.readline().decode().strip()

                    if response.startswith("Encoder"):
                        _, left, right = response.split(": ")
                        encoder_msg = Int32MultiArray()
                        encoder_msg.data = [int(left), int(right)]
                        self.encoder_publisher.publish(encoder_msg)

                    elif response.startswith("Distance"):
                        _, distance = response.split(": ")
                        distance_msg = Float32()
                        distance_msg.data = float(distance)
                        self.ultrasonic_publisher.publish(distance_msg)

            except Exception as e:
                self.reconnect_serial()
                
    def publish_imu(self, imu_data):
        try:
            # Parse IMU data (assumes format "AX, AY, AZ, GX, GY, GZ")
            ax, ay, az, gx, gy, gz = map(float, imu_data.split(","))
            
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Populate IMU message
            imu_msg.linear_acceleration.x = ax / 16384.0 * 9.81
            imu_msg.linear_acceleration.y = ay / 16384.0 * 9.81
            imu_msg.linear_acceleration.z = az / 16384.0 * 9.81
            imu_msg.angular_velocity.x = gx / 131.0 * math.pi / 180.0
            imu_msg.angular_velocity.y = gy / 131.0 * math.pi / 180.0
            imu_msg.angular_velocity.z = gz / 131.0 * math.pi / 180.0

            self.imu_publisher.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to parse IMU data: {e}")

    def stop_if_timeout(self):
        while rclpy.ok():
            if time.time() - self.last_command_time > self.timeout:
                self.get_logger().warn("No commands received for 2 seconds. Stopping motors.")
                self.send_command_to_arduino(1500, 1500)
            time.sleep(0.5)

    def reconnect_serial(self):
        try:
            self.arduino.close()
        except:
            pass  # Ignore errors during closing

        try:
            self.arduino = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=1)
            time.sleep(2)
            self.get_logger().info("Reconnected to Arduino.")
            self.reset_encoders()
        except Exception as e:
            self.get_logger().error(f"Failed to reconnect to Arduino: {e}")

    def cleanup(self):
        self.get_logger().info("Shutting down motor controller. Sending stop command...")
        self.send_command_to_arduino(1500, 1500)
        time.sleep(0.5)

        if self.arduino and self.arduino.is_open:
            self.arduino.close()
            self.get_logger().info("Arduino connection closed.")


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node shutting down...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
