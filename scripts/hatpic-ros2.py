#!/usr/bin/env/python
"""
File: hatpic-ros2.py
Authors: Julien Mellet, and Simon Le berre
Date: 2024-07-29
Description: A Python script to use the hatpic device to send some commands for a PX4 drone through ROS2.
"""

import serial
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Wrench

class Haptic(Node):
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=115200, timeout=0.01):
        super().__init__('haptic_node')
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.data_lock = threading.Lock()  # Lock for synchronizing access to shared data
        self.pose_pub = self.create_publisher(Pose, 'haptic_pose', 10)
        self.wrench_sub = self.create_subscription(
            Wrench,
            'wrench_topic',
            self.wrench_callback,
            10
        )
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_data)

        # Connect to the serial port
        self.connect()

        # Start a separate thread for reading from the serial port
        self.reading_thread = threading.Thread(target=self.read_data)
        self.reading_thread.daemon = True
        self.reading_thread.start()

    def connect(self):
        try:
            # Open serial port with minimal buffering
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout,
                inter_byte_timeout=self.timeout
            )
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud rate.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port {self.serial_port}: {e}")
            self.ser = None

    def read_data(self):
        while rclpy.ok():  # Run this loop as long as ROS 2 is running
            if not self.ser:
                continue
            
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read_until(b'o')
                    if data and b'i' in data:
                        data_frame = data.decode('utf-8').strip()
                        parsed_data = self.classification(data_frame)
                        if parsed_data:
                            # Lock the data before updating it
                            with self.data_lock:
                                self.data = parsed_data
            except serial.SerialTimeoutException:
                self.get_logger().warning("Serial read timeout occurred.")
            except Exception as e:
                self.get_logger().error(f"Error reading from serial port: {e}")

    def write(self, x):
        if self.ser:
            try:
                self.ser.write(bytes(x, 'utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")

    def extraction_value(self, chain, start, end):
        start_index = chain.find(start)
        end_index = chain.find(end)
        if start_index != -1 and end_index != -1:
            return chain[start_index + len(start):end_index].strip()
        return None

    def classification(self, data_frame):
        if data_frame.startswith('ia'):
            try:
                # Extract values and ensure they are not None
                data_a = self.extraction_value(data_frame, 'a', 'b')
                data_b = self.extraction_value(data_frame, 'b', 'c')
                data_c = self.extraction_value(data_frame, 'c', 'd')
                data_d = self.extraction_value(data_frame, 'd', 'p')
                pitch = self.extraction_value(data_frame, 'p', 'r')
                roll = self.extraction_value(data_frame, 'r', 'o')
                
                # Convert values to float and handle None cases
                data_a = float(data_a) if data_a is not None else 0.0
                data_b = float(data_b) if data_b is not None else 0.0
                data_c = float(data_c) if data_c is not None else 0.0
                data_d = float(data_d) if data_d is not None else 0.0
                pitch = float(pitch) if pitch is not None else 0.0
                roll = float(roll) if roll is not None else 0.0
                
                return [data_a, data_b, data_c, data_d, pitch, roll]
                
            except ValueError as e:
                self.get_logger().error(f"Error parsing data frame: {e}")
        else:
            self.get_logger().warning("Invalid data frame received")
        return None

    def publish_data(self):
        with self.data_lock:  # Lock the data before accessing it
            if self.data:
                msg = Pose()
                msg.position.x = self.data[0]  # Example mapping
                msg.position.y = self.data[1]  # Example mapping
                msg.position.z = self.data[2]  # Example mapping
                msg.orientation.x = self.data[3]  # Example mapping
                msg.orientation.y = self.data[4]  # Example mapping
                msg.orientation.z = self.data[5]  # Example mapping
                msg.orientation.w = 1.0  # Assuming full quaternion is not provided
                
                self.pose_pub.publish(msg)
                self.get_logger().info(f"Published pose: {self.data}")

    def wrench_callback(self, msg):
        # Extract force and torque from the Wrench message
        data_joy_a = msg.force.x
        data_joy_b = msg.force.y
        data_joy_c = msg.force.z
        data_joy_d = msg.torque.z

        # Construct the data string to send to the joystick
        # Example: ia1250b950c1050d900o
        data_string = f"ia{int(data_joy_a)}b{int(data_joy_b)}c{int(data_joy_c)}d{int(data_joy_d)}o"
        print(data_string)
        #self.write(data_string)

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")

def main(args=None):
    rclpy.init(args=args)
    haptic = Haptic()
    rclpy.spin(haptic)
    haptic.disconnect()
    haptic.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
