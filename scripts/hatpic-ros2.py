#!/usr/bin/env python
"""
File: hatpic-ros2.py
Authors: Julien Mellet, and Simon Le berre
Date: 2024-07-29
Description: A Python script to use the hatpic device to send some commands for a PX4 drone through ROS2.
"""

import rclpy
import numpy as np
import serial
import time
from geometry_msgs.msg import WrenchStamped, PoseStamped
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import State, TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode

class Hatpic(Node):

    def __init__(self):
        super().__init__('hatpic_node')

        # Define publishers and subscribers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        # Check address of the port
        self.hatpic_ser = serial.Serial(port='/dev/ttyUSB0',  baudrate=115200, timeout=1)

        # Data from the Hatpic device
        self.data_a = 0
        self.data_b = 0
        self.data_c = 0
        self.data_d = 0
        
        self.roll  = 0
        self.pitch = 0

        # Coefficient of the joystick
        self.k_j = 0.0005
        self.dead_zone = 22
        self.f_max = 350

        # Robot's variables
        self.k_b = 350
        self.state = State()
        self.pose_stamped = PoseStamped()

        # Subscriber
        self.odom_sub = self.create_subscription(
            WrenchStamped,
            '/wrench_estimation',
            self.wrench_feedback_callback,
            qos_profile)

        # Publisher
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        #self.velocity = VehicleOdometry
        #self.angular_velocity = VehicleOdometry
        #self.cmd_thrust = VehicleThrustSetpoint
        #self.cmd_torque = VehicleTorqueSetpoint
        self.vehicle_mass = 2.5
        self.vehicle_inertia = np.array([[0.15, 0, 0],
                                         [0, 0.15, 0],
                                         [0, 0, 0.1]])
        self.ext_lin_acc = np.zeros(3)
        self.ext_ang_acc = np.zeros(3)
        self.int_vec_lin = np.zeros(3)
        self.int_vec_ang = np.zeros(3)
        self.gravity = np.array([0, 0, 9.81])  # Assuming gravity is along the z-axis

        # Create timer for periodic execution
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.k_I_lin = np.array([0.9, 0.9, 0.9])
        self.k_I_ang = np.array([0.9, 0.9, 0.9])

    #def torque_callback(self, msg):
    #    self.cmd_torque = np.array([msg.xyz[0], msg.xyz[1], msg.xyz[2]])
    #    #print(self.cmd_torque)

    def write(self, x):
        self.hatpic_ser.write(bytes(x,'utf-8'))
        time.sleep(0.01)

    def read(self):
        return self.classification(self.get_data())

    def get_data(self):
        data_list = [b'a', b'b', b'c', b'd', b'p' ,b'r' ,b'0', b'-', b'1',b'2', b'3', b'4', b'5', b'6', b'7', b'8', b'9']
        data = self.hatpic_ser.read(1)
        start_timing = time.time()
        while data == None:
            data = self.hatpic_ser.read(1)
            time.sleep(0.01)
            t = time.time()
            if t-start_timing > 2:
                print('no data received: :(')
                break
            
        if data:
            if (isinstance(data, bytes)):
                if (data.decode('utf-8')) == "i":
                    data_frame =  "i"
                    while (data.decode('utf-8')) != "o":
                        data = self.hatpic_ser.read(1)
                        if (isinstance(data, bytes)):
                            if data in data_list:
                                data_frame = data_frame + data.decode('utf-8')
                            else:
                                pass
                        else:
                            pass
                        
                    data_frame = data_frame +"o"
                    return data_frame

    def extraction_value(self, chain, start, end):
        #print(chain)
        start_index = chain.find(start)
        end_index   = chain.find(end)
        if start_index != -1 and end_index != -1:
            return chain[start_index + len(start):end_index]
        else:
            return -1

    def classification(self, data_frame):
        if data_frame != None:
            if data_frame[0:2] == 'ia':
                self.data_a = int(self.extraction_value(data_frame, 'a', 'b'))
                self.data_b = int(self.extraction_value(data_frame, 'b', 'c'))
                self.data_c = int(self.extraction_value(data_frame, 'c', 'd'))
                self.data_d = int(self.extraction_value(data_frame, 'd', 'r'))
                self.roll   = int(self.extraction_value(data_frame, 'r', 'p'))
                self.pitch  = int(self.extraction_value(data_frame, 'p', 'o'))
                data_hatpic = [self.data_a, self.data_b, self.data_c, self.data_d]
                return data_hatpic

    def wrench_feedback_callback(self, msg):
        data_joy_a = msg.wrench.force.x
        data_joy_b = msg.wrench.force.y
        data_joy_c = msg.wrench.force.z
        data_joy_d = msg.wrench.torque.z

        data_joy_a = msg.wrench.force.z
        
        # Apply saturation limit of f_max
        data_joy_a *= self.k_b
        if data_joy_a > self.f_max:
            data_joy_a = self.f_max
        elif data_joy_a < -self.f_max:
            data_joy_a = -self.f_max
        data_joy_a += 1000

        #print("d: ", str(int(data_joy_a)))

        # Data example ia1250b950c1050d900o
        self.write('ia'+ str(int(data_joy_a)) +'b0c0d0o')

    def joystick_processing(self, data):
        data -= 1000
        # Dead zone ensures that 0 velocity is sent
        if (data > self.dead_zone) or (data < -self.dead_zone):
            return data * self.k_j
        else:
            return 0

    def timer_callback(self):
        self.calculate_wrench_estimate()


def main(args=None):
    rclpy.init(args=args)
    hatpic = Hatpic()
    rclpy.spin(hatpic)
    hatpic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
