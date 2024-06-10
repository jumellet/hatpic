#!/usr/bin/env python
"""
File: hatpic-ros1.py
Authors: Julien Mellet, and Simon Le berre
Date: 2024-05-26
Description: A Python script to use the hatpic device to send some commands through ROS1.
"""

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, AttitudeTarget, State
import serial
import time

class Hatpic:
    def __init__(self):
        # Check address of the port
        self.hatpic_ser = serial.Serial(port='/dev/ttyUSB0',  baudrate=115200, timeout=1)

        # Data from the Hatpic device
        self.data_a = 0
        self.data_b = 0
        self.data_c = 0
        self.data_d = 0

        # Coefficient of the joystick
        self.k_j = 0.00033
        self.dead_zone = 22
        self.f_max = 350

        # Robot's variables
        self.state = State()
        self.pose_stamped = PoseStamped()

        # Subscribe to wrenchapplied on the robot body
        self.wrench_sub = rospy.Subscriber("/wrench_estimation", WrenchStamped, self.wrench_feedback_callback)

        # Publish for the robot a new position
        self.cmd_robot_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

        self.timer_period = 0.033  # 30 Hz
        # Timer definition
        self.timer = rospy.Timer(rospy.Duration(self.timer_period), self.cmd_robot)

    def write(self, x):
        self.hatpic_ser.write(bytes(x,'utf-8'))
        time.sleep(0.01)

    def read(self):
        return self.classification(self.get_data())

    def get_data(self):
        data_list = [b'a', b'b', b'c', b'd' ,b'0', b'-', b'1',b'2', b'3', b'4', b'5', b'6', b'7', b'8', b'9']
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
                self.data_d = int(self.extraction_value(data_frame, 'd', 'o'))
                data_hatpic = [self.data_a, self.data_b, self.data_c, self.data_d]
                return data_hatpic
    
    def wrench_feedback_callback(self, msg):
        data_joy_a = msg.wrench.force.x
        data_joy_b = msg.wrench.force.y
        data_joy_c = msg.wrench.force.z
        data_joy_d = msg.wrench.torque.z

        data_joy_a = msg.wrench.force.z
        
        # Apply saturation limit of f_max
        data_joy_a *= 250
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

    def cmd_robot(self, event):
        datas = self.read()
        if datas != None:
            x = self.joystick_processing(datas[0])
            self.pose_stamped.pose.position.x += x
            self.pose_stamped.pose.position.y = 0
            self.pose_stamped.pose.position.z = 1

            self.pose_stamped.pose.orientation.x = 0
            self.pose_stamped.pose.orientation.y = 0
            self.pose_stamped.pose.orientation.z = 0
            self.pose_stamped.pose.orientation.w = 1

            self.cmd_robot_pub.publish(self.pose_stamped)
            #print("x: ", x)

if __name__ == '__main__':
    try:
        rospy.init_node('hatpic_node', anonymous=True)

        hatpic = Hatpic()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass