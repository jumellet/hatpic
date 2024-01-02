#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS/SCS), and an URT
# Be sure that SCServo(STS/SMS/SCS) properties are already set as %% ID : 1 / Baudnum : 6 (Baudrate : 1000000)
#

from rospy import init_node, loginfo, Publisher, Time, Rate, Subscriber, spin, sleep, wait_for_message, ROSInterruptException
from geometry_msgs.msg import Twist, Wrench, WrenchStamped, Transform, PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import os
import threading
from scipy.spatial.transform import Rotation
from time import time


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from scservo_sdk import *                    # Uses SCServo SDK library

# Control table address
ADDR_SCS_MODE               = 33
ADDR_SCS_TORQUE_ENABLE      = 40
ADDR_SCS_GOAL_ACC           = 41
ADDR_SCS_GOAL_POSITION      = 42
ADDR_SCS_GOAL_SPEED         = 46
ADDR_SCS_PRESENT_POSITION   = 56
ADDR_SCS_PRESENT_SPEED      = 58
ADDR_SCS_PRESENT_TORQUE     = 60

# Control modes
MODE_POSITION               = 0
MODE_WHEEL                  = 1

# Default setting
SCS_ID                      = 1                 # SCServo ID : 1
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE  = 1000        # SCServo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE  = 1500        # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
INIT_POSITION               = 1000
OFFSET_ORIGIN_POSITION      = 1000
SCS_MOVING_STATUS_THRESHOLD = 20          # SCServo moving status threshold
SCS_MOVING_SPEED            = 2000        # SCServo moving speed
SCS_MOVING_ACC              = 200           # SCServo moving acc
protocol_end                = 0           # SCServo bit end(STS/SMS=0, SCS=1)

index = 0
scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]         # Goal position

# Torque control parameters
KP_TORQUE = 14#-1500.0 # Proportional gain
KI_TORQUE = 1    # Integral gain
KI_INTEGRAL = 5#50
KE = 0.40           # Elastic coefficient
K_FORCE_FEEDBACK = 80 #100000 # Coefficient force feedback
KP_MAP_JOYSTICK = 0.00003
ALPHA_LOW_PASS = 0.05

# Control loop variables
torque_setpoint = 0.0    # Set your desired torque value
torque_feedback = 0.0;
integral_term = 0.0
INTEGRAL_LIMIT = 2000
torque_error = 0.0
external_wrench = Wrench()
external_wrench.torque.x = 0
external_wrench.torque.y = 0
external_wrench.torque.z = 0

delta_t = 0.02
tStart = time.time()

# Define a lock to ensure thread safety
lock = threading.Lock()

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = PacketHandler(protocol_end)
    
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


# Write SCServo mode
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_MODE, MODE_POSITION)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# Initialize servo position
scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_POSITION, INIT_POSITION)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# Write SCServo acc
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_ACC, SCS_MOVING_ACC)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# Write SCServo speed
scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_SPEED, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# ROS initializations
init_node('joystick_node', anonymous=True)

#pub_joystick_left = Publisher('/joystick_left', Twist, queue_size=1)
#pub_joystick_right = Publisher('/joystick_right', Twist, queue_size=1)
pub_trajectory = Publisher('/iris/command/trajectory', MultiDOFJointTrajectory, queue_size=1)


#msg_joystick_left = Twist()
#msg_joystick_right = Twist()


msg_trajectory = MultiDOFJointTrajectory()
msg_trajectory_position = Transform()
msg_trajectory_velocity = Twist()
msg_trajectory_acceleration = Twist()

def callback_wrench(data):
    global external_wrench
    try:
        external_wrench.torque.x = (1 - ALPHA_LOW_PASS) * (-data.wrench.force.x*K_FORCE_FEEDBACK) + ALPHA_LOW_PASS * external_wrench.torque.x

    except ValueError:
      pass

def listen_thread():
    Subscriber('/iris/force_sensor', WrenchStamped, callback_wrench)

def break_thread():
    global auto_iteration_flag
    if getch() == chr(0x1b):
        print("Escape key pressed. Exiting loop. \n", end='')
        auto_iteration_flag = False
        

def plot_loop():
    global integral_term, torque_setpoint, auto_iteration_flag
    global msg_trajectory, external_wrench, tStart, delta_t
    global msg_trajectory_position, msg_trajectory_velocity, msg_trajectory_acceleration
    while auto_iteration_flag:
        # Read SCServo present position and speed
        scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCS_ID, ADDR_SCS_PRESENT_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result), end='\r')
        elif scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error), end='\r')

        # Read SCServo present torque
        scs_present_torque, scs_comm_result_2, scs_error_2 = packetHandler.read4ByteTxRx(portHandler, SCS_ID, ADDR_SCS_PRESENT_TORQUE)
        if scs_comm_result_2 != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result_2), end='\r')
        elif scs_error_2 != 0:
            print(packetHandler.getRxPacketError(scs_error_2), end='\r')

        scs_present_position = SCS_LOWORD(scs_present_position_speed)
        scs_present_speed = SCS_HIWORD(scs_present_position_speed)
        scs_present_torque = SCS_LOWORD(scs_present_torque)
        
        
        torque_feedback = SCS_TOHOST(scs_present_torque, 10)
        
        # Providing torque control from troque setpoint
        torque_error = torque_setpoint - torque_feedback
        #integral_term += KI_TORQUE * torque_error

        # Anti-windup - limit the integral term to prevent excessive accumulation
        integral_term = min(max(integral_term, -INTEGRAL_LIMIT), INTEGRAL_LIMIT)

        # Calculate torsional spring
        torque_setpoint = KE * (scs_present_position - OFFSET_ORIGIN_POSITION) + int(external_wrench.torque.x)

        # Calculate speed control output using PI control
        speed_control_output = int(-KP_TORQUE * torque_error + KI_TORQUE * integral_term)
        if speed_control_output > 6000:
            speed_control_output = 6000
        elif speed_control_output < -6000:
            speed_control_output = -6000
        
        speed_control_output = SCS_TOSCS(int(speed_control_output), 15)

        signed_speed = SCS_TOHOST(scs_present_speed, 15)
        processed_speed = SCS_TOSCS(signed_speed, 15)
        # Plot variables
        print("[ID:%03d] PresSpd:%03d integral:%03d ctlSpd:%03d PresTrq:%03d TrqErr:%03d TrqSpt:%03d extTrq:%03d \n" 
                % (SCS_ID, SCS_TOHOST(scs_present_speed, 15), integral_term, SCS_TOHOST(speed_control_output, 15), SCS_TOHOST(scs_present_torque, 10), torque_error, torque_setpoint, external_wrench.torque.x), end='\r')
        
        if (time.time() - tStart) < delta_t:
            pass
        else:
            # Write SCServo speed
            scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_SPEED, speed_control_output)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % packetHandler.getRxPacketError(scs_error))
            tStart = time.time()

        # Publish ROS topics
        yaw = 0.0
        rot = Rotation.from_euler('xyz', [0.0, 0.0, yaw], degrees=False)

        msg_trajectory_position.translation.x += (scs_present_position - OFFSET_ORIGIN_POSITION) * KP_MAP_JOYSTICK
        msg_trajectory_position.translation.y = 0
        msg_trajectory_position.translation.z = 1
        msg_trajectory_position.rotation.x = rot.as_quat()[0]
        msg_trajectory_position.rotation.y = rot.as_quat()[1]
        msg_trajectory_position.rotation.z = rot.as_quat()[2]
        msg_trajectory_position.rotation.w = rot.as_quat()[3]

        point = MultiDOFJointTrajectoryPoint([msg_trajectory_position],
                                             [msg_trajectory_velocity],
                                             [msg_trajectory_acceleration],
                                             Time.now())
        msg_trajectory.points.append(point)


        #msg_trajectory.points.velocities.linear.x = scs_present_position - OFFSET_ORIGIN_POSITION

        #angular.x = scs_present_position - OFFSET_ORIGIN_POSITION

        pub_trajectory.publish(msg_trajectory)



# Main loop
global auto_iteration_flag
auto_iteration_flag = True

while True:
    #print("Press any key to continue! (or press ESC to quit!)\n", end='')
    if getch() == chr(0x1b):
        break

    # Write SCServo goal position
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_POSITION, scs_goal_position[index])
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    # Write SCServo mode
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_MODE, MODE_WHEEL)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    # Create new threads
    second_loop_thread = threading.Thread(target=plot_loop)
    third_loop_thread = threading.Thread(target=break_thread)
    forth_loop_thread = threading.Thread(target=listen_thread)

    # Start loop thread
    second_loop_thread.start()
    third_loop_thread.start()
    forth_loop_thread.start()

    #spin()

    # Wait for loop threads to finish
    second_loop_thread.join()
    third_loop_thread.join()
    forth_loop_thread.join()


scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_TORQUE_ENABLE, 0)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
# Close port
portHandler.closePort()