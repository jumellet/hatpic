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
from geometry_msgs.msg import Twist, Wrench
import os
import threading


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
KP_TORQUE = 50#-1500.0 # Proportional gain
KI_TORQUE = 1    # Integral gain
KI_INTEGRAL = 5#50
KE = 0.40           # Elastic coefficient

# Control loop variables
torque_setpoint = 200.0    # Set your desired torque value
torque_feedback = 0.0;
integral_term = 0.0
INTEGRAL_LIMIT = 2000
torque_error = 0.0
external_wrench = Wrench()
external_wrench.torque.x = 0
external_wrench.torque.y = 0
external_wrench.torque.z = 0

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

def callback_wrench(data):
    global external_wrench
    try:
      external_wrench.torque.x = data.torque.x
    except ValueError:
      pass

# ROS initializations
init_node('joystick_node', anonymous=True)

pub_joystick_left = Publisher('/joystick_left', Twist, queue_size=1)
pub_joystick_right = Publisher('/joystick_right', Twist, queue_size=1)
sub_external_wrench = Subscriber('/external_wrench', Wrench, callback_wrench)

msg_joystick_left = Twist()
msg_joystick_right = Twist()



def break_thread():
    global auto_iteration_flag
    if getch() == chr(0x1b):
        print("Escape key pressed. Exiting loop. \n", end='')
        auto_iteration_flag = False

def plot_loop():
    global integral_term, torque_setpoint, auto_iteration_flag
    global msg_joystick_left, msg_joystick_right, external_wrench
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
        
        # Write SCServo speed
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_SCS_GOAL_SPEED, speed_control_output)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        # Publish ROS topics
        msg_joystick_left.angular.x = scs_present_position - OFFSET_ORIGIN_POSITION

        pub_joystick_left.publish(msg_joystick_left)



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

    # Start loop thread
    second_loop_thread.start()
    third_loop_thread.start()

    # Wait for loop threads to finish
    second_loop_thread.join()
    third_loop_thread.join()


scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_TORQUE_ENABLE, 0)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
# Close port
portHandler.closePort()