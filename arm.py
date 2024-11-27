import os
import time
import cv2 as cv
import logging
import numpy as np
import math

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
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED = 32

# Protocol version 
PROTOCOL_VERSION            = 1.0
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM12'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 400           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 500            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
DXL_IDS = [1,2,3,4]


class dxlRobot:
    def __init__(self) -> None:
        # IMPORTANT VARIABLES AND CONSTANTS
        self.HOME_POSITION = []
        self.SEGMENT_LENGTHS = [50, 93, 93] #in mm
        
        
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        # Open port
        if self.portHandler.openPort():
            logging.info("Succeeded to open the port")
        else:
            logging.error("Failed to open the port")
            getch()
            quit()
        
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()    
            
        
        # Enable Dynamixel Torque
        for DXL_ID in DXL_IDS:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            
            if dxl_comm_result != COMM_SUCCESS:
                logging.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                logging.error("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                logging.info("Dynamixel has been successfully connected")
                
        # Initializing speed
        for DXL_ID in DXL_IDS:
            self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 40)


    
    def movej(self, joints: list, positions: list) -> None:
        """
        INPUTS

        - joints (list[int]): A list of joint IDs to be moved.
        - positions (list[int]): A list of target positions corresponding to each joint.
        
        """
        # Create a list of length equal to the number of joints we want to move
        joints_that_reached_positions = [False] * len(joints)
        
        # Write a position for each joint
        for joint, position in zip(joints,positions):
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, joint, ADDR_MX_GOAL_POSITION, position)
            if dxl_comm_result != COMM_SUCCESS:
                logging.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                logging.info("%s" % self.packetHandler.getRxPacketError(dxl_error))
                
        # Going into a loop that breaks when the robot reaches a position
        while 1:
            for i, (joint, position) in enumerate(zip(joints, positions)):
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, joint, ADDR_MX_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    logging.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    logging.info("%s" % self.packetHandler.getRxPacketError(dxl_error))


                if not abs(position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                    joints_that_reached_positions[i] = True
            
            if all(joints_that_reached_positions):
                logging.info("All joints have reached their target positions.")
                break
            else:
                logging.info(joints_that_reached_positions)
                
    def movep(self, x:int, y:int, z:int) -> None:
        """
        Moves robot to a inputed position using inverse kinematics
        """
        theta1 = np.arctan(y,x)
        
        
        
        
        

    def close(self) -> None:
        # Disable Dynamixel Torque
        for DXL_ID in DXL_IDS:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                logging.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                logging.info("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # Close port
            self.portHandler.closePort()
        
        
    



