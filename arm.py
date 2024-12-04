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
ADDR_CW_COMPLIANCE_SLOPE = 28
ADDR_CCW_COMPLIANCE_SLOPE = 29

# Protocol version 
PROTOCOL_VERSION            = 1.0
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 400           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 500            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
DXL_IDS = [1,2,3,4]



# Other parameters for the robot
deg2pos_conversion_const = 3.4132
zeroPos_robot = [515, 535, 510, 510]


logging.basicConfig(level=logging.INFO)

class dxlRobot:
    def __init__(self, DEVICENAME, defSpeed) -> None:
        
        # IMPORTANT VARIABLES AND CONSTANTS
        self.HOME_POSITION = []
        
        
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
            logging.info("Succeeded to change the baudrate")
        else:
            logging.error("Failed to change the baudrate")
            logging.error("Press any key to terminate...")
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
            self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, defSpeed)

    def setSpeed(self, speed):
        # Initializing speed
        for DXL_ID in DXL_IDS:
            self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, speed)
    
    def setSlope(self, slope):
        for DXL_ID in DXL_IDS:
            self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_CW_COMPLIANCE_SLOPE, slope)
            self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_CCW_COMPLIANCE_SLOPE, slope)
    
    def movej(self, joints: list[int], positions: list[float]) -> None:
        """
        INPUTS

        - joints (list[int]): A list of joint IDs to be moved.
        - positions (list[int]): A list of target positions corresponding to each joint.
        
        """
        # Convert degrees to position of the motor
        positions = np.array(positions)
        positions = positions * deg2pos_conversion_const + np.array(zeroPos_robot[:len(joints)])
        positions = np.round(positions).astype(int)
               
        # Write a position for each joint
        for joint, position in zip(joints,positions):
            self.packetHandler.write2ByteTxRx(self.portHandler, joint, ADDR_MX_GOAL_POSITION, position)

        time.sleep(0.001)
         
        
    
    def movep(self, x:float, y:float, z:float) -> None:
        print("Motor pose: ", self.motorPose()[:3])
        xyz_now = self.calculateXYZ(self.motorPose()[:3])
        xyz_goal = np.array([x, y, z])
        print("Robot xyz now: ",xyz_now)
        print("Robot xyz goal: ", xyz_goal)
        distance = np.sqrt((xyz_now[0] - xyz_goal[0])**2 + (xyz_now[1] - xyz_goal[1])**2 + (xyz_now[2] - xyz_goal[2])**2)
        
        print("Distance: ", distance)
        
        vector = xyz_goal - xyz_now
        print("Vector: ",vector)
        
        slices = round(distance)*2
        print("Slices:", slices)
        
        vector_slice = vector/slices
        
        for i in range(slices):    
            next_robot_pose = self.calculateANG(xyz_now[0] + vector_slice[0]*i, xyz_now[1] + vector_slice[1]*i, xyz_now[2] + vector_slice[2]*i)
            #print(next_robot_pose)
            self.movej([1,2,3,4], next_robot_pose)
        
        
         
    def calculateANG(self, x:float, y:float, z:float) -> list:
        """
        Moves robot to a inputed position using inverse kinematics
        """
        
        
        # For inverse kinematics
        d1 = 50
        a2 = 93
        a3 = 93
        a4 = 50
        
        oe = np.array([x, y, z])

        theta1 = np.arctan2(y,x)
        
        r = np.sqrt(x**2 + y**2)
        s = z - d1
        c = np.sqrt(r**2 + s**2)
        
        c3 = (r**2 + s**2 - a2**2 - a3**2) / (2*a2*a3)

        phi1 = np.arccos((a2**2 + c**2 - a3**2)/(2*a2*c))
        phi2 = np.arctan2(s,r)
        theta2 = -(np.pi/2 - phi1 - phi2)
        theta3 = -np.arccos(c3)
        theta4 = -theta2 - theta3 - np.pi/2
        
        THETASrad = ([theta1, theta2, theta3, theta4])
        THETASdeg = np.rad2deg([theta1, theta2, theta3, theta4]).tolist()
        
        return THETASdeg
        

    def motorPose(self) -> list:
        THETAS = []
        for joint in range(1,5):
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, joint, ADDR_MX_PRESENT_POSITION)
            #print(joint, dxl_present_position, dxl_comm_result, dxl_error)
            if dxl_comm_result != COMM_SUCCESS:
                logging.info("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                logging.info("%s" % self.packetHandler.getRxPacketError(dxl_error))
            THETAS.append(dxl_present_position)
            
        THETAS = (THETAS - np.array(zeroPos_robot)) / deg2pos_conversion_const
        
        THETAS = np.deg2rad(THETAS)
            
        return THETAS
    
    def forwardTransfer(self, theta, d, a, alpha) -> np.matrix:
        """rotation translation translation rotation"""
        
        #creating short hand notation for np.cos(x) and np.sin(x)
        #th is theta al is alpha
        c_th = np.cos(theta)
        s_th = np.sin(theta)
        c_al = np.cos(alpha)
        s_al = np.sin(alpha)
        
        rot_z = np.matrix([[c_th, -s_th,  0,   0],
                        [s_th,  c_th,  0,   0],
                        [0,     0,     1,   0],
                        [0,     0,     0,   1]])
        
        trans_ad = np.matrix([[1, 0, 0, a],
                            [0, 1, 0, 0],
                            [0, 0, 1, d],
                            [0, 0, 0, 1]])
        
        rot_x = np.matrix([[1, 0,     0,    0],
                        [0, c_al, -s_al, 0],
                        [0, s_al,  c_al, 0],
                        [0, 0,     0,    1]])
        
        
        return(rot_z * trans_ad * rot_x)
    
    def calculateXYZ(self, angles:list[float]) -> list:
        
        theta1, theta2, theta3 = angles
        
        theta4 = 0
        
        
        DH = np.array([[theta1,          50,  0,    (np.pi/2)],
               [theta2+np.pi/2,  0,  93,    0        ],
               [theta3,          0,  93,    0        ],
               [theta4,          0,  50,    0        ]]).astype(float)


        Tmatrix1 = self.forwardTransfer(DH[0][0],DH[0][1],DH[0][2],DH[0][3])
        Tmatrix2 = self.forwardTransfer(DH[1][0],DH[1][1],DH[1][2],DH[1][3])
        Tmatrix3 = self.forwardTransfer(DH[2][0],DH[2][1],DH[2][2],DH[2][3])
        Tmatrix4 = self.forwardTransfer(DH[3][0],DH[3][1],DH[3][2],DH[3][3])   
        
        
        
        
        T04 = Tmatrix1*Tmatrix2*Tmatrix3
        T04 = T04[:3,-1].flatten()
        T04 = np.round([T04[0,0], T04[0,1], T04[0,2]], 4)
        
        return T04
        

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
            sys.exit()
        
        
    



