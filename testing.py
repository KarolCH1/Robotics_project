import os
import time
import cv2 as cv
import logging
import numpy as np
import math




# Other parameters for the robot
deg2pos_conversion_const = 3.4132
zeroPos_robot = [515, 535, 510, 510]

# For inverse kinematics
d1 = 50
a2 = 93
a3 = 93
a4 = 50

logging.basicConfig(level=logging.INFO)

class dxlRobot:
    def __init__(self) -> None:
        pass

    
    def movej(self, joints: list[int], positions: list[float]) -> None:
        """
        INPUTS

        - joints (list[int]): A list of joint IDs to be moved.
        - positions (list[int]): A list of target positions corresponding to each joint.
        
        """
        # Convert degrees to position of the motor
        positions = np.array(positions)
        print("Positions for movej",positions)
        positions = positions * deg2pos_conversion_const + np.array(zeroPos_robot[:len(joints)])
        positions = np.round(positions).astype(int)
        print("Positions for movej",positions)
        # Create a list of length equal to the number of joints we want to move
        
           
    def movep(self, xend:float, yend:float, zend:float, beta:float) -> None:
        """
        Moves robot to a inputed position using inverse kinematics
        """
        # Calculate distance between goal position and present position
        
        
        oe = np.array([xend, yend, zend])

        theta1 = np.arctan2(yend,xend)
        
        if beta == 0:
            oc = oe - a4*np.transpose(np.array([0,0,1]))
        elif np.abs(beta) == np.pi/2:
            oc = oe - a4*np.transpose(np.array([np.cos(theta1), np.sin(theta1), 0]))
        elif np.abs(beta) == np.pi:
            oc = oe - a4*np.transpose(np.array([0,0,-1]))
        else:
            logging.error("This program can only deal with vertical or horizontal stylus")
        
        x, y, z = oc
        
        r = np.sqrt(x**2 + y**2)
        s = z - d1
        c = np.sqrt(r**2 + s**2)
        
        c3 = (r**2 + s**2 - a2**2 - a3**2) / (2*a2*a3)

        s3 = np.sqrt(1-c3**2)

        
        phi1 = np.arccos((a2**2 + c**2 - a3**2)/(2*a2*c))
        phi2 = np.arctan2(s,r)
        theta2 = - (np.pi/2 - phi1 - phi2)
        theta3 = -np.arccos(c3)
        theta4 = beta - theta2 - theta3
        
        
        THETAS = np.rad2deg([theta1, theta2, theta3, theta4]).tolist()
        self.movej([1,2,3,4], THETAS)
        
        print(THETAS)
        print(self.calculateXYZ(THETAS))

    
    
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
        
        theta1, theta2, theta3, theta4 = angles
        
        
        DH = np.array([[theta1,          50,  0,    (np.pi/2)],
               [theta2+np.pi/2,  0,  93,    0        ],
               [theta3,          0,  93,    0        ],
               [theta4,          0,  50,    0        ]]).astype(float)


        Tmatrix1 = self.forwardTransfer(DH[0][0],DH[0][1],DH[0][2],DH[0][3])
        Tmatrix2 = self.forwardTransfer(DH[1][0],DH[1][1],DH[1][2],DH[1][3])
        Tmatrix3 = self.forwardTransfer(DH[2][0],DH[2][1],DH[2][2],DH[2][3])
        Tmatrix4 = self.forwardTransfer(DH[3][0],DH[3][1],DH[3][2],DH[3][3])   
        
        
        
        
        T04 = Tmatrix1*Tmatrix2*Tmatrix3*Tmatrix4
        print(T04)
        
        
        return T04

        
        
    




dxlRobot = dxlRobot()

dxlRobot.movep(50,50,100,-np.pi)

