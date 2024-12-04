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

        theta1 = np.atan2(yend,xend)
        
        if beta == 0:
            oc = oe - a4*np.transpose(np.array([0,0,1]))
        elif np.abs(beta) == np.pi/2:
            oc = oe - a4*np.transpose(np.array([np.cos(theta1), np.sin(theta1), 0]))
        elif np.abs(beta) == np.pi:
            oc = oe - a4*np.transpose(np.array([0,0,-1]))
        else:
            logging.error("This program can only deal with vertical or horizontal stylus")
        
        x, y, z = oc
        
        
        
        r = np.sqrt(x*x + y*y)
        s = z - d1
        c = np.sqrt(r*r + s*s)
        
        c3 = (r*r + s*s - a2*a2 - a3*a3) / (2*a2*a3)

        s3 = np.sqrt(1-c3**2)

        
        phi1 = np.acos((a2*a2 + c*c - a3*a3)/(2*a2*c))
        phi2 = np.atan2(s,r)
        print("PHI", phi1, phi2)
        theta2 = - (np.pi/2 - phi1 - phi2)
        theta3 = -np.acos(c3)
        theta4 = beta - theta2 - theta3
        
        print("\n",[theta1, theta2, theta3, theta4])
        THETAS = np.rad2deg([theta1, theta2, theta3, theta4]).tolist()
        print(THETAS)
        self.movej([1,2,3,4], THETAS)
        
        #print(self.calculateXYZ(np.deg2rad(THETAS)))

    
    
    def denavitMatrix(self, theta, d, a, alpha) -> np.array:
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        sin_a = np.sin(alpha)
        cos_a = np.cos(alpha)
        A = np.array([[cos_t, -sin_t*cos_a, sin_t*sin_a, a*cos_t],
             [sin_t, cos_t*cos_a, -cos_t*sin_a, a*sin_t],
             [0, sin_a, cos_a, d],
             [0, 0, 0, 1]])
        
        return A
    
    def calculateXYZ(self, angles:list[float]) -> list:
        
        theta1, theta2, theta3, theta4 = angles
        
        
        Tmatrix1 = self.denavitMatrix(theta1, d1, 0, np.pi/2)
        Tmatrix2 = self.denavitMatrix(theta2 + np.pi/2, 0, a2, 0)
        Tmatrix3 = self.denavitMatrix(theta3, 0, a3, 0)
        Tmatrix4 = self.denavitMatrix(theta4, 0, a4, 0)
        
        print("\nT1 = ", Tmatrix1,"\nT2 = ", Tmatrix2, "\nT3 = ", Tmatrix3, "\nT4 = ", Tmatrix4)
        A54 = [[1, 0, 0, -15], [0, 1, 0, 45], [0, 0, 1, 0], [0, 0, 0, 1]]
        
        T05 = Tmatrix1*Tmatrix2*Tmatrix3*Tmatrix4*A54
        print(T05)
        
        # Convert angles from degrees to radians if needed
        theta1 = np.radians(theta1)
        theta2 = np.radians(theta2)
        theta3 = np.radians(theta3)
        theta4 = np.radians(theta4)
        
        # Compute trigonometric functions
        cos1 = np.cos(theta1)
        sin1 = np.sin(theta1)
        cos2 = np.cos(theta2 + np.pi / 2)
        sin2 = np.sin(theta2 + np.pi / 2)
        cos3 = np.cos(theta3)
        sin3 = np.sin(theta3)
        cos4 = np.cos(theta4)
        sin4 = np.sin(theta4)
        
        # Compute x, y, z
        x = (
            a2 * cos2 * cos1 +
            a3 * cos3 * cos2 * cos1 +
            a4 * cos4 * (cos3 * cos2 * cos1 - sin3 * sin2 * cos1) -
            a3 * sin3 * sin2 * cos1 -
            a4 * sin4 * (cos3 * sin2 * cos1 + sin3 * cos2 * cos1)
        )
        y = a2*np.sin(theta2 + theta3 + np.pi/2) + a3*np.cos(theta2) + a4*np.sin(theta2 + theta3 + theta4 +np.pi/2)+a4
        
        
        # y = (
        #     -a2 * sin2 * sin1 +
        #     a3 * (cos3 * (-sin2 * sin1) + sin3 * cos2 * cos1) +
        #     a4 * (
        #         cos4 * (cos3 * (-sin2 * sin1) + sin3 * cos2 * cos1) +
        #         sin4 * (cos3 * cos2 * cos1 - sin3 * (-sin2 * sin1))
        #     )
        # )
        
        z = (
            d1 +
            a2 * sin2 +
            a3 * cos2 * sin3 +
            a3 * sin2 * cos3 +
            a4 * cos4 * (cos2 * sin3 + sin2 * cos3) +
            a4 * sin4 * (cos2 * cos3 - sin2 * sin3)
        )
        
        return [x, y, z]

        
        
    




dxlRobot = dxlRobot()

dxlRobot.movep(50,50,100,-np.pi)

