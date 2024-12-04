# -*- coding: utf-8 -*-
"""
Created on Fri Nov 29 16:54:51 2024

@author: Sjoer
"""

import numpy as np

def forwardTransfer(theta, d, a, alpha):
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

#defining the display functiom, gets rids of the scientific notation.
def GFG(arr,prec):
    new = np.array_str(arr, precision=prec, suppress_small=True)
    print(new)
    
thetas = [0.983, -0.604, -1.553, 0.587]
theta1 = thetas[0]
theta2 = thetas[1]
theta3 = thetas[2]
theta4 = thetas[3]

      
DH = np.array([[theta1,          50,  0,    (np.pi/2)],
               [theta2+np.pi/2,  0,  93,    0        ],
               [theta3,          0,  93,    0        ],
               [theta4,          0,  50,    0        ]]).astype(float)


Tmatrix1 = forwardTransfer(DH[0][0],DH[0][1],DH[0][2],DH[0][3])
Tmatrix2 = forwardTransfer(DH[1][0],DH[1][1],DH[1][2],DH[1][3])
Tmatrix3 = forwardTransfer(DH[2][0],DH[2][1],DH[2][2],DH[2][3])
Tmatrix4 = forwardTransfer(DH[3][0],DH[3][1],DH[3][2],DH[3][3])   
                              
                                     
T04 = Tmatrix1*Tmatrix2*Tmatrix3*Tmatrix4
GFG(T04, 4)


                            