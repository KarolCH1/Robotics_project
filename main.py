import arm
import time
import camera
import cv2 as cv
import numpy as np

dxlRobot = arm.dxlRobot(DEVICENAME = 'COM13')
#camera = camera.Camera(camera_index=1)

#dxlRobot.movej([1,2,3,4], [0,-45,-45, 0])
#camera.createTrackbars()
# while 1:
#     circle_center = camera.getCircle()
#     print(circle_center)
pos = dxlRobot.movep(50,35,75,-np.pi)
print(dxlRobot.motorPose())
print(pos)

dxlRobot.close()