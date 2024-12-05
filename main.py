import arm
import time
import camera
import cv2 as cv
import numpy as np



dxlRobot = arm.dxlRobot(DEVICENAME = 'COM15', defSpeed = 100)
dxlRobot.movej([1,2,3,4],[0,0,-np.pi/2,-np.pi])


camera = camera.Camera(camera_index=1)

dxlRobot.movej([1,2,3,4], [0,-45,-45, 0])
camera.createTrackbars()
while 1:
    circle_center = camera.getCircle()
    #print(circle_center)
    key = cv.waitKey(1) & 0xFF
    if key == ord('k'):
        break

time.sleep(2)


x_new = xyz_now[0]+circle_center[0]
y_new = xyz_now[1]+circle_center[1]
print(x_new, y_new)
dxlRobot.movep(x_new,y_new,80)
# time.sleep(1)
# dxlRobot.movep(130,50,80, np.pi)
# time.sleep(1)
# dxlRobot.movep(130,-50,80, np.pi)
# xyz_now = dxlRobot.calculateXYZ(dxlRobot.motorPose())
# #xyz_now = dxlRobot.calculateXYZ([0.983, -0.604, -1.553, 0.587])
# #angles = dxlRobot.calculateANG(130,50,120,np.pi)
# print("Motor pose: ", dxlRobot.motorPose())
# #print("Angles: ", angles)
# print("XYZ now: ", xyz_now)


dxlRobot.close()