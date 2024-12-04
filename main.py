import arm
import time
import camera
import cv2 as cv
import numpy as np



dxlRobot = arm.dxlRobot(DEVICENAME = 'COM15', defSpeed = 100)
dxlRobot.movej([1,2,3,4],[0,0,-90,0])


camera = camera.Camera(camera_index=1)

#dxlRobot.movej([1,2,3,4], [0,-45,-45, 0])
camera.createTrackbars()
while 1:
    circle_center = camera.getCircle()
    #print(circle_center)
    key = cv.waitKey(1) & 0xFF
    if key == ord('k'):
        break

time.sleep(2)

xyz_now = dxlRobot.calculateXYZ(dxlRobot.motorPose()[:3])
print("XYZ now:", xyz_now)
x_new = xyz_now[0]+circle_center[0]
y_new = xyz_now[1]+circle_center[1]
dxlRobot.movep(x_new,y_new,80)
# time.sleep(1)
# dxlRobot.movep(130,50,80)


dxlRobot.close()