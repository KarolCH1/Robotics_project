import arm
import time
import camera
import cv2 as cv

#dxlRobot = arm.dxlRobot()
camera = camera.Camera(camera_index=0)

#dxlRobot.movej([1,2], [800,450])
camera.createTrackbars()
while 1:
    circle_center = camera.getCircle()
    print(circle_center)

#dxlRobot.close()