import arm
import time
import camera
import cv2 as cv
import numpy as np



dxlRobot = arm.dxlRobot(DEVICENAME = 'COM16', defSpeed = 100)
dxlRobot.movej([1,2,3,4],[0,0,-np.pi/2,-np.pi/2])
# dxlRobot.movej([1,2,3,4],[0,0,0,0])


camera = camera.Camera(camera_index=1)

camera.createTrackbars()

start_time = time.time()

while time.time() - start_time < 20:
    circle_center = camera.getCircle()
    #kprint(circle_center)
    

time.sleep(2)

 
T05 = dxlRobot.calculateT05(dxlRobot.motorPose())
xyz_camera_now = T05[:3,3]
R05 = T05[:3,:3]
print(xyz_camera_now)
#I subtract 20 mm so the camera doesnt stylus doesnt crash
delta_d_camera = circle_center[2] - 4
delta_x_camera = circle_center[0]
delta_y_camera = circle_center[1]
#v5 is the distance vector from the camera to the ring expressed in frame 5 
v5 = np.matrix([[delta_d_camera],
               [delta_y_camera],
               [delta_x_camera]])

xyz_ring = xyz_camera_now + R05*v5
print("xyz_ring: ", xyz_ring)

dxlRobot.movep(xyz_ring[0,0],xyz_ring[1,0],xyz_ring[2,0], np.pi)
# x_new = xyz_now[0]+circle_center[0]
# y_new = xyz_now[1]+circle_center[1]
# print(x_new, y_new)
# dxlRobot.movep(x_new,y_new,80)
# time.sleep(1)

# time.sleep(1)
# dxlRobot.movep(130,-50,80, np.pi)
# xyz_now = dxlRobot.calculateXYZ(dxlRobot.motorPose())
# #xyz_now = dxlRobot.calculateXYZ([0.983, -0.604, -1.553, 0.587])
# #angles = dxlRobot.calculateANG(130,50,120,np.pi)
# print("Motor pose: ", dxlRobot.motorPose())
# #print("Angles: ", angles)
# print("XYZ now: ", xyz_now)


dxlRobot.close()