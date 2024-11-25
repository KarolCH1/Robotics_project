import arm

dxlRobot = arm.dxlRobot()

while 1:
    # ret, frame = cap.read()
    # cv.imshow('camera',frame)
    # Write goal position
    dxlRobot.movej([1,2], [450,450])
    dxlRobot.movej([1,2], [400,400])
    break
    # if cv.waitKey(1) & 0xFF == ord('q'):
    #     break
    

dxlRobot.close()