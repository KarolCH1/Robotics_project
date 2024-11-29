import cv2 as cv
import numpy as np
import logging


class Camera:
    def __init__(self, camera_index):
        self.cap = cv.VideoCapture(camera_index)
        if not self.cap.isOpened():
            logging.error(f"Failed to open camera with index {camera_index}.")
        else:
            logging.info(f"Opened camera with index {camera_index}.")
        (ret, frame) = self.cap.read()
        fps = self.cap.get(cv.CAP_PROP_FPS)
    
    def createTrackbars(self)-> None:
        # Starting values for the trackbars
        minDist = 20
        param1 = 69
        param2 = 39
        minRadius = 80
        maxRadius = 100
        
        hue = 20
        sat = 75
        val = 75

        
        cv.namedWindow("Hough Circles",cv.WINDOW_NORMAL)
        cv.createTrackbar("Min Dist", "Hough Circles", minDist, 100, lambda x: None)
        cv.createTrackbar("Param1", "Hough Circles", param1, 255, lambda x: None)
        cv.createTrackbar("Param2", "Hough Circles", param2, 100, lambda x: None)
        cv.createTrackbar("Min Radius", "Hough Circles", minRadius, 300, lambda x: None)
        cv.createTrackbar("Max Radius", "Hough Circles", maxRadius, 300, lambda x: None)
        
        # cv.createTrackbar("Hue low", "Hough Circles", 0, 255, lambda x: None)
        # cv.createTrackbar("Sat low", "Hough Circles", 0, 255, lambda x: None)
        # cv.createTrackbar("Val low", "Hough Circles", 0, 255, lambda x: None)
        
        # cv.createTrackbar("Hue high", "Hough Circles", 255, 255, lambda x: None)
        # cv.createTrackbar("Sat high", "Hough Circles", 255, 255, lambda x: None)
        # cv.createTrackbar("Val high", "Hough Circles", 255, 255, lambda x: None)
        
        cv.createTrackbar("Thresh lower", "Hough Circles", 1, 255, lambda x: None)
        cv.createTrackbar("Adap block", "Hough Circles", 3, 40, lambda x: None)
        cv.createTrackbar("Const C", "Hough Circles", 2, 10, lambda x: None)
    
    def getCircle(self) -> list:
        (ret, frame) = self.cap.read()
        #fps = self.cap.get(cv.CAP_PROP_FPS)
        
       
        
        minDist = cv.getTrackbarPos("Min Dist", "Hough Circles" )
        param1 = cv.getTrackbarPos("Param1", "Hough Circles")
        param2 = cv.getTrackbarPos("Param2", "Hough Circles")
        minRadius = cv.getTrackbarPos("Min Radius", "Hough Circles")
        maxRadius = cv.getTrackbarPos("Max Radius", "Hough Circles")
        
        # hue_low = cv.getTrackbarPos("Hue low", "Hough Circles")
        # sat_low = cv.getTrackbarPos("Sat low", "Hough Circles")
        # val_low = cv.getTrackbarPos("Val low", "Hough Circles")
        
        # hue_high = cv.getTrackbarPos("Hue high", "Hough Circles")
        # sat_high = cv.getTrackbarPos("Sat high", "Hough Circles")
        # val_high = cv.getTrackbarPos("Val high", "Hough Circles")
        
        threshold_lower = cv.getTrackbarPos("Thresh lower", "Hough Circles")
        
        adaptive_threshold_block = 2*cv.getTrackbarPos("Adap block", "Hough Circles") + 1
        const_c = cv.getTrackbarPos("Const C", "Hough Circles")
        
        # hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
        
        # lower_bound = (hue_low, sat_low, val_low)
        # upper_bound = (hue_high, sat_high, val_high) 
        
        # # Create the mask
        # mask = cv.inRange(hsv, lower_bound, upper_bound)

        # # Apply the mask to the original image
        # result = cv.bitwise_and(hsv, hsv, mask=mask)
        
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        
        #frame_thresh = cv.threshold(frame_gray,threshold_lower,255,cv.THRESH_BINARY)[1]
        frame_thresh = cv.adaptiveThreshold(frame_gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, adaptive_threshold_block,const_c)
        frame_thresh = cv.medianBlur(frame_thresh, 9)
                
        edges = cv.Canny(frame_thresh, 150, 200)
        
        
        
        circles = cv.HoughCircles(edges, cv.HOUGH_GRADIENT, dp=1.2, minDist=minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
        
        x = None
        y = None
        
        if circles is not None:
            # Convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            largest_circle = max(circles, key=lambda r: r[2])
            
            x, y, r = largest_circle
            cv.circle(frame, (x, y), r, (0, 255, 0), 4)
            # Draw the center of the circle
            cv.circle(frame, (x, y), 2, (0, 0, 255), 3)
            
            
        
        cv.imshow('Camera view',frame)
        cv.imshow('HSV image with mask', frame_thresh)
        cv.imshow('Edges', edges)
        cv.waitKey(1)
        
        return [x,y]
        
        
        
        
        
    def update(self):
        pass