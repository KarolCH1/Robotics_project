import cv2 as cv
import numpy as np
import logging

# Some parameters
cap_diameter = 30.2 #mm
cap_diameter_pix_at_160mm = 73.0
ratio = 160.0*73.0

class Camera:
    def __init__(self, camera_index):
        self.cap = cv.VideoCapture(camera_index)
        if not self.cap.isOpened():
            logging.error(f"Failed to open camera with index {camera_index}.")
        else:
            logging.info(f"Opened camera with index {camera_index}.")
        (ret, frame) = self.cap.read()
        fps = self.cap.get(cv.CAP_PROP_FPS)
        
        self.height, self.width, _ = frame.shape
        print(f"Frame width: {self.width}")
        print(f"Frame height: {self.height}")
    
    def createTrackbars(self)-> None:
        # Starting values for the trackbars
        minDist = 20
        param1 = 69
        param2 = 39
        minRadius = 50
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
        
        cv.createTrackbar("Thresh max", "Hough Circles", 50, 255, lambda x: None)
        cv.createTrackbar("Adap block", "Hough Circles", 3, 40, lambda x: None)
        cv.createTrackbar("Const C", "Hough Circles", 2, 10, lambda x: None)
    
    def getCircle(self) -> list:
        (ret, frame) = self.cap.read()
        
        minDist = cv.getTrackbarPos("Min Dist", "Hough Circles" )
        param1 = cv.getTrackbarPos("Param1", "Hough Circles")
        param2 = cv.getTrackbarPos("Param2", "Hough Circles")
        minRadius = cv.getTrackbarPos("Min Radius", "Hough Circles")
        maxRadius = cv.getTrackbarPos("Max Radius", "Hough Circles")

        
        threshold_max = cv.getTrackbarPos("Thresh max", "Hough Circles")
        
        
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        frame_blur = cv.medianBlur(frame_gray, 9)
        
        frame_thresh = cv.threshold(frame_blur,threshold_max,255,cv.THRESH_BINARY)[1]
    
        
                
        edges = cv.Canny(frame_thresh, 150, 200)
        circles = cv.HoughCircles(edges, cv.HOUGH_GRADIENT, dp=1.2, minDist=minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
        
        x = None
        y = None
        r = None
        distance = None
        xmm = None
        ymm = None
        
        if circles is not None:
            # Convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            largest_circle = max(circles, key=lambda r: r[2])
            
            x, y, r = largest_circle
            cv.circle(frame, (x, y), r, (0, 255, 0), 4)
            # Draw the center of the circle
            cv.circle(frame, (x, y), 2, (0, 0, 255), 3)
            cv.circle(frame, (self.width//2, self.height//2), 2, (255, 0, 0),3)
            
            
            distance = ratio/r
            
            
            xmm = (cap_diameter/(2*r))*(x - self.width/2)
            ymm = -(cap_diameter/(2*r))*(y - self.height/2)
            print("X: ", x, "Y: ", y, "Xmm: ", xmm, "Ymm: ", ymm)
            # Calculate vector [x,y] in mm from the center of the camera
            
            
        
        cv.imshow('Camera view',frame)
        cv.imshow('HSV image with mask', frame_thresh)
        cv.imshow('Edges', edges)
        cv.waitKey(1)
        
        
        
        return [xmm,ymm]
        
        
        
        
        
    def update(self):
        pass