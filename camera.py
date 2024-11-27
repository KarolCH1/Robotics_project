import cv2 as cv

import logging



class Camera:
    def __init__(self, camera_index=0):
        self.cap = cv.VideoCapture(camera_index)
        if not self.cap.isOpened():
            logging.error(f"Failed to open camera with index {camera_index}.")
        #(ret, frame) = self.cap.read()
        #fps = self.cap.get(cv.CAP_PROP_FPS)