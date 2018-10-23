from socket import *
import numpy as np
import cv2

class CameraFeed:
    def __init__(self, ip = "", port = 8195, mode = 0):
        # Mode 0 -> Drone is TCP Server, 1 -> Drone is Client
        self.port = port 
        self.ip = ip
        self.s = socket(AF_INET, SOCK_STREAM)
        self.s.connect((ip, port))
        
    def getStream(self):
        while True:
            p = self.s.recv(8194)
            if p == 0:
                break
            img = np.fromstring(p)
            cv2.imshow("Frame", img)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

