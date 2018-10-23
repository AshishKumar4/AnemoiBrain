from socket import *
import numpy as np
import cv2
import pickle

class CameraFeed:
    def __init__(self, ip = "", port = 8195, mode = 0):
        # Mode 0 -> Drone is TCP Server, 1 -> Drone is Client
        self.port = port 
        self.ip = ip
        self.s = socket(AF_INET, SOCK_STREAM)
        self.s.connect((ip, port))
    def getStream(self, res = [480, 640, 3]):
        while True:
            p = self.s.recv(res[0]*res[1]*res[2])
            if p == 0:
                break
            img = pickle.loads(p)
            cv2.imshow("Frame", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

