from socket import *
import numpy as np
import cv2
import pickle
import struct

class CameraFeed:
    def __init__(self, ip = "", port = 8195, mode = 0):
        # Mode 0 -> Drone is TCP Server, 1 -> Drone is Client
        self.port = port 
        self.ip = ip
    def getStream(self, res = [480, 640, 3]):
        self.s = socket(AF_INET, SOCK_STREAM)
        self.s.connect((ip, port))
        g = self.s.recv(1024) # Recieve the size of the sample image
        s = struct.unpack("L", g)[0]
        p = b''
        k = b''
        while True:
            i = len(k)
            p = k
            while i < s:
                p += self.s.recv(8192)
                i += 8192
            if p == 0:
                break
            k = p[s:]
            p = p[:s]
            img = pickle.loads(p)
            cv2.imshow("Frame", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

