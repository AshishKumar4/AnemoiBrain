from Controls import *
from Controls.HardControls import *
from Controls.AirSimControls import *
from Sensors import *
from Sensors.CameraFeed import *
import airsim
import queue
import cv2 
import matplotlib.pyplot as plt  
import numpy as np
import time
from threading import Thread

class AirSimDrone:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.camera = AirSimCameraFeed(client = self.client)
        print(self.client.confirmConnection())
        self.client.enableApiControl(True)
        #self.controls = AirSimControls()
        return 
    def ready(self):
        self.client.armDisarm(True)
        self.client.takeoffAsync()
    def getView(self, type = 1, camera = "0", disp = True):
        if type == 1:
            self.camera.show(None)
        else:
            self.camera.showStereo()
    def showLive(self, buffer = queue.Queue(4096), disp = True, type = 1, camera = "0", delay = 0.0333):
        self.buff = buffer
        if type == 1:
            self.camThread = self.camera.startStream(buffer = self.buff, disp = disp)
            #self.camera.getStream(buffer = self.buff, disp = False)
        else:
            self.camThread = self.camera.startStereoStream(buffer = self.buff, disp = disp)
    def streamDisparity(self, disp = True, numDisparities = 16, blockSize = 19):
        self.buff = queue.Queue(4096)
        self.camThread = self.camera.startDisparityStream(buffer = self.buff, disp = disp, numDisparities = numDisparities, blockSize = blockSize)
        """
        while True:
            try:
                h = self.buff.get_nowait()
                gl = cv2.cvtColor(h[0], cv2.COLOR_BGR2GRAY)
                gr = cv2.cvtColor(h[1], cv2.COLOR_BGR2GRAY)
                d = s.compute(gl, gr)
                #d = h
                #d = cv2.hconcat(h)
                cv2.imshow("Frame", d)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                time.sleep(0.0333)
            except Exception as e:
                #print(e)
                pass
        """
    def getDisparity(self, disp = True, numDisparities = 16, blockSize = 19):
        g = self.camera.getDisparity(numDisparities = numDisparities, blockSize = blockSize)
        if(disp):
            plt.imshow(g)
            plt.show()
    def popDisparity(self, disp = True):
        try:
            g = self.buff.get_nowait()
            if(disp):
                plt.imshow(g)
                plt.show()
        except:
            pass
        return g
    def MoveByAngleThrottle(self):
        return

class RealDrone:
    def __init__(self):
        return

class Drone:
    def __init__(self, droneType = AirSimDrone()):
        self.drone = droneType
        return 
    def getView(self, type = 1, camera = "0"):
        self.drone.getView(type = type, camera = camera)
    def showLive(self, disp = True, type = 1, camera = "0", delay = 0.0333):
        self.drone.showLive(disp = disp, type = type, camera = camera, delay = delay)
    def MoveByAngleThrottle(self):
        self.drone.MoveByAngleThrottle()
        

    
