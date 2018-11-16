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
        print(client.confirmConnection())
        #self.controls = AirSimControls()
        return 
    def getView(self, type = 1, camera = "0"):
        if type == 1:
            self.camera.show(None)
        else:
            self.camera.showStereo()
    def showLive(self, disp = True, type = 1, camera = "0", delay = 0.0333):
        self.buff = queue.Queue(4096)
        if type == 1:
            self.camThread = self.camera.startStream(buffer = self.buff, disp = True)
            #self.camera.getStream(buffer = self.buff, disp = False)
        else:
            self.camThread = self.camera.startStereoStream(buffer = self.buff, disp = True)
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
        

    