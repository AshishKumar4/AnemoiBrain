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

class AirSimControls:
    def __init__(self, ip = "0.0.0.0", portBase = 8300):
        # We Spawn 'n' threads corresponding to number of channels
        self.channels = 6
        self.ports = list()
        self.ip = ip
        #self.channelDescriptors
        for i in range(0, 6):
            self.ports.append(portBase + i)
        return None 
    def ChannelControllers(self, id):
        return None
    def setThrottle(self, val):
        return None 
    def setPitch(self, val):
        return None 
    def setRoll(self, val):
        return None 
    def setYaw(self, val):
        return None 
    def setAux1(self, val):
        return None 
    def setAux2(self, val):
        return None 
    