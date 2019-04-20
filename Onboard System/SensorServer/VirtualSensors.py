from Sensors import *
from socket import *

# Basically Manage connections to the Camera and Sensor, to provide raw data for onboard usage as well as connection to the servers for offboard
# and provide APIs to access the camera and sensory data in a reliable Way

class VirtualSensor:
    def __init__(self):
        self.refLocation = None
        return None 

    def getCurrentLocation(self):
        # Use Accelerometer, Gyro, GPS to get current Location
        return None 

    def getRelativeLocation(self, relative = None):
        if relative is None:
            relative = self.refLocation
        return None

    def setReferenceLocation(self, location):
        self.refLocation = location 
        return None 

    def getCurrentVector(self, relative = None):
        
        return None

    def getRelativeVector(self, relative = None):
        if relative is None:
            relative = self.refLocation
        return None

    def getCurrentDirection(self):
        return None 

    def getCameraFrame(self):
        return None 

