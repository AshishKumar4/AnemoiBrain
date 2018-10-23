# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

from socket import *

class Camera:
    def __init__(self):
        
        # initialize the camera and grab a reference to the raw camera capture
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        
        # allow the camera to warmup
        time.sleep(0.1)

        self.ss = None

    def streamBindTCP(self, port = 8195):
        # Drone Camera is the host
        self.ip = ""
        self.port = port 
        self.s = socket(AF_INET, SOCK_STREAM)
        self.s.bind((self.ip, port))
        self.s.listen(5)
        self.ss, self.a = self.s.accept()   # Wait for a Client

    def streamReverseTCP(self, ip = "", port = 8195):   
        # Drone Camera is the Client
        self.ip = streamip
        self.port = port
        self.s = socket(AF_INET, SOCK_STREAM)
        self.s.connect((streamip, port))
        self.ss = self.s

    def startStream(self):
        # capture frames from the camera
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
            
            # Send this data over to Base via sockets
            print(type(image))
            print(image.shape)
            self.ss.send(image.tostring())
            # show the frame
            #cv2.imshow("Frame", image)
            #key = cv2.waitKey(1) & 0xFF
        
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
        
            # if the `q` key was pressed, break from the loop
            #if key == ord("q"):
            #	break

    