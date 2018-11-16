from socket import *
import cv2
import pickle
import struct
import os, sys, time, re, json
import numpy as np
import matplotlib.pyplot as plt
import airsim
import queue 
from threading import Thread

imread = plt.imread
def imread8(im_file):
    ''' Read image as a 8-bit numpy array '''
    im = np.asarray(Image.open(im_file))
    return im

def read_png(res):
    import io, PIL.Image
    img = PIL.Image.open(io.BytesIO(res))
    return np.asarray(img)

def read_npy(res):
    import io
    return np.load(io.BytesIO(res))

client = airsim.MultirotorClient()

def getView(camera = "0"):
    responses = client.simGetImages([airsim.ImageRequest(camera, airsim.ImageType.Scene, False, False)])
    response = responses[0]
    # get numpy array
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 
    # reshape array to 4 channel image array H X W X 4
    img_rgba = img1d.reshape(response.height, response.width, 4)  
    return img_rgba


class RpiCameraFeed:
    def __init__(self, ip = "10.1.1.3", port = 8195, mode = 0):
        # Mode 0 -> Drone is TCP Server, 1 -> Drone is Client
        self.port = port 
        self.ip = ip
    def getStream(self, res = [480, 640, 3]):
        self.s = socket(AF_INET, SOCK_STREAM)
        self.s.connect((self.ip, self.port))
        #g = self.s.recv(1024) # Recieve the size of the sample image
        s = 921764#struct.unpack("L", g)[0]
        p = b''
        k = b''
        while True:
            i = len(k)
            p = k
            while i < s:
                m = self.s.recv(4096)
                p += m
                #print(len(p))
                i += len(m)
            if p == 0:
                break
            k = p[s:]
            p = p[:s]
            img = pickle.loads(p)
            cv2.imshow("Frame", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break


class AirSimCameraFeed:
    def __init__(self, ip = "10.1.1.3", port = 8195, mode = 0, delay = 0.05, client = airsim.MultirotorClient()):
        # Mode 0 -> Drone is TCP Server, 1 -> Drone is Client
        self.port = port 
        self.ip = ip
        self.delay = delay
        self.client = client
    def getView(self, camera = "0"):
        responses = self.client.simGetImages([airsim.ImageRequest(camera, airsim.ImageType.Scene, False, False)])
        response = responses[0]
        # get numpy array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 
        # reshape array to 4 channel image array H X W X 4
        img_rgba = img1d.reshape(response.height, response.width, 4)  
        return img_rgba
    def startStream(self, res = [480, 640, 3], camera = "0", buffer = queue.Queue(), disp = True):
        self.thread = Thread(target = self.Stream, kwargs = {"buffer":buffer, "disp":disp, "camera":camera, "res":res})
        self.thread.start()
        return self.thread
    def startStereoStream(self, res = [480, 640, 3], camera = "0", buffer = queue.Queue(), disp = True):
        self.thread = Thread(target = self.StereoStream, kwargs = {"buffer":buffer, "disp":disp, "camera":camera, "res":res})
        self.thread.start()
        return self.thread
    def Stream(self, res = [480, 640, 3], camera = "0", buffer = queue.Queue(), disp = True):
        while True:
            img = self.getView(camera)
            try:
                buffer.put_nowait(img)
            except:
                pass
            if(disp):
                cv2.imshow("Frame", img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
            time.sleep(self.delay)
    def StereoStream(self, res = [480, 640, 3], cameras = ("2", "1"), buffer = queue.Queue(4096), disp = True):
        while True:
            left = self.getView(cameras[0])
            right = self.getView(cameras[1])
            try:
                buffer.put_nowait((left,right))
            except:
                pass
            img = cv2.hconcat((left, right))
            if(disp):
                cv2.imshow("Frame", img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
            time.sleep(self.delay)
    def capture(self,res=[480, 640, 3]):
        img = self.getView("0")
        return img
    def show(self, img):
        if img is None:
            self.show(self.capture())
        else:
            cv2.imshow("frame", img)
            k = cv2.waitKey(0)
    def captureStereo(self,res=[480, 640, 3], cameras = ("0", "1")):
        left = self.getView(cameras[0])
        right = self.getView(cameras[1])
        return (left, right)
    def showStereo(self, stereo):
        if stereo is None:
            self.showStereo(self.captureStereo())
        else:
            img = cv2.hconcat(stereo)
            cv2.imshow("Frame", img)
            k = cv2.waitKey(0)

class UnrealCameraFeed:
    from unrealcv import client
    def __init__(self, ip = "10.1.1.3", port = 8195, mode = 0, delay = 0.05):
        # Mode 0 -> Drone is TCP Server, 1 -> Drone is Client
        self.port = port 
        self.ip = ip
        client.connect()
        self.delay = delay
        #plt.title('Keep this window in focus, it will be used to receive key press event')
        #plt.show() # Add event handler
    def startStream(self, res = [480, 640, 3], camera = "0", buffer = queue.Queue(), disp = True):
        self.thread = Thread(target = self.Stream, kwargs = {"buffer":buffer, "disp":disp, "camera":camera, "res":res})
        self.thread.start()
        return self.thread
    def startStereoStream(self, res = [480, 640, 3], camera = "0", buffer = queue.Queue(), disp = True):
        self.thread = Thread(target = self.StereoStream, kwargs = {"buffer":buffer, "disp":disp, "camera":camera, "res":res})
        self.thread.start()
        return self.thread
    def Stream(self, res = [480, 640, 3], camera = "0", buffer = queue.Queue(), disp = True):
        while True:
            rr = client.request('vget /camera/'+camera+'/lit png')
            img = read_png(rr)
            try:
                buffer.put_nowait(img)
            except:
                pass
            if(disp):
                cv2.imshow("Frame", img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
            time.sleep(self.delay)
    def StereoStream(self, res = [480, 640, 3], cameras = ("2", "1"), buffer = queue.Queue(4096), disp = True):
        while True:
            res = client.request('vget /camera/'+cameras[0]+'/lit png')
            left = read_png(res)
            res = client.request('vget /camera/'+cameras[1]+'/lit png')
            right = read_png(res)
            try:
                buffer.put_nowait((left,right))
            except:
                pass
            img = cv2.hconcat((left, right))
            if(disp):
                cv2.imshow("Frame", img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
            time.sleep(self.delay)
    def capture(self,res=[480, 640, 3]):
        rr = client.request('vget /camera/0/lit png')
        img = read_png(rr)
        return img
    def show(self, img):
        if img is None:
            self.show(self.capture())
        else:
            cv2.imshow("frame", img)
            k = cv2.waitKey(0)
            #plt.imshow(img)
            #plt.show()
    def captureStereo(self,res=[480, 640, 3]):
        res = client.request('vget /camera/0/lit png')
        left = read_png(res)
        res = client.request('vget /camera/1/lit png')
        right = read_png(res)
        return (left, right)
    def showStereo(self, stereo):
        if stereo is None:
            self.showStereo(self.captureStereo())
        else:
            img = cv2.hconcat(stereo)
            cv2.imshow("Frame", img)
            #plt.subplot(121); plt.imshow(stereo.left)
            #plt.subplot(122); plt.imshow(stereo.right)
            k = cv2.waitKey(0)
            #plt.show()

class CameraFeed:
    def __init__(self, ip = "10.1.1.3", port = 8195, mode = 0, camType = RpiCameraFeed):
        self.camera = camType()
    def getStream(self, res=[480, 640, 3]):
        self.camera.getStream(res)
    def capture(self, res=[480, 640, 3]):
        return self.camera.capture(res)
    def show(self, img = None):
        #self.camera.show(img)
        if img is None:
            self.show(self.capture())
        else:
            cv2.imshow("frame", img)
            k = cv2.waitKey(0)

#c = CameraFeed(port = 8185)
#c = CameraFeed(camType = UnrealCameraFeed)
#c.getStream()
#c.show()
