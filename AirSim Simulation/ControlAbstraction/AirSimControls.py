import airsim
import queue
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time
from threading import Thread
from socket import *

#  STREAM_PROTOCOL_1 Refers to conventional failproof streaming of values as packets, encapsulated and later stripped off on the other hand 
#  STREAM_PROTOCOL_2 Refers to newer, faster and lightweight but simplest streaming, stream of simple bytes. This isn't failproof and errors 
#                    may be entroduced.

STREAM_PROTOCOL = 2


def valMap(i, imin, imax, omin, omax):
    aa = omin + (((omax - omin) / (imax - imin)) * (i - imin))  # The formula to map ranges
    return aa


def valMapMids(i, imin, imax, mid, omin, omax):
    aa = 0.0
    if(i <= mid):
        aa = ((i - imin) * (((omax-omin)/2.0)/(mid - imin))) + omin
    else:
        aa = ((i - mid) * (((omax-omin)/2.0)/(imax - mid))) + ((omax-omin)/2)
    return aa


class AirSimControls:
    def __init__(self, ip="0.0.0.0", portBase=8400):
        # We Spawn 'n' threads corresponding to number of channels
        self.channels = 6
        self.ports = list()
        self.ip = ip
        self.threads = list()
        self.portBase = portBase
        self.socks = dict()#list()
        self.connections = dict()#list()
        self.channelTable = {
            0: {"name": "throttle", "func": self.setThrottle},
            1: {"name": "pitch", "func": self.setPitch},
            2: {"name": "roll", "func": self.setRoll},
            3: {"name": "yaw", "func": self.setYaw},
            4: {"name": "aux1", "func": self.setAux1},
            5: {"name": "aux2", "func": self.setAux2},
        }
        # self.channelDescriptors
        print("Spawning Listener Threads to listen to the Base Station...")
        for i in range(0, 6):
            self.ports.append(portBase + i)
            self.threads.append(Thread(target=self.ChannelControllers, args=(i,)))
        self.MainThread = Thread(target = self.ChannelSyncToSim)
        print("Starting Listener Threads, Base PORT is " + str(portBase))
        # We Now need to setup the AirSim Simulator before we can start the threads!
        ############################ AirSim MultiCopter Initialization ############################
        client = airsim.MultirotorClient()
        self.client = client
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
        # Some Parameter Definitions, tweak them for realistic behavior
        self.timeSlice = 0.001
        self.rmin = -1
        self.rmax = 1
        self.pmin = -1
        self.pmax = 1
        self.ymin = -6
        self.ymax = 6
        self.tmin = 0
        self.tmax = 10
        self.t = self.r = self.y = self.p = self.a1 = self.a2 = 0
        # moveByAngleThrottleAsync --> pitch, roll, throttle, yaw_rate
        ###########################################################################################
    def launchThreads(self):
        for i in range(0, 6):
            self.threads[i].start()
        self.MainThread.start()
        for i in range(0, 6):
            self.threads[i].join()
        self.MainThread.join()
        return None
    def ChannelSyncToSim(self):
        # Now we Sync the data and send it over to the simulator
        while True:
            self.client.moveByAngleThrottleAsync(self.p, self.r, self.t, self.y, self.timeSlice)
            time.sleep(self.timeSlice)
    def ChannelControllers(self, id):
        print("ID: " + str(id))
        lport = id + self.portBase
        s = socket(AF_INET, SOCK_STREAM)
        self.socks[id] = s
        s.bind((self.ip, lport))
        s.listen(5)
        while True:
            try:
                c, a = s.accept()
                self.connections[id] = c
                # Handshake Message ->  IN: "Hello Gardien!"
                buff = c.recv(1024).decode("utf-8")
                if buff == "Hello Gardien!":
                    print("Connection to overlord successfull!")
                    c.send(b"Hello Overloard!")  # Handshake
                else:
                    print("Connection Failed!, Got Back this handshake -->")
                    print(buff)
                    exit()
                # Go into an infinite while loop and look for command requests
                bb = b""
                while True:
                    g = c.recv(4096)
                    if len(g) == 0:
                        print("Connection Broken...")
                        del self.connections[id]
                        del c
                        raise Exception('Connection Broken')
                    try:
                        if STREAM_PROTOCOL == 1:
                            tmp = bb + g
                            a = tmp.decode("utf-8")
                            bb = bytes(a[tmp.rfind(b".") :], "utf-8")  # Get the index of the last dot and store it for future
                            b = str(a).split(".")
                            print(b)
                            for i in b:  # For multiple logged commands
                                cmd = i.split(":")
                                if len(cmd) == 3:  # We recieved the command properly
                                    # print("Got Request nicely...")
                                    val = float(int(cmd[1]))
                                    #print("Got Request "+ self.channelTable[id]["name"] + " :"+ str(val))
                                    self.channelTable[id]["func"](val)
                        elif STREAM_PROTOCOL == 2:
                            for i in g:
                                self.channelTable[id]["func"](i)
                    except ValueError as e:
                        print("Error in Recieving and parsing...")
                        del self.connections[id]
                        del c
                        raise Exception('Unknown Error in parsing')
            except Exception as e:
                print(e)
        return None
    def setThrottle(self, val):
        print("Set Value to " + str(val))
        self.t = valMap(val, 0, 255, self.tmin, self.tmax)
        return None
    def setPitch(self, val):
        print("Set Value to " + str(val))
        self.p = valMap(val, 0, 255, self.pmin, self.pmax)
        return None
    def setRoll(self, val):
        print("Set Value to " + str(val))
        self.r = valMap(val, 0, 255, self.rmin, self.rmax)
        return None
    def setYaw(self, val):
        print("Set Value to " + str(val))
        self.y = valMap(val, 0, 255, self.ymin, self.ymax)
        return None
    def setAux1(self, val):
        print("Set Value to " + str(val))
        # self.t = valMap(val, 0, 255, self.tmin, self.tmax)
        return None
    def setAux2(self, val):
        print("Set Value to " + str(val))
        # self.t = valMap(val, 0, 255, self.tmin, self.tmax)
        return None

