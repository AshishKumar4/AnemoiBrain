import airsim
import queue
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time
from threading import Thread
from socket import *


class AirSimControls:
    def __init__(self, ip="0.0.0.0", portBase=8400):
        # We Spawn 'n' threads corresponding to number of channels
        self.channels = 6
        self.ports = list()
        self.ip = ip
        self.threads = list()
        self.portBase = portBase
        self.socks = list()
        self.connections = list()
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
        print("Starting Listener Threads, Base PORT is " + str(portBase))
        for i in range(0, 6):
            self.threads[i].start()
        for i in range(0, 6):
            self.threads[i].join()
        return None
    def ChannelControllers(self, id):
        print("ID: " + str(id))
        lport = id + self.portBase
        s = socket(AF_INET, SOCK_STREAM)
        self.socks.append(s)
        s.bind((self.ip, lport))
        s.listen(5)
        while True:
            c, a = s.accept()
            self.connections.append(c)
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
                try:
                    g = c.recv(4096)
                    if g == 0:
                        print("Connection Broken...")
                        del self.connections[id]
                        del c
                        break
                    tmp = bb + g
                    a = tmp.decode("utf-8")
                    bb = bytes(
                        a[tmp.rfind(b".") :], "utf-8"
                    )  # Get the index of the last dot and store it for future
                    b = str(a).split(".")
                except Exception as e:
                    print("Error in Recieving and parsing...")
                    del self.connections[id]
                    del c
                    break
                print(b)
                for i in b:  # For multiple logged commands
                    cmd = i.split(":")
                    if len(cmd) == 3:  # We recieved the command properly
                        # print("Got Request nicely...")
                        val = int(cmd[1])
                        print(
                            "Got Request " + self.channelTable[id]["name"] + " :" + str(val)
                        )
                        self.channelTable[id]["func"](val)
        return None
    def setThrottle(self, val):
        print("Set Value to " + str(val))
        return None
    def setPitch(self, val):
        print("Set Value to " + str(val))
        return None
    def setRoll(self, val):
        print("Set Value to " + str(val))
        return None
    def setYaw(self, val):
        print("Set Value to " + str(val))
        return None
    def setAux1(self, val):
        print("Set Value to " + str(val))
        return None
    def setAux2(self, val):
        print("Set Value to " + str(val))
        return None

