from socket import * 
from time import *

class RemoteControl:
    def __init__(self, ip = '127.0.0.1', port = 8194):
        self.s = socket(AF_INET, SOCK_STREAM)
        self.s.connect((ip, port))
        self.s.send(b'Hello Gardien!') # The Handshake Message
        if(self.s.recv(1024) != b'Hello Overloard!'):
            raise Exception("Could Not Connect to the Drone Control Server!")        
        self.cmd(b'T:0 Y:0 R:0 P:0 C:0') # Set everything to 0
        #print(self.s.recv(1024))
        self.cmd(b'T:255 Y:255: R:255 P:255 C:0')
        self.cmd(b'T:0 Y:0 R:127 P:127 C:0')
        self.y = 0
        self.r = self.p = 127
        return 
    def arm(self):
        self.cmd(b'T:0 Y:255 R:127 P:127 C:0')
        sleep(1)
        self.cmd(b'T:0 Y:127 R:127 P:127 C:0')
        self.y = self.r = self.p = 127
        print('ARMed Successfully...')
        return True
    def disarm(self):
        self.cmd(b'T:0 Y:0 R:127 P:127 C:0')
        self.y = 0
        self.r = self.p = 127
        print('Disarmed Successfully...')
        return True
    def throttle(self, val):
        self.cmd(b'T:' + val + ' Y:' + self.y + ' R:' + self.r + ' P:' + self.p + ' C:0')
    def cmd(self, c):
        self.s.send(c)
        f = self.s.recv(1024)
        return f
