import serial
import time 
import sys

device = sys.argv[1] if len(sys.argv)>1 else "/dev/ttyUSB0"
baudrate = sys.argv[2] if len(sys.argv)>2 else 115200
s = serial.Serial(device, baudrate)
k = s.read_until(b'\n')
print(k, len(k))
k = range(0, 1000)
print("Reading...")
while True:
    t = time.time()
    for i in k:
        h = s.read_until(b'\n')
    print(1000/(time.time() - t), "Hz")
