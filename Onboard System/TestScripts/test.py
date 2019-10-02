import serial
import time 

s = serial.Serial('/dev/ttyUSB0', 250000)
k = s.read_until(b'\r\n')
print(k, len(k))
k = range(0, 1000)
print("Reading...")
while True:
    t = time.time()
    for i in k:
        h = s.read_until(b'\n')
    print(1000/(time.time() - t), "Hz")