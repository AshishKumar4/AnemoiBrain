import serial 
import time 

# Enable Serial Communication
ser = serial.Serial("/dev/ttyUSB1", baudrate=115200, timeout=1)

def getResult(ser, n=4):
    ser.readline() # \r\n
    o = ser.readline() # Actual output
    while n > 2:
        ser.readline();
        n -= 1
    return o

class gps_sim908:
    def __init__(self, sfile):
        self.sfile = sfile
        self.sfile.write(b'ATE0'+b'\r')      # Disable the Echo
        self.sfile.flush()
        time.sleep(1)
        self.sfile.read_all()
        
        self.sfile.write(b'AT+CGPSPWR?' + b'\r')
        self.sfile.flush()
        o = getResult(self.sfile)
        print(o)
        
        if b'CGPSPWR: 1' not in o:
            print('Powering on GPS')
            self.sfile.write(b'AT+CGPSPWR=1' + b'\r')
            self.sfile.flush()
            print(getResult(self.sfile))
            time.sleep(1)

            self.sfile.write(b'AT+CGPSRST=0' + b'\r')
            self.sfile.flush()
            print(getResult(self.sfile))
            time.sleep(1)
            
        self.sfile.write(b'AT+CGPSINF=0' + b'\r')
        self.sfile.flush()
        print(getResult(self.sfile, 3))
        
        self.sfile.write(b'AT+CGPSSTATUS?' + b'\r')
        self.sfile.flush()
        o = getResult(self.sfile)
        print(o)
        
        while b'Fix' not in o:
            print('Attempting to get GPS Fix...')
            time.sleep(10)
            self.sfile.write(b'AT+CGPSSTATUS? ' + b'\r')
            self.sfile.flush()
            o = getResult(self.sfile)
            print(o)
        print('Got GPS Fix Successfully!')
    def getRaw(self, inf = b'32'):
        self.sfile.write(b'AT+CGPSINF=' + inf + b'\r')
        #self.sfile.flush()
        o = getResult(self.sfile, 3)
        return o
        