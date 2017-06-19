import serial
import time
import struct

a = 1
d = bytearray()
d.append(2)
d=d+struct.pack('>i',1000)
ser = serial.Serial('COM7', 115200, timeout=1)
for i in range(1, 2):
 #ser.write(b'\x08')
 len = ser.write(d)
 print(len)
 time.sleep(0.1)
# s = ser.read(10)        # read up to ten bytes (timeout)
# line = ser.readline()   # read a '/n' terminated line

ser.close()

