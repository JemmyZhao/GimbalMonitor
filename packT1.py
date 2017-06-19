import struct

a = b'\x03'
b = b''
b = struct.unpack('f',a+a+a+a)
print(b)