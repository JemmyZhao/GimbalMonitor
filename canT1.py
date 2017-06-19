from ctypes import *
import time
import struct

CHAR = c_char
UCHAR = c_ubyte
BYTE = c_byte
WORD = c_ushort
DWORD = c_ulong

WCHAR = c_wchar
UINT = c_uint
USHORT = c_ushort
INT = c_int
ULONG = c_ulong

DOUBLE = c_double
FLOAT = c_float

BOOLEAN = BYTE
BOOL = c_long

PVOID = c_void_p


class BOARO_INFO(Structure):
    _fields_ = [('hw_Version', USHORT),
                ('fw_Version', USHORT),
                ('dr_Version', USHORT),
                ('in_Version', USHORT),
                ('irq_Num', USHORT),
                ('can_Num', BYTE),
                ('str_Serial_Num', CHAR * 20),
                ('str_hw_Type', CHAR * 40),
                ('Reserved', USHORT * 4)]

    def __str__(self):
        return format(self.hw_Version)

    def show(self):
        print('hw Version: ', self.hw_Version)
        print('fw Version: ', self.fw_Version)
        print('dr Version: ', self.dr_Version)
        print('in Version: ', self.in_Version)
        print('irq Num: ', self.irq_Num)
        print('CAN Num: ', self.can_Num)
        print('str Serial Num: ', self.str_Serial_Num)
        print('str hw Type: ', self.str_hw_Type)
        print('Reserved: ', self.Reserved)


class CAN_OBJ(Structure):
    _fields_ = [('ID', UINT),
                ('TimeStamp', UINT),
                ('TimeFlag', BYTE),
                ('SendType', BYTE),
                ('RemoteFlag', BYTE),
                ('ExternFlag', BYTE),
                ('DataLen', BYTE),
                ('Data', BYTE * 8),
                ('Reserved', BYTE * 3)]

    def __str__(self):
        return format(self.ID)

    def show(self):
        print('ID:', hex(self.ID))
        print('Time Stamp: ', self.TimeStamp, '\t Time Flag: ', self.TimeFlag)
        print('Send Type: ', self.SendType, '\t Remote Flag: ', self.RemoteFlag)
        print('Extern Flag: ', self.ExternFlag, '\t Data Len: ', self.DataLen)
        print('Data: ', bytearray(self.Data))
        print('Reserved: ', self.Reserved)


class CAN_STATUS(Structure):
    _fields_ = [('ErrInterrupt', UCHAR),
                ('regMode', UCHAR),
                ('regStatus', UCHAR),
                ('regALCapture', UCHAR),
                ('regECCapture', UCHAR),
                ('regEWLimit', UCHAR),
                ('regRECounter', UCHAR),
                ('regTXCounter', UCHAR),
                ('Reserved', DWORD)]

    def __str__(self):
        return format(self.regRECounter, self.regTXCounter)


class ERR_INFO(Structure):
    _fields_ = [('ErrCode', UINT),
                ('Passive_ErrData', BYTE * 3),
                ('ArLost_ErrData', BYTE)]

    def __str__(self):
        return format(self.ErrCode)


class INIT_CONFIG(Structure):
    _fields_ = [('AccCode', DWORD),
                ('AccMask', DWORD),
                ('Reserved', DWORD),
                ('Filter', UCHAR),
                ('Timing0', UCHAR),
                ('Timing1', UCHAR),
                ('Mode', UCHAR)]

    def show(self):
        print('Acc Code: ', self.AccCode)
        print('Acc Mask: ', hex(self.AccMask))
        print('Reserved: ', self.Reserved)
        print('Filter: ', self.Filter)
        print('Timing 0: ', self.Timing0)
        print('Timing 1: ',  self.Timing1)
        print('Mode: ', self.Mode)

    def __init__(self):
        self.AccCode = 0
        self.AccMask = 0xffffff
        self.Filter = 0
        # 1M
        self.Timing0 = 0
        self.Timing1 = 0x14
        self.Mode = 0

    def __str__(self):
        return format(self.Filter)


# _tagChgDesIPAndPort
class CHGDESIPANDPORT(Structure):
    _fields_ = [('szpwd', CHAR * 10),
                ('szdesip', CHAR * 20),
                ('desport', INT)]

    def __str__(self):
        return format(self.szpwd, self.szdesip)


class FILTER_RECORD(Structure):
    _fields_ = [('ExtFrame', DWORD),
                ('Start', DWORD),
                ('End', DWORD)]

    def __str__(self):
        return format(self.ExtFrame)


#can_api = CDLL('ECanVci.dll')
can_api = WinDLL('ECanVci.dll')

can_api.OpenDevice.argtypes = [DWORD, DWORD, DWORD]
can_api.CloseDevice.argtypes = [DWORD, DWORD]

can_api.InitCAN.argtypes = [DWORD, DWORD, DWORD, POINTER(INIT_CONFIG)]

can_api.ReadBoardInfo.argtypes = [DWORD, DWORD, POINTER(BOARO_INFO)]
can_api.ReadErrInfo.argtypes = [DWORD, DWORD, DWORD, POINTER(ERR_INFO)]
can_api.ReadCANStatus.argtypes = [DWORD, DWORD, DWORD, POINTER(CAN_STATUS)]

can_api.GetReference.argtypes = [DWORD, DWORD, DWORD, DWORD, PVOID]
can_api.SetReference.argtypes = [DWORD, DWORD, DWORD, DWORD, PVOID]

can_api.GetReceiveNum.argtypes = [DWORD, DWORD, DWORD]
can_api.ClearBuffer.argtypes = [DWORD, DWORD, DWORD]

can_api.StartCAN.argtypes = [DWORD, DWORD, DWORD]
can_api.ResetCAN.argtypes = [DWORD, DWORD, DWORD]

can_api.Transmit.argtypes = [DWORD, DWORD, DWORD, POINTER(CAN_OBJ), ULONG]
can_api.Receive.argtypes = [DWORD, DWORD, DWORD, POINTER(CAN_OBJ), ULONG, INT]


class CAN:
    def __init__(self):
        self.deviceType = 3
        self.deviceInd = 0
        self.CANInd = 0
        self.init_config = INIT_CONFIG()
        self.board_info = BOARO_INFO()
        self.board_info_p = pointer(self.board_info)
        self.rxo = CAN_OBJ()
        self.rxo_p = pointer(self.rxo)
        self.txo = CAN_OBJ()
        self.txo_p = pointer(self.txo)

    def open(self):
        # Open Device
        dwRel = can_api.OpenDevice(self.deviceType, self.deviceInd, self.CANInd)
        if dwRel == 1:
            print('CAN Device open success!')
        else:
            print('CAN Device NOT found')
            return False
        # Init Device
        dwRel = can_api.InitCAN(self.deviceType, self.deviceInd, self.CANInd,
                                byref(self.init_config))
        if dwRel == 1:
            print('CAN Device init success!')
        else:
            print('CAN Device init fail')
            return False
        # Start CAN
        dwRel = can_api.StartCAN(self.deviceType, self.deviceInd, self.CANInd)
        if dwRel == 1:
            print('CAN Start Success')
        else:
            print('CAN Start Fault')
            return False

        self.init_config.show()
        return True

    def close(self):
        dwRel = can_api.CloseDevice(self.deviceType, self.deviceInd)
        if dwRel == 1:
            print('CAN Device Closed!')
            return True
        else:
            print('Can not close CAN Device')
            return False

    def read(self, num=1):
        length = can_api.Receive(self.deviceType, self.deviceInd, self.CANInd,
                                 self.rxo_p, num, 10)
        return length

    def write(self):
        length = can_api.Transmit(self.deviceType, self.deviceInd, self.CANInd,
                                  self.txo_p, 1)
        return length

    def show_rx_msg(self):
        self.rxo_p[0].show()
        print('\n')

can = CAN()
can.open()
for i in range(20):
    length = can.read()
    if length > 0:
        can.show_rx_msg()
    time.sleep(0.1)

can.close()






