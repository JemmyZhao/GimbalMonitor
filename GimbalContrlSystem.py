import threading
import time
import serial
import numpy
import matplotlib
from enum import Enum
import struct
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import matplotlib

class MsgState(Enum):
    MAVLINK_RX_STOP = 1
    MAVLINK_RX_STX = 2
    MAVLINK_RX_PLTH = 3
    MAVLINK_RX_PSEQ = 4
    MAVLINK_RX_SYSID = 5
    MAVLINK_RX_COMPID = 6
    MAVLINK_RX_MSGID = 7
    MAVLINK_RX_DATA = 8
    MAVLINK_RX_CRCL = 9
    MAVLINK_RX_CRCH = 10

# Mavlink Msg
class MavMsg:
    def __init__(self):
        self.stx = 0
        self.payload_len = 0
        self.packet_seq = 0
        self.sys_id = 0
        self.comp_id = 0
        self.msg_id = 0
        self.data = []
        self.crc_l = 0
        self.crc_h = 0

MAV_TYPE_GIMBAL = 26
MAV_COMP_ID_IMU = 200
MAV_COMP_ID_CONTROLLER = 70

# Attitude quaternion info
MAVLINK_MSG_ID_ATTITUDE_QUATERNION = 31
MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN = 32
MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC = 246

# Attitude info
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_ATTITUDE_LEN = 28
MAVLINK_MSG_ID_ATTITUDE_CRC = 39

# Control parameter info
MAVLINK_MSG_ID_CONTROL_PARAM = 180
MAVLINK_MSG_ID_CONTROL_PARAM_LEN = 29
MAVLINK_MSG_ID_CONTROL_PARAM_CRC = 70

MAVLINK_MSG_ID_CONTROL_PARAM_AXIS_ROLL = 0
MAVLINK_MSG_ID_CONTROL_PARAM_AXIS_PITCH = 1
MAVLINK_MSG_ID_CONTROL_PARAM_AXIS_YAW = 2


def byte2uint32(buffers, bias):
    x = struct.unpack('i', buffers[bias] + buffers[bias+1] + buffers[bias+2] + buffers[bias+3])
    return x[0]

def byte2float(buffers, bias):
    x = struct.unpack('f', buffers[bias] + buffers[bias+1] + buffers[bias+2] + buffers[bias+3])
    return  format(x[0], '.2f')

# Mavlink Attitude Quaternian packet
class MavAttitudeQuatPck:
    def __init__(self):
            self.time_boot_ms = 0
            self.q1 = 1
            self.q2 = 0
            self.q3 = 0
            self.q4 = 0
            self.roll_speed = 0
            self.pitch_speed = 0
            self.yaw_speed = 0


# Mavlink Attitude packet
class MavAttitudePck:
    def __init__(self):
        self.time_boot_ms =0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.roll_speed = 0
        self.pitch_speed = 0
        self.yaw_speed = 0


class MavControlParameterPck:
    def __init__(self):
        self.axis_id = 0;

class Mavlink:
    def __init__(self):
        self.ser = serial.Serial("COM5", 115200, timeout=1)
        self.msg_state = MsgState.MAVLINK_RX_STOP
        self.msg = MavMsg()
        self.msg_data_index = 0
        self.mav_attitude_pck = MavAttitudePck()

    def serial2mavlink(self):
        buf = self.ser.read(1)
        #print(buf)
        if len(buf) > 0:
            if self.msg_state == MsgState.MAVLINK_RX_STOP:
                if(ord(buf) == 0xfe):
                    #print("start")
                    self.msg_state = MsgState.MAVLINK_RX_STX
                    self.msg.stx = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_STX:
                self.msg_state = MsgState.MAVLINK_RX_PLTH
                self.msg.payload_len = ord(buf)
                #print("payload len:", self.msg.payload_len)
            elif self.msg_state == MsgState.MAVLINK_RX_PLTH:
                self.msg_state = MsgState.MAVLINK_RX_PSEQ
                self.msg.packet_seq = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_PSEQ:
                self.msg_state = MsgState.MAVLINK_RX_SYSID
                self.msg.sys_id = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_SYSID:
                self.msg_state = MsgState.MAVLINK_RX_COMPID
                self.msg.comp_id = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_COMPID:
                self.msg_state = MsgState.MAVLINK_RX_MSGID
                self.msg.msg_id = ord(buf)
                self.msg.data.clear()
            elif self.msg_state == MsgState.MAVLINK_RX_MSGID:
                self.msg.data.append(buf)
                #print("data", self.msg_data_index)
                self.msg_data_index += 1
                if(self.msg_data_index == self.msg.payload_len):
                    self.msg_data_index = 0
                    self.msg_state = MsgState.MAVLINK_RX_DATA
                #print('data len: ', len(self.msg.data))
            elif self.msg_state == MsgState.MAVLINK_RX_DATA:
                self.msg_state = MsgState.MAVLINK_RX_CRCL
                self.msg.crc_l = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_CRCL:
                self.msg_state = MsgState.MAVLINK_RX_CRCH
                self.msg.crc_h = ord(buf)
                self.msg_state = MsgState.MAVLINK_RX_STOP

    def decode_attitude(self):
        if (self.msg_state == MsgState.MAVLINK_RX_STOP) and (self.msg.payload_len == MAVLINK_MSG_ID_ATTITUDE_LEN):
            #print('payload len: ', self.msg.payload_len, 'msg data len: ', len(self.msg.data))
            self.mav_attitude_pck.time_boot_ms = byte2uint32(self.msg.data, 0)
            self.mav_attitude_pck.roll = byte2float(self.msg.data, 4)
            self.mav_attitude_pck.pitch = byte2float(self.msg.data, 8)
            self.mav_attitude_pck.yaw = byte2float(self.msg.data, 12)
            self.mav_attitude_pck.roll_speed = byte2float(self.msg.data, 16)
            self.mav_attitude_pck.pitch_speed = byte2float(self.msg.data, 20)
            self.mav_attitude_pck.yaw_speed = byte2float(self.msg.data, 24)
            return True
        else:
            return False

    def get_attitude(self):
        self.serial2mavlink()
        return self.decode_attitude()

    def print_msg_data(self):
        n = 0
        for i in self.msg.data:
            print(i, ' ', n)
            n = n+1

    def print_attitude(self):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            print("Roll:", self.mav_attitude_pck.roll, '\t', "Pitch: ", self.mav_attitude_pck.pitch, '\t', "Yaw: ", self.mav_attitude_pck.yaw, '\t')

    def get_print_attitude(self):
        while(1):
            self.serial2mavlink()
            self.decode_attitude()
            self.print_attitude()

#
mavlink = Mavlink()

# tiem range of the sensor
time_len = 500
time_wight = 0.8
time_head = -time_len * time_wight
time_tail = time_len * (1 - time_wight)
time_range = np.arange(time_head, time_tail, 1)
index_time_now = int(time_len*time_wight)
index_time_head = 0
index_time_tail = time_len - 1

fig = plt.figure()
fig.patch.set_facecolor('w')

ax_bg = 'w'
angle_lim = 100
angle_speed_lim = 10
attitude_window = fig.add_subplot(1, 2, 1, xlim=(time_head, time_tail), ylim=(-angle_lim, angle_lim),
                                  axisbg=ax_bg, title="Angle")
attitude_speed_window = fig.add_subplot(1, 2, 2, xlim=(time_head, time_tail), ylim=(-angle_speed_lim, angle_speed_lim),
                                        axisbg=ax_bg, title="Angle Speed")

# Set grid
attitude_window.grid()
gridlines = attitude_window.get_xgridlines() + attitude_window.get_ygridlines()
for line in gridlines:
    line.set_linestyle('-')
    line.set_alpha(0.5)

attitude_speed_window.grid()
gridlines = attitude_speed_window.get_xgridlines() + attitude_speed_window.get_ygridlines()
for line in gridlines:
    line.set_linestyle('-')
    line.set_alpha(0.5)



# Data to plot
roll_data = np.zeros(len(time_range))
pitch_data = np.zeros(time_len)
yaw_data = np.zeros(time_len)
roll_speed_data = np.zeros(time_len)
pitch_speed_data = np.zeros(time_len)
yaw_speed_data = np.zeros(time_len)

# Line style
# Set color
# Line width
line_width = 1.5
line_alpha = 0.6
line_roll, = attitude_window.plot(time_range, roll_data, label="Roll", color="red",
                                  linewidth=line_width, alpha=line_alpha)
fill_roll_line = attitude_window.fill_between(time_range, 0, roll_data, facecolor='red', alpha=0.4)
line_pitch, = attitude_window.plot(time_range, pitch_data, label="Pitch", color="green",
                                   linewidth=line_width, alpha=line_alpha)
line_yaw, = attitude_window.plot(time_range, yaw_data, label="Yaw",
                                 linewidth = line_width, alpha=line_alpha)

line_roll_speed, = attitude_speed_window.plot(time_range, roll_speed_data, label="Roll", color="red",
                                              linewidth=line_width, alpha=line_alpha)
line_pitch_speed, = attitude_speed_window.plot(time_range, pitch_speed_data, label="Roll", color="green",
                                               linewidth=line_width, alpha=line_alpha)
line_yaw_speed, = attitude_speed_window.plot(time_range, yaw_speed_data, label="Roll",
                                             linewidth=line_width, alpha=line_alpha)

# Fill Color


# Set annotate
px_text = time_len + 1
ann_roll = attitude_window.annotate('%.2f' % (roll_data[index_time_tail]), xy=(time_len, roll_data[index_time_tail]),
                              xytext=(px_text, roll_data[index_time_tail]), color='red')
ann_pitch = attitude_window.annotate('%.2f' % (pitch_data[index_time_tail]), xy=(time_len, pitch_data[index_time_tail]),
                                xytext=(px_text, pitch_data[index_time_tail]), color='red')
ann_yaw = attitude_window.annotate('%.2f' % (yaw_data[index_time_tail]), xy=(time_len, yaw_data[index_time_tail]),
                            xytext=(px_text, yaw_data[index_time_tail]), color='red')





def plot_init():
    line_roll.set_data(time_range, roll_data)
    line_pitch.set_data(time_range, pitch_data)
    line_yaw.set_data(time_range, yaw_data)
    line_roll_speed.set_data(time_range, roll_speed_data)
    line_pitch_speed.set_data(time_range, pitch_speed_data)
    line_yaw_speed.set_data(time_range, yaw_speed_data)
    return line_roll, line_pitch, line_yaw, line_roll_speed, line_pitch_speed, line_yaw_speed

def move_data_window(x_data, x_new):
    x_data = np.append(x_data, x_new)
    x_data = np.delete(x_data, 0)
    return x_data

def animate(i):
    global roll_data
    global roll_speed_data
    global pitch_data
    global pitch_speed_data
    global yaw_data
    global yaw_speed_data

    line_roll.set_ydata(roll_data)
    line_pitch.set_ydata(pitch_data)
    line_yaw.set_ydata(yaw_data)
    line_roll_speed.set_ydata(roll_speed_data)
    line_pitch_speed.set_ydata(pitch_speed_data)
    line_yaw_speed.set_ydata(yaw_speed_data)

    return line_roll, line_pitch, line_yaw, line_roll_speed, line_pitch_speed, line_yaw_speed


def getData():
    global roll_data
    global pitch_data
    global yaw_data
    global roll_speed_data
    global pitch_speed_data
    global yaw_speed_data
    global mavlink
    while(1):
        if mavlink.get_attitude():
            #print('getData')
            roll_data = np.append(np.delete(roll_data, 0), mavlink.mav_attitude_pck.roll)
            pitch_data = np.append(np.delete(pitch_data, 0), mavlink.mav_attitude_pck.pitch)
            yaw_data = np.append(np.delete(yaw_data, 0), mavlink.mav_attitude_pck.yaw)
            roll_speed_data = np.append(np.delete(roll_speed_data, 0), mavlink.mav_attitude_pck.roll_speed)
            pitch_speed_data = np.append(np.delete(pitch_speed_data, 0), mavlink.mav_attitude_pck.pitch_speed)
            yaw_speed_data = np.append(np.delete(yaw_speed_data, 0), mavlink.mav_attitude_pck.yaw_speed)


threads = []
t1 = threading.Thread(target=getData)
threads.append(t1)

if __name__ == '__main__':
    for t in threads:
        t.setDaemon(True)
        t.start()

    anim = animation.FuncAnimation(fig, animate, init_func=plot_init,
                                   interval=5, blit=True)
    plt.show()















