# V1 add GUI
# V2 add CAN BUS PRO
import threading
import time
import serial
from enum import Enum
import struct
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import euclid3
import math
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import can_pro_gimbal as can_pro


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

# Efficient data info
MAVLINK_MSG_ID_EFF_DATA = 77
MAVLINK_MSG_ID_EFF_DATA_LEN = 22
MAVLINK_MSG_ID_EFF_DATA_CRC = 79

MAVLINK_AX_ROLL = 0
MAVLINK_AX_PITCH = 1
MAVLINK_AX_YAW = 2


class Controller:
    def __init__(self):
        # parameter set
        #         rate_p    rate_i    rate_d   rate_i_max  rate_f_hz  stab_p    stab_d
        self.ax_param = [[0, 0, 0, 0, 0, 0, 0],  # Roll ax
                         [0, 0, 0, 0, 0, 0, 0],  # Pitch ax
                         [0, 0, 0, 0, 0, 0, 0]]  # Yaw ax

    def __repr__(self):
        return ''

    def param_map(self, axis_id=0, param_id=0, param=0):
        self.ax_param[axis_id][param_id] = param

    def get_param(self, axis_id=0, param_id=0):
        return self.ax_param[axis_id][param_id]

    def get_param_str(self, axis_id=0, param_id=0):
        #print(axis_id, ' ', param_id, ' ', format(self.ax_param[axis_id][param_id], '.4f'))
        return format(self.ax_param[axis_id][param_id], '.4f')

    def print_param(self):
        for i in self.ax_param:
            for j in i:
                print(j)

    def set_param(self, axis_id=0, param_id=0, param=0.0):
        self.ax_param[axis_id][param_id] = param

    def pack_param(self, axis_id=0, param_id=0):
        msg = bytearray()
        msg.append(0xFE)
        msg.append(MAVLINK_MSG_ID_EFF_DATA_LEN)
        msg.append(0)
        msg.append(MAV_TYPE_GIMBAL)
        msg.append(MAV_COMP_ID_CONTROLLER)
        msg.append(MAVLINK_MSG_ID_EFF_DATA)
        msg.append(axis_id)
        msg.append(param_id)
        msg = msg + struct.pack('f', self.ax_param[axis_id][param_id])
        msg = msg + struct.pack('f', 1)
        msg = msg + struct.pack('f', 0)
        msg = msg + struct.pack('f', 0)
        msg = msg + struct.pack('f', 0)
        msg.append(0)
        msg.append(0)
        #print("ax: ", axis_id, ' ', 'param: ', param_id, ' ', 'value: ', self.ax_param[axis_id][param_id])
        #print('Msg: ', msg)
        return msg


# The entity of gimbal
class Gimbal:
    def __init__(self):
        self.attitude_quaternion = euclid3.Quaternion(1.0, 0, 0, 0)
        self.attitude_euler = self.attitude_quaternion.get_euler()
        self.temperature = 0

    def update_attitude(self, mavlink):
        if isinstance(mavlink, Mavlink):
            self.attitude_quaternion.w = mavlink.mav_eff_data_pck.q0
            self.attitude_quaternion.x = mavlink.mav_eff_data_pck.q1
            self.attitude_quaternion.y = mavlink.mav_eff_data_pck.q2
            self.attitude_quaternion.z = mavlink.mav_eff_data_pck.q3
            self.attitude_euler = self.get_euler_ch()

    def get_euler(self):
        rad2deg = 180.0/math.pi
        attitude = self.attitude_quaternion.get_euler()
       # attitude[0] = attitude[0]*rad2deg
       # attitude[1] = attitude[1]*rad2deg
       # attitude[2] = attitude[2]*rad2deg
        #print(type(attitude[0]))
        return attitude[0]*rad2deg, attitude[1]*rad2deg, attitude[2]*rad2deg

    def get_euler_ch(self):
        rad2deg = 180.0/math.pi
        rad_x = math.asin(2 * self.attitude_quaternion.x * self.attitude_quaternion.z
                          - 2 * self.attitude_quaternion.w * self.attitude_quaternion.y)
        rad_y = math.atan2(2 * self.attitude_quaternion.y * self.attitude_quaternion.z
                           + 2 * self.attitude_quaternion.w * self.attitude_quaternion.x,
                           -2 * self.attitude_quaternion.x ** 2 - 2 * self.attitude_quaternion.y ** 2 + 1)
        rad_z = -math.atan2(2 * self.attitude_quaternion.x * self.attitude_quaternion.y
                            + 2 * self.attitude_quaternion.w * self.attitude_quaternion.z,
                            -2 * self.attitude_quaternion.y ** 2 - 2 * self.attitude_quaternion.z ** 2 + 1)
        return rad_x * rad2deg, rad_y * rad2deg, rad_z * rad2deg


def byte2uint32(buffers, bias):
    x = struct.unpack('i', buffers[bias] + buffers[bias+1] + buffers[bias+2] + buffers[bias+3])
    return x[0]


def byte2float(buffers, bias):
    x = struct.unpack('f', buffers[bias] + buffers[bias+1] + buffers[bias+2] + buffers[bias+3])
    return x[0]#format(x[0], '.2f')


def byte2float4(buffers, bias):
    x = struct.unpack('f', buffers[bias] + buffers[bias+1] + buffers[bias+2] + buffers[bias+3])
    return x[0]#format(x[0], '.4f')


def byte2float6(buffers, bias):
    x = struct.unpack('f', buffers[bias] + buffers[bias+1] + buffers[bias+2] + buffers[bias+3])
    return x[0]#format(x[0], '.6f')


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
        self.axis_id = 0
        self.rate_p = 0
        self.rate_i = 0
        self.rate_d = 0
        self.rate_i_max = 0
        self.rate_filter_hz = 0
        self.stabilize_p = 0
        self.stabilize_d = 0


class MavEfficientDataPck:
    def __init__(self):
        self.axis_id = 0
        self.param_id = 0
        self.param = 0
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0


class Mavlink:
    def __init__(self):
        self.ser = serial.Serial("COM5", 115200, timeout=1)
        self.msg_state = MsgState.MAVLINK_RX_STOP
        self.msg_rs = MavMsg()
        self.msg_tx = MavMsg()
        self.msg_data_index = 0
        self.mav_attitude_pck = MavAttitudePck()
        self.mav_attitude_quat_pck = MavAttitudeQuatPck()
        self.mav_control_param_pck = MavControlParameterPck()
        self.mav_eff_data_pck = MavEfficientDataPck()

    def serial_send_buf(self, buf):
        return self.ser.write(buf)

    def serial2mavlink(self):
        buf = self.ser.read(1)
        #print(hex(ord(buf)))
        #print(buf)
        if len(buf) > 0:
            if self.msg_state == MsgState.MAVLINK_RX_STOP:
                if ord(buf) == 0xfe:
                    self.msg_state = MsgState.MAVLINK_RX_STX
                    self.msg_rs.stx = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_STX:
                self.msg_state = MsgState.MAVLINK_RX_PLTH
                self.msg_rs.payload_len = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_PLTH:
                self.msg_state = MsgState.MAVLINK_RX_PSEQ
                self.msg_rs.packet_seq = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_PSEQ:
                self.msg_state = MsgState.MAVLINK_RX_SYSID
                self.msg_rs.sys_id = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_SYSID:
                self.msg_state = MsgState.MAVLINK_RX_COMPID
                self.msg_rs.comp_id = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_COMPID:
                self.msg_state = MsgState.MAVLINK_RX_MSGID
                self.msg_rs.msg_id = ord(buf)
                self.msg_rs.data.clear()
            elif self.msg_state == MsgState.MAVLINK_RX_MSGID:
                self.msg_rs.data.append(buf)
                self.msg_data_index += 1
                if self.msg_data_index == self.msg_rs.payload_len:
                    self.msg_data_index = 0
                    self.msg_state = MsgState.MAVLINK_RX_DATA
            elif self.msg_state == MsgState.MAVLINK_RX_DATA:
                self.msg_state = MsgState.MAVLINK_RX_CRCL
                self.msg_rs.crc_l = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_CRCL:
                self.msg_state = MsgState.MAVLINK_RX_CRCH
                self.msg_rs.crc_h = ord(buf)
                self.msg_state = MsgState.MAVLINK_RX_STOP

    # Return True if decode success
    def decode_attitude(self):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            if self.msg_rs.payload_len == MAVLINK_MSG_ID_ATTITUDE_LEN:
                self.mav_attitude_pck.time_boot_ms = byte2uint32(self.msg_rs.data, 0)
                self.mav_attitude_pck.roll = byte2float(self.msg_rs.data, 4)
                self.mav_attitude_pck.pitch = byte2float(self.msg_rs.data, 8)
                self.mav_attitude_pck.yaw = byte2float(self.msg_rs.data, 12)
                self.mav_attitude_pck.roll_speed = byte2float(self.msg_rs.data, 16)
                self.mav_attitude_pck.pitch_speed = byte2float(self.msg_rs.data, 20)
                self.mav_attitude_pck.yaw_speed = byte2float(self.msg_rs.data, 24)
                return True
            else:
                return False
        else:
            return False

    # Return True if decode success
    def decode_attitude_quaternion(self):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            if self.msg_rs.payload_len == MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN:
                self.mav_attitude_quat_pck.time_boot_ms = byte2uint32(self.msg_rs.data, 0)
                self.mav_attitude_quat_pck.q1 = byte2float4(self.msg_rs.data, 4)
                self.mav_attitude_quat_pck.q2 = byte2float4(self.msg_rs.data, 8)
                self.mav_attitude_quat_pck.q3 = byte2float4(self.msg_rs.data, 12)
                self.mav_attitude_quat_pck.q4 = byte2float4(self.msg_rs.data, 16)
                self.mav_attitude_quat_pck.roll_speed = byte2float(self.msg_rs.data, 20)
                self.mav_attitude_quat_pck.pitch_speed = byte2float(self.msg_rs.data, 24)
                self.mav_attitude_quat_pck.yaw_speed = byte2float(self.msg_rs.data, 28)
                return True
            else:
                return False
        else:
            return False

    # Return True if decode success
    def decode_eff_data(self, controller):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            if self.msg_rs.payload_len == MAVLINK_MSG_ID_EFF_DATA_LEN:
                self.mav_eff_data_pck.axis_id = ord(self.msg_rs.data[0])
                self.mav_eff_data_pck.param_id = ord(self.msg_rs.data[1])
                self.mav_eff_data_pck.param = byte2float6(self.msg_rs.data, 2)
                self.mav_eff_data_pck.q0 = byte2float6(self.msg_rs.data, 6)
                self.mav_eff_data_pck.q1 = byte2float6(self.msg_rs.data, 10)
                self.mav_eff_data_pck.q2 = byte2float6(self.msg_rs.data, 14)
                self.mav_eff_data_pck.q3 = byte2float6(self.msg_rs.data, 18)
                #print(self.mav_eff_data_pck.axis_id, ' ', self.mav_eff_data_pck.param_id, ' ', self.mav_eff_data_pck.param)
                if isinstance(controller, Controller):
                    self.map_param_to_controller(controller)
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False

    # Return True if decode success
    def decode_control_parameter(self):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            if self.msg_rs.payload_len == MAVLINK_MSG_ID_CONTROL_PARAM_LEN:
                self.mav_control_param_pck.axis_id = ord(self.msg_rs.data[0])
                self.mav_control_param_pck.rate_p = byte2float6(self.msg_rs.data, 1)
                self.mav_control_param_pck.rate_i = byte2float6(self.msg_rs.data, 5)
                self.mav_control_param_pck.rate_d = byte2float6(self.msg_rs.data, 9)
                self.mav_control_param_pck.rate_i_max = byte2float6(self.msg_rs.data, 13)
                self.mav_control_param_pck.rate_filter_hz = byte2float6(self.msg_rs.data, 17)
                self.mav_control_param_pck.stabilize_p = byte2float6(self.msg_rs.data, 21)
                self.mav_control_param_pck.stabilize_d = byte2float6(self.msg_rs.data, 25)
                return True
            else:
                return False
        else:
            return False

    # Return True if decode success
    def decode_msg(self, controller):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            if self.msg_rs.msg_id == MAVLINK_MSG_ID_EFF_DATA:
                if isinstance(controller, Controller):
                    return self.decode_eff_data(controller)
                else:
                    return False
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_ATTITUDE:
                return self.decode_attitude()
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_CONTROL_PARAM:
                return self.decode_control_parameter()
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                return self.decode_attitude_quaternion()
        else:
            return False

    def map_param_to_controller(self, controller):
        if isinstance(controller, Controller):
            controller.param_map(self.mav_eff_data_pck.axis_id, self.mav_eff_data_pck.param_id, self.mav_eff_data_pck.param)
            return True
        else:
            return False

    def get_param_from_controller(self, controller, axis_id=0, param_id=0):
        if isinstance(controller, Controller):
            self.mav_eff_data_pck.axis_id = axis_id
            self.mav_eff_data_pck.param_id = param_id
            self.mav_eff_data_pck.param = controller.get_param(axis_id, param_id)


mavlink = Mavlink()
controller = Controller()
gimbal = Gimbal()
# Time range of the sensor
time_len = 300
time_wight = 0.8
time_head = -time_len * time_wight
time_tail = time_len * (1 - time_wight)
time_range = np.arange(time_head, time_tail, 1)
index_time_now = int(time_len*time_wight)
index_time_head = 0
index_time_tail = time_len - 1

fig = plt.Figure()
fig.patch.set_facecolor('gray')

ax_bg = 'black'
angle_lim = 100
angle_speed_lim = 10
attitude_window = fig.add_subplot(1, 1, 1, xlim=(time_head, time_tail), ylim=(-angle_lim, angle_lim),
                                  axisbg=ax_bg, title="Attitude")

# Set grid
attitude_window.grid()
gridlines = attitude_window.get_xgridlines() + attitude_window.get_ygridlines()
for line in gridlines:
    line.set_linestyle('-')
    line.set_alpha(0.5)




# Data to plot
roll_data = np.zeros(len(time_range))
pitch_data = np.zeros(time_len)
yaw_data = np.zeros(time_len)


# Line style
# Set color
# Line width
line_width = 1.5
line_alpha = 0.8
line_roll, = attitude_window.plot(time_range, roll_data, label="Roll", color="red",
                                  linewidth=line_width, alpha=line_alpha)
fill_roll_line = attitude_window.fill_between(time_range, 0, roll_data, facecolor='red', alpha=0.4)
line_pitch, = attitude_window.plot(time_range, pitch_data, label="Pitch", color="green",
                                   linewidth=line_width, alpha=line_alpha)
line_yaw, = attitude_window.plot(time_range, yaw_data, label="Yaw",
                                 linewidth = line_width, alpha=line_alpha)


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

    return line_roll, line_pitch, line_yaw

root = tk.Tk()

tk_var_roll = tk.StringVar()
tk_var_pitch = tk.StringVar()
tk_var_yaw = tk.StringVar()

tk_var_roll_rate_p = tk.StringVar()
tk_var_roll_rate_i = tk.StringVar()
tk_var_roll_rate_d = tk.StringVar()
tk_var_roll_rate_i_max = tk.StringVar()
tk_var_roll_ft_hz = tk.StringVar()
tk_var_roll_stab_p = tk.StringVar()
tk_var_roll_stab_d = tk.StringVar()

tk_var_pitch_rate_p = tk.StringVar()
tk_var_pitch_rate_i = tk.StringVar()
tk_var_pitch_rate_d = tk.StringVar()
tk_var_pitch_rate_i_max = tk.StringVar()
tk_var_pitch_ft_hz = tk.StringVar()
tk_var_pitch_stab_p = tk.StringVar()
tk_var_pitch_stab_d = tk.StringVar()

tk_var_yaw_rate_p = tk.StringVar()
tk_var_yaw_rate_i = tk.StringVar()
tk_var_yaw_rate_d = tk.StringVar()
tk_var_yaw_rate_i_max = tk.StringVar()
tk_var_yaw_ft_hz = tk.StringVar()
tk_var_yaw_stab_p = tk.StringVar()
tk_var_yaw_stab_d = tk.StringVar()


def set_tk_attitude_var():
    tk_var_roll.set(format(gimbal.attitude_euler[0], '.1f'))
    tk_var_pitch.set(format(gimbal.attitude_euler[1], '.1f'))
    tk_var_yaw.set(format(gimbal.attitude_euler[2], '.1f'))
#         rate_p    rate_i    rate_d   rate_i_max  rate_f_hz  stab_p    stab_d
#            0        1         2          3          4         5          6
def set_tk_control_param_var():
    tk_var_roll_rate_p.set(controller.get_param_str(0, 0))
    tk_var_roll_rate_i.set(controller.get_param_str(0, 1))
    tk_var_roll_rate_d.set(controller.get_param_str(0, 2))
    tk_var_roll_rate_i_max.set(controller.get_param_str(0, 3))
    tk_var_roll_ft_hz.set(controller.get_param_str(0, 4))
    tk_var_roll_stab_p.set(controller.get_param_str(0, 5))
    tk_var_roll_stab_d.set(controller.get_param_str(0, 6))

    tk_var_pitch_rate_p.set(controller.get_param_str(1, 0))
    tk_var_pitch_rate_i.set(controller.get_param_str(1, 1))
    tk_var_pitch_rate_d.set(controller.get_param_str(1, 2))
    tk_var_pitch_rate_i_max.set(controller.get_param_str(1, 3))
    tk_var_pitch_ft_hz.set(controller.get_param_str(1, 4))
    tk_var_pitch_stab_p.set(controller.get_param_str(1, 5))
    tk_var_pitch_stab_d.set(controller.get_param_str(1, 6))

    tk_var_yaw_rate_p.set(controller.get_param_str(2, 0))
    tk_var_yaw_rate_i.set(controller.get_param_str(2, 1))
    tk_var_yaw_rate_d.set(controller.get_param_str(2, 2))
    tk_var_yaw_rate_i_max.set(controller.get_param_str(2, 3))
    tk_var_yaw_ft_hz.set(controller.get_param_str(2, 4))
    tk_var_yaw_stab_p.set(controller.get_param_str(2, 5))
    tk_var_yaw_stab_d.set(controller.get_param_str(2, 6))

lab_width = 10
lab_root = tk.Label(root, text="GIMBAL MNG").grid(row=0, column=0, columnspan=19)
lab_roll_text = tk.Label(root, fg='red', text="Roll deg: ").grid(row=2, column=12, sticky='W')
lab_pitch_text = tk.Label(root, fg='green', text="Pitch deg: ").grid(row=3, column=12, sticky='W')
lab_yaw_text = tk.Label(root, fg='blue', text="Yaw deg: ").grid(row=4, column=12, sticky='W')
lab_null = tk.Label(root, text=' ').grid(row=1, column=11, rowspan=40)
lab_attitude_text = tk.Label(root, text="Attitude").grid(row=1, column=12, columnspan=8)
lab_ctrl_param_text = tk.Label(root, text="Control Parameter").grid(row=6, column=12, columnspan=8)

entry_width = 8
param_lab_width = 6

lab_roll_param_text = tk.Label(root, text="Roll Param: ").grid(row=7, column=12, columnspan=8, sticky='W')
lab_roll_rate_p = tk.Label(root, text="Rate P: ").grid(row=8, column=12, sticky='W')
ent_roll_rate_p = tk.Entry(root, textvariable=tk_var_roll_rate_p, width=entry_width)#grid(row=8, column=13, sticky='W')
lab_roll_rate_i = tk.Label(root, text="Rate I: ").grid(row=8, column=14, sticky='W')
ent_roll_rate_i = tk.Entry(root, textvariable=tk_var_roll_rate_i, width=entry_width)#.grid(row=8, column=15, sticky='W')
lab_roll_rate_d = tk.Label(root, text="Rate D: ").grid(row=8, column=16, sticky='W')
ent_roll_rate_d = tk.Entry(root, textvariable=tk_var_roll_rate_d, width=entry_width)#.grid(row=8, column=17, sticky='W')
lab_roll_rate_i_max = tk.Label(root, text="Rate Max I: ").grid(row=8, column=18, sticky='W')
ent_roll_rate_i_max = tk.Entry(root, textvariable=tk_var_roll_rate_i_max, width=entry_width)#.grid(row=8, column=19, sticky='W')
lab_roll_stab_p = tk.Label(root, text="Stabilize P: ").grid(row=9, column=12, sticky='W')
ent_roll_stab_p = tk.Entry(root, textvariable=tk_var_roll_stab_p, width=entry_width)#.grid(row=9, column=13, sticky='W')
lab_roll_stab_d = tk.Label(root, text="Stabilize D: ").grid(row=9, column=14, sticky='W')
ent_roll_stab_d = tk.Entry(root, textvariable=tk_var_roll_stab_d, width=entry_width)#.grid(row=9, column=15, sticky='W')
lab_roll_filter_hz = tk.Label(root, text="Filter Hz: ").grid(row=9, column=16, sticky='W')
ent_roll_filter_hz = tk.Entry(root, textvariable=tk_var_roll_ft_hz, width=entry_width)#.grid(row=9, column=17, sticky='W')

lab_pitch_param_text = tk.Label(root, text="Pitch Param: ").grid(row=10, column=12, columnspan=8, sticky='W')
lab_pitch_rate_p = tk.Label(root, text="Rate P: ").grid(row=11, column=12, sticky='W')
ent_pitch_rate_p = tk.Entry(root, textvariable=tk_var_pitch_rate_p, width=entry_width)#.grid(row=11, column=13, sticky='W')
lab_pitch_rate_i = tk.Label(root, text="Rate I: ").grid(row=11, column=14, sticky='W')
ent_pitch_rate_i = tk.Entry(root, textvariable=tk_var_pitch_rate_i, width=entry_width)#.grid(row=11, column=15, sticky='W')
lab_pitch_rate_d = tk.Label(root, text="Rate D: ").grid(row=11, column=16, sticky='W')
ent_pitch_rate_d = tk.Entry(root, textvariable=tk_var_pitch_rate_d, width=entry_width)#.grid(row=11, column=17, sticky='W')
lab_pitch_rate_i_max = tk.Label(root, text="Rate Max I: ").grid(row=11, column=18, sticky='W')
ent_pitch_rate_i_max = tk.Entry(root, textvariable=tk_var_pitch_rate_i_max, width=entry_width)#.grid(row=11, column=19, sticky='W')
lab_pitch_stab_p = tk.Label(root, text="Stabilize P: ").grid(row=12, column=12, sticky='W')
ent_pitch_stab_p = tk.Entry(root, textvariable=tk_var_pitch_stab_p, width=entry_width)#.grid(row=12, column=13, sticky='W')
lab_pitch_stab_d = tk.Label(root, text="Stabilize D: ").grid(row=12, column=14, sticky='W')
ent_pitch_stab_d = tk.Entry(root, textvariable=tk_var_pitch_stab_d, width=entry_width)#.grid(row=12, column=15, sticky='W')
lab_pitch_filter_hz = tk.Label(root, text="Filter Hz: ").grid(row=12, column=16, sticky='W')
ent_pitch_filter_hz = tk.Entry(root, textvariable=tk_var_pitch_ft_hz, width=entry_width)#.grid(row=12, column=17, sticky='W')

lab_yaw_param_text = tk.Label(root, text="Yaw Param: ").grid(row=13, column=12, columnspan=8, sticky='W')
lab_yaw_rate_p = tk.Label(root, text="Rate P: ").grid(row=14, column=12, sticky='W')
ent_yaw_rate_p = tk.Entry(root, textvariable=tk_var_yaw_rate_p, width=entry_width)#.grid(row=14, column=13, sticky='W')
lab_yaw_rate_i = tk.Label(root, text="Rate I: ").grid(row=14, column=14, sticky='W')
ent_yaw_rate_i = tk.Entry(root, textvariable=tk_var_yaw_rate_i, width=entry_width)#.grid(row=14, column=15, sticky='W')
lab_yaw_rate_d = tk.Label(root, text="Rate D: ").grid(row=14, column=16, sticky='W')
ent_yaw_rate_d = tk.Entry(root, textvariable=tk_var_yaw_rate_d, width=entry_width)#.grid(row=14, column=17, sticky='W')
lab_yaw_rate_i_max = tk.Label(root, text="Rate Max I: ").grid(row=14, column=18, sticky='W')
ent_yaw_rate_i_max = tk.Entry(root, textvariable=tk_var_yaw_rate_i_max, width=entry_width)#.grid(row=14, column=19, sticky='W')
lab_yaw_stab_p = tk.Label(root, text="Stabilize P: ").grid(row=15, column=12, sticky='W')
ent_yaw_stab_p = tk.Entry(root, textvariable=tk_var_yaw_stab_p, width=entry_width)#.grid(row=15, column=13, sticky='W')
lab_yaw_stab_d = tk.Label(root, text="Stabilize D: ").grid(row=15, column=14, sticky='W')
ent_yaw_stab_d = tk.Entry(root, textvariable=tk_var_yaw_stab_d, width=entry_width)#.grid(row=15, column=15, sticky='W')
lab_yaw_filter_hz = tk.Label(root, text="Filter Hz: ").grid(row=15, column=16, sticky='W')
ent_yaw_filter_hz = tk.Entry(root, textvariable=tk_var_yaw_ft_hz, width=entry_width)#.grid(row=15, column=17, sticky='W')

tk.Label(root, text='  ').grid(row=2, column=20)


serial_sleep_time = 0.01


def send_cmd(ax_id, param_id, value):
    controller.set_param(ax_id, param_id, value)
    mavlink.serial_send_buf(controller.pack_param(ax_id, param_id))
    time.sleep(serial_sleep_time)
    mavlink.serial_send_buf(controller.pack_param(ax_id, param_id))
    time.sleep(serial_sleep_time)
    mavlink.serial_send_buf(controller.pack_param(ax_id, param_id))


# Bind entries to callbacks
def ent_roll_rate_p_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(0, 0, value)

def ent_roll_rate_i_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(0, 1, value)

def ent_roll_rate_d_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(0, 2, value)


def ent_roll_rate_i_max_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(0, 3, value)


def ent_roll_ft_hz_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(0, 4, value)


def ent_roll_stab_p_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(0, 5, value)


def ent_roll_stab_d_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(0, 6, value)


def ent_pitch_rate_p_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(1, 0, value)


def ent_pitch_rate_i_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(1, 1, value)


def ent_pitch_rate_d_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(1, 2, value)


def ent_pitch_rate_i_max_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(1, 3, value)


def ent_pitch_ft_hz_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(1, 4, value)


def ent_pitch_stab_p_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(1, 5, value)


def ent_pitch_stab_d_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(1, 6, value)


def ent_yaw_rate_p_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(2, 0, value)


def ent_yaw_rate_i_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(2, 1, value)


def ent_yaw_rate_d_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(2, 2, value)


def ent_yaw_rate_i_max_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(2, 3, value)


def ent_yaw_ft_hz_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(2, 4, value)


def ent_yaw_stab_p_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(2, 5, value)


def ent_yaw_stab_d_callback(var):
    value = float(var.get())
    #print(value)
    send_cmd(2, 6, value)


# Binding Callback function
def binding():
    ent_roll_rate_p.bind('<Return>', (lambda _: ent_roll_rate_p_callback(ent_roll_rate_p)))
    ent_roll_rate_i.bind('<Return>', (lambda _: ent_roll_rate_i_callback(ent_roll_rate_i)))
    ent_roll_rate_d.bind('<Return>', (lambda _: ent_roll_rate_d_callback(ent_roll_rate_d)))
    ent_roll_rate_i_max.bind('<Return>', (lambda _: ent_roll_rate_i_max_callback(ent_roll_rate_i_max)))
    ent_roll_filter_hz.bind('<Return>', (lambda _: ent_roll_ft_hz_callback(ent_roll_filter_hz)))
    ent_roll_stab_p.bind('<Return>', (lambda _: ent_roll_stab_p_callback(ent_roll_stab_p)))
    ent_roll_stab_d.bind('<Return>', (lambda _: ent_roll_stab_d_callback(ent_roll_stab_d)))

    ent_pitch_rate_p.bind('<Return>', (lambda _: ent_pitch_rate_p_callback(ent_pitch_rate_p)))
    ent_pitch_rate_i.bind('<Return>', (lambda _: ent_pitch_rate_i_callback(ent_pitch_rate_i)))
    ent_pitch_rate_d.bind('<Return>', (lambda _: ent_pitch_rate_d_callback(ent_pitch_rate_d)))
    ent_pitch_rate_i_max.bind('<Return>', (lambda _: ent_pitch_rate_i_max_callback(ent_pitch_rate_i_max)))
    ent_pitch_filter_hz.bind('<Return>', (lambda _: ent_pitch_ft_hz_callback(ent_pitch_filter_hz)))
    ent_pitch_stab_p.bind('<Return>', (lambda _: ent_pitch_stab_p_callback(ent_pitch_stab_p)))
    ent_pitch_stab_d.bind('<Return>', (lambda _: ent_pitch_stab_d_callback(ent_pitch_stab_d)))

    ent_yaw_rate_p.bind('<Return>', (lambda _: ent_yaw_rate_p_callback(ent_yaw_rate_p)))
    ent_yaw_rate_i.bind('<Return>', (lambda _: ent_yaw_rate_i_callback(ent_yaw_rate_i)))
    ent_yaw_rate_d.bind('<Return>', (lambda _: ent_yaw_rate_d_callback(ent_yaw_rate_d)))
    ent_yaw_rate_i_max.bind('<Return>', (lambda _: ent_yaw_rate_i_max_callback(ent_yaw_rate_i_max)))
    ent_yaw_filter_hz.bind('<Return>', (lambda _: ent_yaw_ft_hz_callback(ent_yaw_filter_hz)))
    ent_yaw_stab_p.bind('<Return>', (lambda _: ent_yaw_stab_p_callback(ent_yaw_stab_p)))
    ent_yaw_stab_d.bind('<Return>', (lambda _: ent_yaw_stab_d_callback(ent_yaw_stab_d)))

binding()


# Grid
def griding():
    ent_roll_rate_p.grid(row=8, column=13, sticky='W')
    ent_roll_rate_i.grid(row=8, column=15, sticky='W')
    ent_roll_rate_d.grid(row=8, column=17, sticky='W')
    ent_roll_rate_i_max.grid(row=8, column=19, sticky='W')
    ent_roll_stab_p.grid(row=9, column=13, sticky='W')
    ent_roll_stab_d.grid(row=9, column=15, sticky='W')
    ent_roll_filter_hz.grid(row=9, column=17, sticky='W')

    ent_pitch_rate_p.grid(row=11, column=13, sticky='W')
    ent_pitch_rate_i.grid(row=11, column=15, sticky='W')
    ent_pitch_rate_d.grid(row=11, column=17, sticky='W')
    ent_pitch_rate_i_max.grid(row=11, column=19, sticky='W')
    ent_pitch_stab_p.grid(row=12, column=13, sticky='W')
    ent_pitch_stab_d.grid(row=12, column=15, sticky='W')
    ent_pitch_filter_hz.grid(row=12, column=17, sticky='W')

    ent_yaw_rate_p.grid(row=14, column=13, sticky='W')
    ent_yaw_rate_i.grid(row=14, column=15, sticky='W')
    ent_yaw_rate_d.grid(row=14, column=17, sticky='W')
    ent_yaw_rate_i_max.grid(row=14, column=19, sticky='W')
    ent_yaw_stab_p.grid(row=15, column=13, sticky='W')
    ent_yaw_stab_d.grid(row=15, column=15, sticky='W')
    ent_yaw_filter_hz.grid(row=15, column=17, sticky='W')

griding()


lab_roll_var = tk.Label(root, fg='red', textvariable=tk_var_roll).grid(row=2, column=13, sticky='W')
lab_pitch_var = tk.Label(root, fg='green', textvariable=tk_var_pitch).grid(row=3, column=13, sticky='W')
lab_yaw_var = tk.Label(root, fg='blue', textvariable=tk_var_yaw).grid(row=4, column=13, sticky='W')

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().grid(row=1, column=0, sticky='E', rowspan=40, columnspan=10)

def closing():
    can.close()
    root.destroy()

root.protocol('WM_DELETE_WINDOW', closing)




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

    return line_roll, line_pitch, line_yaw


# STEP 3
def get_data_1():
    global roll_data
    global pitch_data
    global yaw_data
    global roll_speed_data
    global pitch_speed_data
    global yaw_speed_data

    global mavlink
    global controller
    global gimbal

    while True:
        mavlink.serial2mavlink()
        ready = mavlink.decode_msg(controller)
        if ready:
            gimbal.update_attitude(mavlink)
            roll_data = np.append(np.delete(roll_data, 0), gimbal.attitude_euler[0])
            pitch_data = np.append(np.delete(pitch_data, 0), gimbal.attitude_euler[1])
            yaw_data = np.append(np.delete(yaw_data, 0), gimbal.attitude_euler[2])


# GUI
def lab_change():
    #global var_roll
    while True:
        set_tk_attitude_var()
        #set_tk_control_param_var()
        time.sleep(0.01)
        #root.update()

def param_change():
    while True:
        set_tk_control_param_var()
        time.sleep(1)

can = can_pro.CAN4Gimbal()
can.can_open()

def can_update():
    while True:
        can.high_level_read()
        #time.sleep(0.01)


threads = []
t1 = threading.Thread(target=get_data_1)
t2 = threading.Thread(target=lab_change)
t3 = threading.Thread(target=param_change)
can_thread = threading.Thread(target=can_update)
threads.append(t1)
threads.append(t2)
threads.append(t3)
threads.append(can_thread)


def data_init():
    for i in range(1000):
        mavlink.serial2mavlink()
        mavlink.decode_msg(controller)
        set_tk_attitude_var()
        set_tk_control_param_var()

if __name__ == '__main__':
    data_init()
    for t in threads:
        t.setDaemon(True)
        t.start()
    anim = animation.FuncAnimation(fig, animate, init_func=plot_init,
                                   interval=5, blit=True)
    root.mainloop()






















