import serial
from enum import Enum
import struct
import euclid3
import math
import numpy as np


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
MAV_COMP_ID_ROLL_MOTOR = 71
MAV_COMP_ID_PITCH_MOTOR = 72
MAV_COMP_ID_YAW_MOTOR = 73

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

MAVLINK_MSG_ID_CM_RT_DATA = 79
MAVLINK_MSG_ID_CM_RT_DATA_LEN = 48
MAVLINK_MSG_ID_CM_RT_DATA_CRC = 49

MAVLINK_MSG_ID_MOTOR_STATE_DATA = 80
MAVLINK_MSG_ID_MOTOR_STATE_DATA_LEN = 32
MAVLINK_MSG_ID_MOTOR_STATE_DATA_CRC = 50

# Controller config data info
MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA = 81
MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA_LEN = 84
MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA_CRC = 51

# Motors config data info
MAVLINK_MSG_ID_MOTORS_CONFIG_DATA = 82
MAVLINK_MSG_ID_MOTORS_CONFIG_DATA_LEN = 240
MAVLINK_MSG_ID_MOTORS_CONFIG_DATA_CRC = 52

# Write cmd data info
MAVLINK_MSG_ID_WRITE_PARAM_DATA = 83
MAVLINK_MSG_ID_WRITE_PARAM_DATA_LEN = 8
MAVLINK_MSG_ID_WRITE_PARAM_DATA_CRC = 53

# Read cmd data info
MAVLINK_MSG_ID_READ_PARAM_DATA = 84
MAVLINK_MSG_ID_READ_PARAM_DATA_LEN = 8
MAVLINK_MSG_ID_READ_PARAM_DATA_CRC = 54

MAVLINK_ATTITUDE_QUAT_TYPE = 0
MAVLINK_MOTOR_STATE_TYPE = 1
MAVLINK_CONTROLLER_CONFIG_TYPE = 2
MAVLINK_MOTOR_CONFIG_TYPE = 3
MAVLINK_WRITE_PARAM_TYPE = 4
MAVLINK_READ_PARAM_TYPE = 5

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
            self.attitude_quaternion.w = mavlink.mav_cm_rt_data_pck.q0
            self.attitude_quaternion.x = mavlink.mav_cm_rt_data_pck.q1
            self.attitude_quaternion.y = mavlink.mav_cm_rt_data_pck.q2
            self.attitude_quaternion.z = mavlink.mav_cm_rt_data_pck.q3
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


class PckInfo:
    def __init__(self):
        self.component_id = MAV_COMP_ID_CONTROLLER
        self.data_type = MAVLINK_ATTITUDE_QUAT_TYPE
        self.write = 0
        self.multi_param = 0


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

    def packing(self):
        msg = bytearray()
        # Head
        msg.append(0xFE)
        # Data len
        msg.append(MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN)
        # Data seq
        msg.append(0)
        # System ID
        msg.append(MAV_TYPE_GIMBAL)
        # Component ID
        msg.append(MAV_COMP_ID_CONTROLLER)
        # Message ID
        msg.append(MAVLINK_MSG_ID_ATTITUDE_QUATERNION)

        msg = msg + struct.pack('i', self.time_boot_ms)
        msg = msg + struct.pack('f', self.q1)
        msg = msg + struct.pack('f', self.q2)
        msg = msg + struct.pack('f', self.q3)
        msg = msg + struct.pack('f', self.q4)
        msg = msg + struct.pack('f', self.roll_speed)
        msg = msg + struct.pack('f', self.pitch_speed)
        msg = msg + struct.pack('f', self.yaw_speed)
        msg.append(0)
        msg.append(0)
        return msg


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


class MavCtrlMotRealtimePck:
    def __init__(self):
        self.q0 = 1
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.ntc_tempre = 0
        self.avg_motor_current = 0
        self.avg_input_current = 0
        self.duty_cycle_now = 0
        self.rpm = 0
        self.input_voltage = 0
        self.tachometer_value = 0
        self.tachometer_abs_value = 0


class MavControllerConfigPck:
    def __init__(self):
        self.params = np.zeros(21)

    def packing(self):
        msg = bytearray()
        # Head
        msg.append(0xFE)
        # Data len
        msg.append(MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA_LEN)
        # Data seq
        msg.append(0)
        # System ID
        msg.append(MAV_TYPE_GIMBAL)
        # Component ID
        msg.append(MAV_COMP_ID_CONTROLLER)
        # Message ID
        msg.append(MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA)
        for param in self.params:
            msg = msg + struct.pack('f', param)
        msg.append(0)
        msg.append(0)
        return msg


class MavMotorsConfigPck:
    def __init__(self):
        self.params = np.zeros(60)

    def packing(self):
        msg = bytearray()
        # Head
        msg.append(0xFE)
        # Data len
        msg.append(MAVLINK_MSG_ID_MOTORS_CONFIG_DATA_LEN)
        # Data seq
        msg.append(0)
        # System ID
        msg.append(MAV_TYPE_GIMBAL)
        # Component ID
        msg.append(MAV_COMP_ID_ROLL_MOTOR)
        # Message ID
        msg.append(MAVLINK_MSG_ID_MOTORS_CONFIG_DATA)
        for param in self.params:
            msg = msg + struct.pack('f', param)
        msg.append(0)
        msg.append(0)
        return msg


class MavParamPck:
    def __init__(self):
        self.param_id = 0
        self.param_value = 0
        self.dev_id = MAV_COMP_ID_CONTROLLER

    def packing_write(self):
        msg = bytearray()
        # Head
        msg.append(0xFE)
        # Data len
        msg.append(MAVLINK_MSG_ID_WRITE_PARAM_DATA_LEN)
        # Data seq
        msg.append(0)
        # System ID
        msg.append(MAV_TYPE_GIMBAL)
        # Component ID
        msg.append(self.dev_id)
        # Message ID
        msg.append(MAVLINK_MSG_ID_WRITE_PARAM_DATA)
        msg = msg + struct.pack('i', self.param_id)
        msg = msg + struct.pack('f', self.param_value)
        msg.append(0)
        msg.append(0)
        return msg

    def packing_read(self):
        msg = bytearray()
        # Head
        msg.append(0xFE)
        # Data len
        msg.append(MAVLINK_MSG_ID_READ_PARAM_DATA_LEN)
        # Data seq
        msg.append(0)
        # System ID
        msg.append(MAV_TYPE_GIMBAL)
        # Component ID
        msg.append(self.dev_id)
        # Message ID
        msg.append(MAVLINK_MSG_ID_READ_PARAM_DATA)
        msg = msg + struct.pack('i', self.param_id)
        msg = msg + struct.pack('f', self.param_value)
        msg.append(0)
        msg.append(0)
        return msg


class Mavlink:
    def __init__(self):
        self.ser = serial.Serial("COM6", 115200, timeout=1)
        self.msg_state = MsgState.MAVLINK_RX_STOP
        self.msg_rs = MavMsg()
        self.msg_tx = MavMsg()
        self.msg_data_index = 0
        self.mav_attitude_pck = MavAttitudePck()
        self.mav_attitude_quat_pck = MavAttitudeQuatPck()
        self.mav_control_param_pck = MavControlParameterPck()
        self.mav_eff_data_pck = MavEfficientDataPck()
        self.mav_cm_rt_data_pck = MavCtrlMotRealtimePck()
        self.tx_pck_info = PckInfo()
        self.rx_pck_info = PckInfo()
        self.mav_controller_config_pck = MavControllerConfigPck()
        self.mav_controller_config_pck_to_gimbal = MavControllerConfigPck()
        self.mav_motors_config_pck = MavMotorsConfigPck()
        self.mav_motors_config_pck_to_gimbal = MavMotorsConfigPck()
        self.mav_write_param_pck = MavParamPck()
        self.mav_read_param_pck = MavParamPck()


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
                #print('payload len', self.msg_rs.payload_len)
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
                #print('msg id', self.msg_rs.msg_id)
                self.msg_rs.data.clear()
            elif self.msg_state == MsgState.MAVLINK_RX_MSGID:
                self.msg_rs.data.append(buf)
                self.msg_data_index += 1
                if self.msg_data_index == self.msg_rs.payload_len:
                    self.msg_data_index = 0
                    self.msg_state = MsgState.MAVLINK_RX_DATA
                    #print(self.msg_rs.data)
            elif self.msg_state == MsgState.MAVLINK_RX_DATA:
                self.msg_state = MsgState.MAVLINK_RX_CRCL
                self.msg_rs.crc_l = ord(buf)
            elif self.msg_state == MsgState.MAVLINK_RX_CRCL:
                self.msg_state = MsgState.MAVLINK_RX_CRCH
                self.msg_rs.crc_h = ord(buf)
                self.msg_state = MsgState.MAVLINK_RX_STOP
        return len(buf)

    def mavlink_read_attitude_quat(self):
        msg = self.mav_attitude_quat_pck.packing()
        self.serial_send_buf(msg)

    def mavlink_read_param(self, dev_id, param_id):
        self.mav_read_param_pck.dev_id = dev_id
        self.mav_read_param_pck.param_id = param_id
        msg = self.mav_read_param_pck.packing_read()
        self.serial_send_buf(msg)

    def mavlink_write_param(self, dev_id, param_id, param_value):
        self.mav_write_param_pck.dev_id = dev_id
        self.mav_write_param_pck.param_id = param_id
        self.mav_write_param_pck.param_value = param_value
        msg = self.mav_write_param_pck.packing_write()
        self.serial_send_buf(msg)

    def mavlink_write_controller_config(self):
        msg = self.mav_controller_config_pck_to_gimbal.packing()
        msg[6] = 1
        print(msg)
        self.serial_send_buf(msg)

    def mavlink_read_controller_config(self):
        msg = self.mav_controller_config_pck_to_gimbal.packing()
        msg[6] = 0
        print(msg)
        self.serial_send_buf(msg)

    def mavlink_write_motors_config(self):
        msg = self.mav_motors_config_pck_to_gimbal.packing()
        msg[6] = 1
        for i in range(10):
            msg[6+i] = i
        self.serial_send_buf(msg)

    def mavlink_read_motors_config(self):
        msg = self.mav_motors_config_pck_to_gimbal.packing()
        msg[6] = 0
        self.serial_send_buf(msg)

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
                return MAVLINK_MSG_ID_ATTITUDE
            else:
                return -1
        else:
            return -1

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
                return MAVLINK_MSG_ID_ATTITUDE_QUATERNION
            else:
                return -1
        else:
            return -1

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
                    return MAVLINK_MSG_ID_EFF_DATA
                else:
                    return -1
            else:
                return -1
        else:
            return -1

    def decode_cm_rt_data(self):
        #print('Controller motor realtime message')
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            if self.msg_rs.payload_len == MAVLINK_MSG_ID_CM_RT_DATA_LEN:
                self.mav_cm_rt_data_pck.q0 = byte2float6(self.msg_rs.data, 0)
                self.mav_cm_rt_data_pck.q1 = byte2float6(self.msg_rs.data, 4)
                self.mav_cm_rt_data_pck.q2 = byte2float6(self.msg_rs.data, 8)
                self.mav_cm_rt_data_pck.q3 = byte2float6(self.msg_rs.data, 12)
                self.mav_cm_rt_data_pck.ntc_tempre = byte2float4(self.msg_rs.data, 16)
                self.mav_cm_rt_data_pck.avg_motor_current = byte2float4(self.msg_rs.data, 20)
                self.mav_cm_rt_data_pck.avg_input_current = byte2float4(self.msg_rs.data, 24)
                self.mav_cm_rt_data_pck.duty_cycle_now = byte2float4(self.msg_rs.data, 28)
                self.mav_cm_rt_data_pck.rpm = byte2float4(self.msg_rs.data, 32)
                self.mav_cm_rt_data_pck.input_voltage = byte2float4(self.msg_rs.data, 36)
                self.mav_cm_rt_data_pck.tachometer_value = byte2float4(self.msg_rs.data, 40)
                self.mav_cm_rt_data_pck.tachometer_abs_value = byte2float4(self.msg_rs.data, 44)
                return MAVLINK_MSG_ID_CM_RT_DATA
            else:
                return -1
        else:
            return -1

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
                return MAVLINK_MSG_ID_CONTROL_PARAM
            else:
                return -1
        else:
            return -1

    def decode_controller_config_data(self):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            if self.msg_rs.payload_len == MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA_LEN:
                for i in range(int(MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA_LEN / 4)):
                    self.mav_controller_config_pck.params[i] = byte2float(self.msg_rs.data, 4*i)
                print(self.mav_controller_config_pck.params)
                return MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA
            else:
                return -1
        else:
            return -1

    def decode_motors_config_data(self):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            if self.msg_rs.payload_len == MAVLINK_MSG_ID_MOTORS_CONFIG_DATA:
                for i in range(int(MAVLINK_MSG_ID_MOTORS_CONFIG_DATA_LEN/4)):
                    self.mav_motors_config_pck.params[i] = byte2float(self.msg_rs.data, 4*i)
                return MAVLINK_MSG_ID_MOTORS_CONFIG_DATA
            else:
                return -1
        else:
            return -1

    def decode_write_param_pck(self):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            if self.msg_rs.payload_len == MAVLINK_MSG_ID_WRITE_PARAM_DATA_LEN:
                self.mav_write_param_pck.param_id = byte2uint32(self.msg_rs.data, 0)
                self.mav_write_param_pck.param_value = byte2float(self.msg_rs.data, 4)
                self.mav_write_param_pck.dev_id = self.msg_rs.comp_id
                return MAVLINK_MSG_ID_WRITE_PARAM_DATA
            else:
                return -1
        else:
            return -1

    def decode_read_param_pck(self):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            if self.msg_rs.payload_len == MAVLINK_MSG_ID_READ_PARAM_DATA_LEN:
                self.mav_read_param_pck.param_id = byte2uint32(self.msg_rs.data, 0)
                self.mav_read_param_pck.param_value = byte2float(self.msg_rs.data, 4)
                self.mav_read_param_pck.dev_id = self.msg_rs.comp_id
                return MAVLINK_MSG_ID_READ_PARAM_DATA
            else:
                return -1
        else:
            return -1

    # Return True if decode success
    def decode_msg(self, controller):
        if self.msg_state == MsgState.MAVLINK_RX_STOP:
            #print(hex(self.msg_rs.msg_id))
            if self.msg_rs.msg_id == MAVLINK_MSG_ID_EFF_DATA:
                if isinstance(controller, Controller):
                    return self.decode_eff_data(controller)
                else:
                    return -1
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_ATTITUDE:
                return self.decode_attitude()
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_CONTROL_PARAM:
                return self.decode_control_parameter()
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                return self.decode_attitude_quaternion()
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_CM_RT_DATA:
                return self.decode_cm_rt_data()
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA:
                return self.decode_controller_config_data()
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_MOTORS_CONFIG_DATA:
                return self.decode_motors_config_data()
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_WRITE_PARAM_DATA:
                return self.decode_write_param_pck()
            elif self.msg_rs.msg_id == MAVLINK_MSG_ID_READ_PARAM_DATA:
                return self.decode_read_param_pck()
        else:
            return -1

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