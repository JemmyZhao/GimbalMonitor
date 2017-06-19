import ch_can
import struct
import time

# MSG ID:  11 bits
#    Priority | Source Address | Destination Address | Write Cmd | Long Msg | Head
#       2     |        3       |           3         |     1     |     1    |   1
#    0    0   |   0   0   0    |       0   0   0     |     0     |     0    |   0

WRITE = 1
READ = 0

PRIORITY_CONTROL = (1<<9)
PRIORITY_ENCODER = (2<<9)
PRIORITY_RQRP = (3<<9)

CONTROLLER_ADDR = 0
ROLL_MOTOR_ADDR = 1
PITCH_MOTOR_ADDR = 2
YAW_MOTOR_ADDR = 3

WRITE_RQRP = PRIORITY_RQRP | WRITE
READ_RQRP = PRIORITY_RQRP | READ

# Motor control
RPY_CTRL =                      0x20c
RP_CTRL =                       0x214
Y_CTRL =                        0x21c

# Encoder
ROLL_ENC =                      0x444
PITCH_ENC =                     0x484
YAW_ENC =                       0x4c4
MCU_WS_ROLL =                   0x60C
MCU_WS_PITCH =                  0x614
MCU_WS_YAW =                    0x61C
# Motors reply single message to MCU
ROLL_RPS_MCU =                  0x644
PITCH_RPS_MCU =                 0x684
YAW_RPS_MCU =                   0x6C4
# MCU write long message to Motors
MCU_WL_ROLL_NORMAL =            0x60E
MCU_WL_ROLL_HEAD =              0x60F
MCU_WL_PITCH_NORMAL =           0x616
MCU_WL_PITCH_HEAD =             0x617
MCU_WL_YAW_NORMAL =             0x61E
MCU_WL_YAW_HEAD =               0x61F
# Motor reply long message to MCU
ROLL_RPL_MCU_NORMAL =           0x646
ROLL_RPL_MCU_HEAD =             0x647
PITCH_RPL_MCU_NORMAL =          0x686
PITCH_RPL_MCU_HEAD =            0x687
YAW_RPL_MCU_NORMAL =            0x6C6
YAW_RPL_MCU_HEAD =              0x6C7

# RQRP Read
# MCU read single message from Motors
MCU_RS_ROLL =                   0x608
MCU_RS_PITCH =                  0x610
MCU_RS_YAW =                    0x618
# Motors read single message from MCU
ROLL_RS_MCU =                   0x640
PITCH_RS_MCU =                  0x680
YAW_RS_MCU =                    0x6C0
# MCU read long message from Motors
MCU_RL_ROLL =                   0x60A
MCU_RL_PITCH =                  0x612
MCU_RL_YAW =                    0x61A
# Motors read long message from MCU
ROLL_RL_MCU =                   0x642
PITCH_RL_MCU =                  0x682
YAW_RL_MCU =                    0x6C2



CURRENT_CTRL_KP = 0
CURRENT_CTRL_KI = 1
ENCODER_OFS = 2
ENCODER_RAT = 3
SPEED_TRACKER_KP = 4
SPEED_TRACKER_KI = 5
DUTY_DOWNRAMP_KP = 6
DUTY_DOWNRAMP_KI = 7
F_SW = 8
DTC = 9
POS_CTRL_KP = 10
POS_CTRL_KI = 11
POS_CTRL_KD = 12
POS_CTRL_ANGLE_DIVISION = 13
SPEED_CTRL_KP = 14
SPEED_CTRL_KI = 15
SPEED_CTRL_KD = 16
SPEED_CRTL_MIN_ERPM = 17
FAULT_STOP_TIME = 18

TACHO = 19
TACHO_ABS = 20
BUS_VOLTAGE = 21
DUTY_CYCLE = 22
ELECTRICAL_SPEED = 23
BUS_CURRENT = 24
MOTOR_CURRENT = 25
MOSFET_TEMP = 26
FAULT_CODE = 27

OFS = 28
RAT = 29
I = 30

MEASURE_R = 31
MEASURE_L = 32
DUTY_TEST = 33
POS_TEST = 34
CALC_CC_TC = 35
CALC_CC_KP = 36
CALC_CC_KI = 37
CURRENT_TEST = 38


class MotConfig:
    def __init__(self):
        self.current_control_kp = 0
        self.current_control_ki = 0
        self.encoder = 0
        self.speed_tracker_kp = 0
        self.speed_tracker_ki = 0
        self.duty_downramp_kp = 0
        self.duty_downramp_ki = 0
        self.f_sw = 0
        self.dtc = 0
        self.position_control_kp = 0
        self.position_control_ki = 0
        self.position_control_kd = 0
        self.position_control_angle_division = 0
        self.speed_control_kp = 0
        self.speed_control_ki = 0
        self.speed_control_kd = 0
        self.speed_control_min_erpm = 0
        self.fault_stop_time = 0


class MotRealData:
    def __init__(self):
        self.tacho = 0
        self.tacho_abs = 0
        self.bus_voltage = 0
        self.duty_cycle = 0
        self.electrical_speed = 0
        self.bus_current = 0
        self.motor_current = 0
        self.mosfet_temp = 0
        self.fault_code = 0


class MotDetectEncoder:
    def __init__(self):
        self.ofs = 0
        self.rat = 0
        self.current = 0


class MotDetectAndCal:
    def __init__(self):
        self.measure_r = 0
        self.measure_l = 0
        self.duty_test = 0
        self.position_test = 0
        self.tc = 0
        self.kp = 0
        self.ki = 0
        self.current_test = 0


class MotorState:
    def __init__(self):
        self.control = 0
        self.encoder = 0


class MsgID:
    def __init__(self):
        self.priority = 0
        self.source = 0
        self.destination = 0
        self.write = 0
        self.long_msg = 0
        self.head = 0


class Msg:
    def __init__(self):
        self.id = 0
        self.data = bytearray(8)


class CanParam:
    def __init__(self, paramid=0, value=0):
        self.id = paramid
        self.value = value
        self.dev_addr = 0
        self.source_addr = 0


MAX_MSGS = 30


class CanBlock:
    def __init__(self):
        self.id = 0
        self.msg_totle = 0
        self.params = []
        self.msg_cnt = 0
        self.dev_addr = 0
        self.source_addr = 0


class CanDevice:
    def __init__(self, add=0, p_t=0, b_t=0):
        self.addr = add
        self.tx_msg = Msg()
        self.rx_msg = Msg()
        self.params_totle = p_t
        self.blocks_totle = b_t
        self.params = [CanParam()]*self.params_totle
        self.blocks = [CanBlock()]*self.blocks_totle

    def rx_param_map(self, param):
        if param.dev_addr != self.addr:
            return False
        if isinstance(param, CanParam):
            self.params[param.id] = param
            return True
        else:
            return False

    def tx_param_map(self, dst_add, param_index):
        param = self.params[param_index]
        param.source_addr = self.addr
        param.dev_addr = dst_add
        return param


class CanMotor(CanDevice):
    def __init__(self, add=1):
        CanDevice.__init__(self, add, 39, 0)
        self.enc = 0
        self.control = 0


class CanMCU(CanDevice):
    def __init__(self, add=0):
        CanDevice.__init__(self, add, 0, 0)


class CAN4Gimbal():
    def __init__(self, devType=3, devInd=0, canInd=0):
        self.canbus = ch_can.CAN(devType, devInd, canInd)
        self.rx_msg = Msg()
        self.tx_msg = Msg()
        self.rx_param = CanParam()
        self.tx_param = CanParam()
        self.rx_block = CanBlock()
        self.tx_block = CanBlock()

        self.roll_ctrl = 0
        self.pitch_ctrl = 0
        self.yaw_ctrl = 0

        self.roll_enc = 0
        self.pitch_enc = 0
        self.yaw_enc = 0

        self.mcu = CanMCU()
        self.roll_motor = CanMotor(1)
        self.pitch_motor = CanMotor(2)
        self.yaw_motor = CanMotor(3)

    def rx_param_map(self):
        if self.rx_param.dev_addr == CONTROLLER_ADDR:
            self.mcu.rx_param_map(self.rx_param)
        elif self.rx_param.dev_addr == ROLL_MOTOR_ADDR:
            self.roll_motor.rx_param_map(self.rx_param)
        elif self.rx_param.dev_addr == PITCH_MOTOR_ADDR:
            self.pitch_motor.rx_param_map(self.rx_param)
        elif self.rx_param.dev_addr == YAW_MOTOR_ADDR:
            self.yaw_motor.rx_param_map(self.rx_param)

    def tx_param_map(self, src_addr, dst_addr, param_id):
        if src_addr == CONTROLLER_ADDR:
            self.tx_param = self.mcu.tx_param_map(dst_add=dst_addr, param_index=param_id)
        if src_addr == ROLL_MOTOR_ADDR:
            self.tx_param = self.roll_motor.tx_param_map(dst_add=dst_addr, param_index=param_id)
        if src_addr == PITCH_MOTOR_ADDR:
            self.tx_param = self.pitch_motor.tx_param_map(dst_add=dst_addr, param_index=param_id)
        if src_addr == YAW_MOTOR_ADDR:
            self.tx_param = self.yaw_motor.tx_param_map(dst_add=dst_addr, param_index=param_id)

    def can_open(self):
        return self.canbus.open()

    def can_read(self):
        rs = self.canbus.read()
        if rs > 0:
            self.rx_msg.id = self.canbus.rxo_p[0].ID
            self.rx_msg.data = bytes(self.canbus.rxo_p[0].Data)
            #print('Msg ID: ', self.rx_msg.id, '\tType: ', type(self.rx_msg.id))
        return rs

    def can_write_param(self):
        self.tx_msg.id = WRITE_RQRP | (self.tx_param.source_addr << 6) | (self.tx_param.dev_addr << 3)
        self.tx_msg.data = struct.pack('i', self.tx_param.id) + struct.pack('f', self.tx_param.value)
        self.can_write()

    def can_write(self):
        self.canbus.txo_p[0].ID = self.tx_msg.id
        self.canbus.txo_p[0].Data = (ch_can.BYTE*8)(*[ch_can.BYTE(d) for d in self.tx_msg.data[:8]])
        return self.canbus.write()

    def close(self):
        return self.canbus.close()

    def decode_rpy_ctrl(self, msg):
        self.roll_ctrl = struct.unpack('h', msg.data[:2])[0]
        self.pitch_ctrl = struct.unpack('h', msg.data[2:4])[0]
        self.yaw_ctrl = struct.unpack('h', msg.data[4:6])[0]

    def decode_rp_ctrl(self, msg):
        self.roll_ctrl = struct.unpack('f', msg.data[:4])[0]
        self.pitch_ctrl = struct.unpack('f', msg.data[4:])[0]
        #print('Roll Ctrl: ', format(self.roll_ctrl, '.4f'), '\t Pitch Ctrl: ', format(self.pitch_ctrl, '.4f'))

    def decode_y_ctrl(self, msg):
        self.yaw_ctrl = struct.unpack('f', msg.data[:4])[0]
        #print('Yaw Ctrl: ', self.yaw_ctrl)

    def decode_roll_enc(self, msg):
        self.roll_enc = struct.unpack('f', msg.data[:4])[0]
        #print('Roll Enc: ', self.roll_enc)

    def decode_pitch_enc(self, msg):
        self.pitch_enc = struct.unpack('f', msg.data[:4])[0]
        #print('Pitch Enc: ', self.pitch_enc)

    def decode_yaw_enc(self, msg):
        self.yaw_enc = struct.unpack('f', msg.data[:4])[0]
        #print('Yaw Enc: ', self.yaw_enc)

    def decode_rqrp_param(self, msg):
        self.rx_param.id = (struct.unpack('i', msg.data[:4]))[0]
        self.rx_param.value = (struct.unpack('f', msg.data[4:]))[0]
        print('Param ID: ', self.rx_param.id, '\t Param Value: ', self.rx_param.value)

    def decode_rqrp_block_head(self, msg):
        self.rx_block.id = struct.unpack('i', msg.data[:4])[0]
        self.rx_block.msg_totle = struct.unpack('i', msg.data[4:])[0]
        self.rx_block.msg_cnt = 0
        self.rx_block.params = [CanParam()]
        print('Block ID: ', self.rx_block.id, '\t Totle Msg: ', self.rx_block.msg_totle)

    def decode_rqrp_block_normal(self, msg):
        self.rx_block.params.append(CanParam(0, struct.unpack('f', msg.data[:4])[0]))
        self.rx_block.params.append(CanParam(0, struct.unpack('f', msg.data[4:])[0]))
        print('Param Cnt: ', self.rx_block.msg_cnt,
              'Param i0: ', self.rx_block.params[2*self.rx_block.msg_cnt].value,
              '\t Param i1', self.rx_block.params[2*self.rx_block.msg_cnt+1].value)
        self.rx_block.msg_cnt = self.rx_block.msg_cnt + 1

    def can_decode(self, msg):
        if isinstance(msg, Msg):
            if msg.id == RPY_CTRL:
                self.decode_rpy_ctrl(msg)
                #print('RPY Control')
            elif msg.id == RP_CTRL:
                self.decode_rp_ctrl(msg)
                #print('RP Control')
            elif msg.id == Y_CTRL:
                self.decode_y_ctrl(msg)
                #print('Y Control')
            elif msg.id == ROLL_ENC:
                self.decode_roll_enc(msg)
            elif msg.id == PITCH_ENC:
                self.decode_pitch_enc(msg)
            elif msg.id == YAW_ENC:
                self.decode_yaw_enc(msg)
            elif msg.id == MCU_WS_ROLL:
                print('MCU Write single Roll:')
                self.rx_param.dev_addr = ROLL_MOTOR_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == MCU_WS_PITCH:
                print('MCU Write single Pitch:')
                self.rx_param.dev_addr = PITCH_MOTOR_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == MCU_WS_YAW:
                print('MCU Write single Yaw:')
                self.rx_param.dev_addr = YAW_MOTOR_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == ROLL_RPS_MCU:
                print('Roll Reply single MCU:')
                self.rx_param.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == PITCH_RPS_MCU:
                print('Pitch Reply single MCU:')
                self.rx_param.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == YAW_RPS_MCU:
                print('Yaw Reply single MCU:')
                self.rx_param.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == MCU_WL_ROLL_NORMAL:
                print('MCU Write long Roll Normal')
                self.rx_block.dev_addr = ROLL_MOTOR_ADDR
                self.decode_rqrp_block_normal(msg)
            elif msg.id == MCU_WL_ROLL_HEAD:
                print('MCU Write long Roll Head')
                self.rx_block.dev_addr = ROLL_MOTOR_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == MCU_WL_PITCH_NORMAL:
                print('MCU Write long Pitch Normal')
                self.rx_block.dev_addr = PITCH_MOTOR_ADDR
                self.decode_rqrp_block_normal(msg)
            elif msg.id == MCU_WL_PITCH_HEAD:
                print('MCU Write long Pitch Head')
                self.rx_block.dev_addr = PITCH_MOTOR_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == MCU_WL_YAW_NORMAL:
                print('MCU Write long Yaw Normal')
                self.rx_block.dev_addr = YAW_MOTOR_ADDR
                self.decode_rqrp_block_normal(msg)
            elif msg.id == MCU_WL_YAW_HEAD:
                print('MCU Write long Yaw Head')
                self.rx_block.dev_addr = YAW_MOTOR_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == ROLL_RPL_MCU_NORMAL:
                print('Roll Reply long MCU Normal ')
                self.rx_block.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_block_normal(msg)
            elif msg.id == ROLL_RPL_MCU_HEAD:
                print('Roll Reply long MCU Head')
                self.rx_block.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == PITCH_RPL_MCU_NORMAL:
                print('Pitch Reply long MCU Normal ')
                self.rx_block.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_block_normal(msg)
            elif msg.id == PITCH_RPL_MCU_HEAD:
                print('Pitch Reply long MCU Head')
                self.rx_block.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == YAW_RPL_MCU_NORMAL:
                print('Yaw Reply long MCU Normal ')
                self.rx_block.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_block_normal(msg)
            elif msg.id == YAW_RPL_MCU_HEAD:
                print('Yaw Reply long MCU Head')
                self.rx_block.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == MCU_RS_ROLL:
                print('MCU Read single Roll')
                self.rx_param.dev_addr = ROLL_MOTOR_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == MCU_RS_PITCH:
                print('MCU Read single Pitch')
                self.rx_param.dev_addr = PITCH_MOTOR_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == MCU_RS_YAW:
                print('MCU Read single Yaw')
                self.rx_param.dev_addr = YAW_MOTOR_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == ROLL_RS_MCU:
                print('Roll Read single MCU')
                self.rx_param.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == PITCH_RS_MCU:
                print('Pitch Read single MCU')
                self.rx_param.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == YAW_RS_MCU:
                print('Yaw Read single MCU')
                self.rx_param.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_param(msg)
            elif msg.id == MCU_RL_ROLL:
                print('MCU Read long Roll')
                self.rx_block.dev_addr = ROLL_MOTOR_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == MCU_RL_PITCH:
                print('MCU Read long Pitch')
                self.rx_block.dev_addr = PITCH_MOTOR_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == MCU_RL_YAW:
                print('MCU Read long Yaw')
                self.rx_block.dev_addr = YAW_MOTOR_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == ROLL_RL_MCU:
                print('Roll Read long MCU')
                self.rx_block.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == PITCH_RL_MCU:
                print('Pitch Read long MCU')
                self.rx_block.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_block_head(msg)
            elif msg.id == YAW_RL_MCU:
                print('Yaw Read long MCU')
                self.rx_block.dev_addr = CONTROLLER_ADDR
                self.decode_rqrp_block_head(msg)

    def high_level_read(self):
        if self.can_read() > 0:
            self.can_decode(self.rx_msg)
'''
gimbal_can = CAN4Gimbal()
gimbal_can.can_open()
for i in range(1000):
    print('count', i)
    gimbal_can.high_level_read()
    time.sleep(0.01)

gimbal_can.close()

'''





