# V1 add GUI
# V2 add CAN BUS PRO
# V3 reconstruction
# V4 mavlink add something
import threading
import time
import can_pro_gimbal as can_pro
import ch_mavlink_v1 as mav
import pyqtgraph as pg
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.dockarea import *
import pyqtgraph.opengl as gl
from PyQt5.QtGui import *

from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *


class HumanInterface:
    def __init__(self):
        self.width = 1000
        self.high = 1000
        self.console_high = 150
        self.plot_high = self.high - self.console_high
        self.console_size = (self.width, self.console_high)
        self.plot_size = (self.width, self.plot_high)
        self.app = QtGui.QApplication([])
        self.win = QtGui.QMainWindow()
        self.area = DockArea()
        self.win.setCentralWidget(self.area)
        self.win.resize(self.width, self.high)
        self.win.setWindowTitle('Gimbal Monitor')
        self.timer_dt = 50

        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', [0, 0, 0])
        pg.setConfigOption('foreground', [255, 255, 255, 100])

        # Dock
        self.d_console = self.dock4console('Console')
        self.d_controller_state = self.dock4plot('Controller State')
        self.d_motors_state = self.dock4plot('Motors State')
        self.d_controller_config = self.dock4plot('Controller Config')
        self.d_motors_config = self.dock4plot('Motor Config')
        self.d_imu_attitude = Dock('IMU Attitude', size=(1000, self.plot_high), closable=True)
        #self.d_imu_attitude.show()
        self.area.addDock(self.d_console, 'bottom')
        self.area.addDock(self.d_controller_state, 'top')
        self.area.addDock(self.d_motors_state, 'top')
        self.area.addDock(self.d_controller_config, 'top')
        self.area.addDock(self.d_motors_config, 'top')
        self.area.addDock(self.d_imu_attitude, 'top')

        self.area.moveDock(self.d_motors_state, 'above', self.d_controller_state)
        self.area.moveDock(self.d_controller_config, 'above', self.d_motors_state)
        self.area.moveDock(self.d_motors_config, 'above', self.d_controller_config)
        self.area.moveDock(self.d_imu_attitude, 'right', self.d_controller_state)

        # Controller State Layout
        self.pen_width = 2
        self.fill_beta = 70
        self.pen_red = pg.mkPen('F00', width=self.pen_width)
        self.pen_green = pg.mkPen('0F0', width=self.pen_width)
        self.pen_blue = pg.mkPen('0AF', width=self.pen_width)
        self.fill_red = [255, 0, 0, self.fill_beta]
        self.fill_green = [0, 255, 0, self.fill_beta]
        self.fill_blue = [0, 200, 255, self.fill_beta]

        self.imu_angle_time_len = 200
        self.imu_angle_data = np.zeros((3, self.imu_angle_time_len))
        self.imu_angle_v_data = np.zeros((3, self.imu_angle_time_len))
        self.w_controller_state = pg.LayoutWidget()
        self.p_attitude = pg.PlotWidget(title="Camera Attitude")
        self.p_angle_v = pg.PlotWidget(title="Angle Velocity")
        self.p_attitude.showGrid(x=True, y=True)
        self.p_angle_v.showGrid(x=True, y=True)
        self.curve_roll = self.p_attitude.plot(self.imu_angle_data[0],
                                               pen=self.pen_red, fillLevel=0, brush=self.fill_red)
        self.curve_pitch = self.p_attitude.plot(self.imu_angle_data[1],
                                                pen=self.pen_green, fillLevel=0, brush=self.fill_green)
        self.curve_yaw = self.p_attitude.plot(self.imu_angle_data[2],
                                              pen=self.pen_blue, fillLevel=0, brush=self.fill_blue)
        self.curve_roll_v = self.p_angle_v.plot(self.imu_angle_v_data[0],
                                                pen=self.pen_red, fillLevel=0, brush=self.fill_red)
        self.curve_pitch_v = self.p_angle_v.plot(self.imu_angle_v_data[1],
                                                 pen=self.pen_green, fillLevel=0, brush=self.fill_green)
        self.curve_yaw_v = self.p_angle_v.plot(self.imu_angle_v_data[2],
                                               pen=self.pen_blue, fillLevel=0, brush=self.fill_blue)
        self.time_remain = 6
        self.angle_max = 100
        self.angle_v_max = 1000
        self.text_interval = self.angle_max*0.12
        self.text_interval_v = self.angle_v_max*0.12
        self.angle_text_max_y = self.angle_max*1.12
        self.angle_v_text_max_y = self.angle_v_max*1.12
        self.p_attitude.setXRange(0-self.time_remain, self.imu_angle_time_len+self.time_remain)
        self.p_attitude.setYRange(-100, 100)
        self.p_angle_v.setXRange(0-self.time_remain, self.imu_angle_time_len+self.time_remain)
        self.p_angle_v.setYRange(-1000, 1000)

        self.t_roll_angle = pg.TextItem()
        self.t_roll_angle.setText('roll: %0.1f' % (0), [255, 0, 0, 200])
        self.t_roll_angle.setPos(0, self.angle_text_max_y)
        self.p_attitude.addItem(self.t_roll_angle)

        self.t_pitch_angle = pg.TextItem()
        self.t_pitch_angle.setText('pitch: %0.1f' % (0), [0, 255, 0, 200])
        self.t_pitch_angle.setPos(0, self.angle_text_max_y-self.text_interval)
        self.p_attitude.addItem(self.t_pitch_angle)

        self.t_yaw_angle = pg.TextItem()
        self.t_yaw_angle.setText('yaw: %0.1f' % (0), [0, 200, 255, 230])
        self.t_yaw_angle.setPos(0, self.angle_text_max_y-2*self.text_interval)
        self.p_attitude.addItem(self.t_yaw_angle)

        self.t_roll_angle_v = pg.TextItem()
        self.t_roll_angle_v.setText('roll: %0.1f' % (0), [255, 0, 0, 200])
        self.t_roll_angle_v.setPos(0, self.angle_v_text_max_y)
        self.p_angle_v.addItem(self.t_roll_angle_v)

        self.t_pitch_angle_v = pg.TextItem()
        self.t_pitch_angle_v.setText('pitch: %0.1f' % (0), [0, 255, 0, 200])
        self.t_pitch_angle_v.setPos(0, self.angle_v_text_max_y-self.text_interval_v)
        self.p_angle_v.addItem(self.t_pitch_angle_v)

        self.t_yaw_angle_v = pg.TextItem()
        self.t_yaw_angle_v.setText('yaw: %0.1f' % (0), [0, 200, 255, 230])
        self.t_yaw_angle_v.setPos(0, self.angle_v_text_max_y-2*self.text_interval_v)
        self.p_angle_v.addItem(self.t_yaw_angle_v)

        self.w_controller_state.addWidget(self.p_attitude, row=0, col=0)
        self.w_controller_state.addWidget(self.p_angle_v, row=1, col=0)
        self.d_controller_state.addWidget(self.w_controller_state, row=0, col=1)

        self.timer_controller_state = QtCore.QTimer()
        self.timer_controller_state.timeout.connect(self.controller_state_update)

        # Motors State Layout
        self.motors_state_time_len = 200
        self.ntc_tempre_data = np.zeros((3, self.motors_state_time_len))
        self.input_current_data = np.zeros((3, self.motors_state_time_len))
        self.motor_current_data = np.zeros((3, self.motors_state_time_len))
        self.input_v_data = np.zeros((3, self.motors_state_time_len))
        self.duty_cycle_now_data = np.zeros((3, self.motors_state_time_len))
        self.rpm_data = np.zeros((3, self.motors_state_time_len))
        self.tacho_data = np.zeros((3, self.motors_state_time_len))
        self.tacho_abs_data = np.zeros((3, self.motors_state_time_len))

        self.w_motors_state = pg.LayoutWidget()
        self.p_ntc_tempre = pg.PlotWidget()
        self.p_current = pg.PlotWidget()
        self.p_input_v = pg.PlotWidget()
        self.p_duty_cycle_now = pg.PlotWidget()
        self.p_rpm = pg.PlotWidget()
        self.p_tacho = pg.PlotWidget()
        self.p_tach_abs = pg.PlotWidget()
        self.p_ntc_tempre.showGrid(True)
        self.p_current.showGrid(True)
        self.p_input_v.showGrid(True)
        self.p_duty_cycle_now.showGrid(True)
        self.p_rpm.showGrid(True)
        self.p_tacho.showGrid(True)
        self.p_tach_abs.showGrid(True)
        self.curve_ntc_tempre = self.p_ntc_tempre.plot(self.ntc_tempre_data[0],
                                                       pen=self.pen_blue, fillLevel=0, brush=self.fill_blue)
        self.curve_input_current = self.p_current.plot(self.input_current_data[0],
                                                       pen=self.pen_)




        # IMU Attitude OpenGL Layout

        self.vr_layout = pg.LayoutWidget()

        self.w_vr = gl.GLViewWidget()
        glEnable(GL_LINE_SMOOTH)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        glCullFace(GL_BACK)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC1_ALPHA)
        self.light()
        glMatrixMode(GL_PROJECTION)
        glMatrixMode(GL_MODELVIEW)
        gluPerspective(40., 1., 1., 40.)
        self.w_vr.opts['distance'] = 40
        self.w_vr.opts['azimuth'] = -10
        print(self.w_vr.opts)
        self.w_vr.setWindowTitle('IMU Attitude')
        #self.w_vr.setCameraPosition(pos=[0, 0, 50],)


        self.w_vr.show()
        self.vr_ground = gl.GLGridItem()
        self.vr_ground.scale(10, 10, 1)
        self.w_vr.addItem(self.vr_ground)

        self.cube_width = 20
        self.cube_high = self.cube_width*(1-0.618)
        self.verts = np.array([
            [self.cube_width/2, self.cube_width/2, self.cube_high/2],
            [-self.cube_width/2, self.cube_width/2, self.cube_high/2],
            [-self.cube_width/2, -self.cube_width/2, self.cube_high/2],
            [self.cube_width/2, -self.cube_width/2, self.cube_high/2],
            [self.cube_width/2, self.cube_width/2, -self.cube_high/2],
            [-self.cube_width/2, self.cube_width/2, -self.cube_high/2],
            [-self.cube_width/2, -self.cube_width/2, -self.cube_high/2],
            [self.cube_width/2, -self.cube_width/2, -self.cube_high/2],
        ])
        self.face = np.array([
            [0, 1, 2],  # up
            [0, 2, 3],
            [4, 6, 5],  # down
            [4, 7, 6],
            [0, 3, 7],  # front
            [0, 7, 4],
            [1, 6, 2],  # back
            [1, 5, 6],
            [0, 4, 5],  # left
            [0, 5, 1],
            [2, 6, 7],  # right
            [2, 7, 3],
        ])
        self.alpha = 1
        self.c_red = [1, 0, 0, self.alpha]
        self.c_greed = [0, 1, 0, self.alpha]
        self.c_blue = [0, 0, 1, self.alpha]
        self.gray_scale = 0.8
        self.c_gray = [self.gray_scale, self.gray_scale, self.gray_scale, self.alpha]
        self.colors = np.array([
            self.c_gray,
            self.c_gray,
            self.c_gray,
            self.c_gray,
            self.c_gray,
            self.c_gray,
            self.c_gray,
            self.c_gray,
            self.c_gray,
            self.c_gray,
            self.c_gray,
            self.c_gray,
        ])
        self.cube = gl.GLMeshItem(vertexes=self.verts, faces=self.face, faceColors=self.colors,
                                  shader='shaded', drawEdges=True, edgeColor=(1, 1, 1, 1), smooth=False)
        self.cube.translate(0, 0, 3)
        self.cube_quat = QQuaternion(1.0, 0.0, 0.0, 0.0)
        self.cube_mat = self.cube_quat.toRotationMatrix()
        self.cube_mat_data = self.cube_mat.data()
        self.cube_transform = pg.Transform3D(self.cube_mat_data[0],
                                             self.cube_mat_data[1],
                                             self.cube_mat_data[2],
                                             0,
                                             self.cube_mat_data[3],
                                             self.cube_mat_data[4],
                                             self.cube_mat_data[5],
                                             0,
                                             self.cube_mat_data[6],
                                             self.cube_mat_data[7],
                                             self.cube_mat_data[8],
                                             0,
                                             0, 0, 0, 1)

        self.cube.setTransform(self.cube_transform)
        self.w_vr.addItem(self.cube)
        self.d_imu_attitude.addWidget(self.w_vr)
        #self.w_vr.show()

    def light(self):
        lightZeroPosition = [00., 4., 10., 1.]
        lightZeroColor = [5, 2, 2, 1]  # green tinged
        glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
        glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1)
        glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05)
        glEnable(GL_LIGHT0)
        '''
        glLightfv(GL_LIGHT0, GL_AMBIENT, GLfloat_4(1.0, 1.0, 1.0, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, GLfloat_4(1.0, 1.0, 1.0, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, GLfloat_4(1.0, 1.0, 1.0, 1.0))
        glLightfv(GL_LIGHT0, GL_POSITION, GLfloat_4(5.0, 5.0, 5.0, 0.0))
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, GLfloat_4(0.2, 0.2, 0.2, 1.0))
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        '''

    def controller_state_update(self):
        self.curve_roll.setData(self.imu_angle_data[0])
        self.curve_pitch.setData(self.imu_angle_data[1])
        self.curve_yaw.setData(self.imu_angle_data[2])
        self.curve_roll_v.setData(self.imu_angle_v_data[0])
        self.curve_pitch_v.setData(self.imu_angle_v_data[1])
        self.curve_yaw_v.setData(self.imu_angle_v_data[2])
        self.t_roll_angle.setText('roll: %0.1f' % (self.imu_angle_data[0][-1]))
        self.t_pitch_angle.setText('pitch: %0.1f' % (self.imu_angle_data[1][-1]))
        self.t_yaw_angle.setText('yaw: %0.1f' % (self.imu_angle_data[2][-1]))
        self.t_roll_angle_v.setText('roll: %0.1f' % (self.imu_angle_v_data[0][-1]))
        self.t_pitch_angle_v.setText('pitch: %0.1f' % (self.imu_angle_v_data[1][-1]))
        self.t_yaw_angle_v.setText('yaw: %0.1f' % (self.imu_angle_v_data[2][-1]))

        self.cube_mat = self.cube_quat.toRotationMatrix()
        self.cube_mat_data = self.cube_mat.data()
        self.cube_transform = pg.Transform3D(self.cube_mat_data[0],
                                             self.cube_mat_data[1],
                                             self.cube_mat_data[2],
                                             0,
                                             self.cube_mat_data[3],
                                             self.cube_mat_data[4],
                                             self.cube_mat_data[5],
                                             0,
                                             self.cube_mat_data[6],
                                             self.cube_mat_data[7],
                                             self.cube_mat_data[8],
                                             0,
                                             0, 0, 0, 1)

        self.cube.setTransform(self.cube_transform)

    def dock4plot(self, name='plot dock'):
        dock = Dock(name, size=self.plot_size, )
        return dock

    def dock4console(self, name='console dock'):
        dock = Dock(name, size=self.console_size)
        return dock

    def run(self):
        self.win.show()
        self.timer_controller_state.start(10)
        self.app.exec_()


class Parameters:
    def __init__(self):
        self.controller_state_shape = (10, 200)
        self.motors_state_shape = (3, 8)
        self.controller_config_shape = (1, 7)
        self.motors_config_shape = (3, 20)

        self.controller_state_data = np.zeros(self.controller_state_shape)
        self.motors_state_data = np.zeros(self.motors_state_shape)
        self.controller_config_data = np.zeros(self.controller_config_shape)
        self.motors_config_data = np.zeros(self.motors_config_shape)

        self.quat2rpy_flag = False
        self.rpy2quat_flag = False


class Transceiver:
    def __init__(self):
        self.mavlink = mav.Mavlink()
        self.controller_mav = mav.Controller()
        self.gimbal_mav = mav.Gimbal()
        self.can = can_pro.CAN4Gimbal(3, 0, 0)
        #self.can.can_open()


class Main:
    def __init__(self):
        self.human_interface = HumanInterface()
        self.parameters = Parameters()
        self.transceiver = Transceiver()

       # self.quat = QQuaternion(1.0, 0, 0, 0)
       # self.eular = self.quat.toEulerAngles()
        self.threads = []
        self.t1 = threading.Thread(target=self.data_operation)
        self.threads.append(self.t1)


    def data_operation(self):
        while True:
            self.transceiver.mavlink.serial2mavlink()
            data_type = self.transceiver.mavlink.decode_msg(self.transceiver.controller_mav)
            if data_type == mav.MAVLINK_MSG_ID_ATTITUDE:
                self.map_attitude()
            elif data_type == mav.MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                self.map_attitude_quaternion()

    def map_attitude(self):
        #print('map attitude')
        self.parameters.controller_state_data = np.roll(self.parameters.controller_state_data, -1)

        self.parameters.controller_state_data[0][-1] = self.transceiver.mavlink.mav_attitude_pck.roll
        self.parameters.controller_state_data[1][-1] = self.transceiver.mavlink.mav_attitude_pck.pitch
        self.parameters.controller_state_data[2][-1] = self.transceiver.mavlink.mav_attitude_pck.yaw
        self.parameters.controller_state_data[3][-1] = self.transceiver.mavlink.mav_attitude_pck.roll_speed
        self.parameters.controller_state_data[4][-1] = self.transceiver.mavlink.mav_attitude_pck.pitch_speed
        self.parameters.controller_state_data[5][-1] = self.transceiver.mavlink.mav_attitude_pck.yaw_speed

        self.human_interface.imu_angle_data = self.parameters.controller_state_data[:3]
        self.human_interface.imu_angle_v_data = self.parameters.controller_state_data[3:]

    def map_attitude_quaternion(self):
        #print('map attitude quaternion')
        self.parameters.controller_state_data = np.roll(self.parameters.controller_state_data, -1)

        self.parameters.controller_state_data[3][-1] = self.transceiver.mavlink.mav_attitude_quat_pck.roll_speed
        self.parameters.controller_state_data[4][-1] = self.transceiver.mavlink.mav_attitude_quat_pck.pitch_speed
        self.parameters.controller_state_data[5][-1] = self.transceiver.mavlink.mav_attitude_quat_pck.yaw_speed
        self.parameters.controller_state_data[6][-1] = self.transceiver.mavlink.mav_attitude_quat_pck.q1
        self.parameters.controller_state_data[7][-1] = self.transceiver.mavlink.mav_attitude_quat_pck.q2
        self.parameters.controller_state_data[8][-1] = self.transceiver.mavlink.mav_attitude_quat_pck.q3
        self.parameters.controller_state_data[9][-1] = self.transceiver.mavlink.mav_attitude_quat_pck.q4

        quat = QQuaternion(self.transceiver.mavlink.mav_attitude_quat_pck.q1,
                           -self.transceiver.mavlink.mav_attitude_quat_pck.q2,
                           -self.transceiver.mavlink.mav_attitude_quat_pck.q3,
                           -self.transceiver.mavlink.mav_attitude_quat_pck.q4,)

        euler = quat.toEulerAngles()
        self.parameters.controller_state_data[0][-1] = euler.x()
        self.parameters.controller_state_data[1][-1] = euler.y()
        self.parameters.controller_state_data[2][-1] = euler.z()

        self.human_interface.cube_quat = quat

        self.human_interface.imu_angle_data = self.parameters.controller_state_data[:3]
        self.human_interface.imu_angle_v_data = self.parameters.controller_state_data[3:]

    def run(self):
        for t in self.threads:
            t.setDaemon(True)
            t.start()
        self.human_interface.run()


system = Main()
system.run()





















