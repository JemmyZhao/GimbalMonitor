# V1 add GUI
# V2 add CAN BUS PRO
# V3 reconstruction
# V4 mavlink add something
import threading
import time
#import can_pro_gimbal as can_pro
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


class Communicate(QtCore.QObject):
    link = QtCore.pyqtSignal()


class ChDock(Dock):
    def __init__(self, name, area=None, size=(10, 10), widget=None, hideTitle=False, autoOrientation=True, closable=False):
        super(ChDock, self).__init__(name, area, size, widget, hideTitle, autoOrientation, closable)
        self.communicate = Communicate()

    def mousePressEvent(self, event):
            self.communicate.link.emit()


class HumanInterface:
    def __init__(self):
        self.width = 1500
        self.high = 900
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
        #self.d_controller_state = Dock('Controller State', size=(1, 1))
        self.d_imu_attitude = Dock('IMU Attitude', size=(1000, self.plot_high), closable=True)
        #self.d_imu_attitude.show()
        self.area.addDock(self.d_console, 'bottom')
        self.area.addDock(self.d_controller_state, 'top')
        self.area.addDock(self.d_motors_state, 'top')
        self.area.addDock(self.d_controller_config, 'top')
        self.area.addDock(self.d_motors_config, 'top')
        self.area.addDock(self.d_imu_attitude, 'top')


        self.area.moveDock(self.d_controller_config, 'above', self.d_motors_config)
        self.area.moveDock(self.d_motors_state, 'above', self.d_controller_config)
        self.area.moveDock(self.d_controller_state, 'above', self.d_motors_state)
        self.area.moveDock(self.d_imu_attitude, 'right', self.d_controller_state)

        # Controller State Layout
        self.pen_width = 1.9
        self.fill_beta = 70
        self.pen_red = pg.mkPen('F11', width=self.pen_width)
        self.pen_green = pg.mkPen('0F0', width=self.pen_width)
        self.pen_blue = pg.mkPen('0AF', width=self.pen_width)
        self.fill_red = [255, 0, 0, self.fill_beta]
        self.fill_green = [0, 255, 0, self.fill_beta]
        self.fill_blue = [0, 200, 255, self.fill_beta]

        self.imu_angle_time_len = 150
        self.imu_angle_data = np.zeros((3, self.imu_angle_time_len))
        self.imu_angle_v_data = np.zeros((3, self.imu_angle_time_len))
        self.w_controller_state = pg.LayoutWidget()
        self.p_attitude = pg.PlotWidget(title="Camera Attitude")
        self.p_angle_v = pg.PlotWidget(title="Angle Velocity")
        self.p_angle_cov = pg.PlotWidget(title='Angle Variance ')
        self.p_angle_v_cov = pg.PlotWidget(title='Angle Velocity Variance')
        self.p_attitude.showGrid(x=True, y=True)
        self.p_angle_v.showGrid(x=True, y=True)
        self.p_angle_cov.showGrid(x=False, y=True)
        self.p_angle_v_cov.showGrid(x=False, y=True)
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

        self.bar_width = 0.4
        self.bar_x_min = 0
        self.bar_x_max = 4
        self.bar_angle_cov_min = -0.005
        self.bar_angle_cov_max = 0.06
        self.bar_angle_v_cov_min = -0.02
        self.bar_angle_v_cov_max = 0.4

        self.roll_cov_data = [0]
        self.pitch_cov_data = [0]
        self.yaw_cov_data = [0]
        self.roll_v_cov_data = [0]
        self.pitch_v_cov_data = [0]
        self.yaw_v_cov_data = [0]

        self.pen_width_bar = 1.2
        self.fill_beta_bar = 98
        self.fill_red_bar = [255, 0, 0, self.fill_beta_bar]
        self.fill_green_bar = [0, 255, 0, self.fill_beta_bar]
        self.fill_blue_bar = [0, 200, 255, self.fill_beta_bar]
        self.pen_red_bar = pg.mkPen('F11', width=self.pen_width_bar)
        self.pen_green_bar = pg.mkPen('0F0', width=self.pen_width_bar)
        self.pen_blue_bar = pg.mkPen('0AF', width=self.pen_width_bar)
        self.bg_roll_cov = pg.BarGraphItem(x=[1], height=self.roll_cov_data, width=self.bar_width, title='roll', pen=self.pen_red_bar, brush=self.fill_red_bar)
        self.bg_pitch_cov = pg.BarGraphItem(x=[2], height=self.pitch_cov_data, width=self.bar_width, title='pitch', pen=self.pen_green_bar, brush=self.fill_green_bar)
        self.bg_yaw_cov = pg.BarGraphItem(x=[3], height=self.yaw_cov_data, width=self.bar_width, title='yaw', pen=self.pen_blue_bar, brush=self.fill_blue_bar)
        self.p_angle_cov.setXRange(self.bar_x_min, self.bar_x_max)
        self.p_angle_cov.setYRange(self.bar_angle_cov_min, self.bar_angle_cov_max)
        self.p_angle_cov.addItem(self.bg_roll_cov)
        self.p_angle_cov.addItem(self.bg_pitch_cov)
        self.p_angle_cov.addItem(self.bg_yaw_cov)

        self.bg_roll_v_cov = pg.BarGraphItem(x=[1], height=self.roll_v_cov_data, width=self.bar_width, title='roll', pen=self.pen_red_bar, brush=self.fill_red_bar)
        self.bg_pitch_v_cov = pg.BarGraphItem(x=[2], height=self.pitch_v_cov_data, width=self.bar_width, title='pitch', pen=self.pen_green_bar, brush=self.fill_green_bar)
        self.bg_yaw_v_cov = pg.BarGraphItem(x=[3], height=self.yaw_v_cov_data, width=self.bar_width, title='yaw', pen=self.pen_blue_bar, brush=self.fill_blue_bar)
        self.p_angle_v_cov.setXRange(self.bar_x_min, self.bar_x_max)
        self.p_angle_v_cov.setYRange(self.bar_angle_v_cov_min, self.bar_angle_v_cov_max)
        self.p_angle_v_cov.addItem(self.bg_roll_v_cov)
        self.p_angle_v_cov.addItem(self.bg_pitch_v_cov)
        self.p_angle_v_cov.addItem(self.bg_yaw_v_cov)


        self.time_remain = 6
        self.angle_max = 100
        self.angle_v_max = 1000
        self.text_interval = self.angle_max*0.12
        self.text_interval_v = self.angle_v_max*0.12
        self.angle_text_max_y = self.angle_max*1.12
        self.angle_v_text_max_y = self.angle_v_max*1.12
        self.p_attitude.setXRange(0-self.time_remain, self.imu_angle_time_len+self.time_remain)
        self.p_attitude.setYRange(-self.angle_max, self.angle_max)
        self.p_angle_v.setXRange(0-self.time_remain, self.imu_angle_time_len+self.time_remain)
        self.p_angle_v.setYRange(-self.angle_v_max, self.angle_v_max)

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

        self.t_cov_y_interval = -self.bar_angle_cov_min*0.5
        self.t_cov_x_interval = 0.18
        self.t_roll_cov = pg.TextItem()
        self.t_roll_cov.setText('roll', [255, 0, 0, 200])
        self.t_roll_cov.setPos(1-self.t_cov_x_interval, 0-self.t_cov_y_interval)
        self.p_angle_cov.addItem(self.t_roll_cov)

        self.t_pitch_cov = pg.TextItem()
        self.t_pitch_cov.setText('pitch', [0, 255, 0, 200])
        self.t_pitch_cov.setPos(2-self.t_cov_x_interval-0.03, 0-self.t_cov_y_interval)
        self.p_angle_cov.addItem(self.t_pitch_cov)

        self.t_yaw_cov = pg.TextItem()
        self.t_yaw_cov.setText('yaw', [0, 200, 255, 200])
        self.t_yaw_cov.setPos(3-self.t_cov_x_interval, -self.t_cov_y_interval)
        self.p_angle_cov.addItem(self.t_yaw_cov)

        self.t_cov_v_y_interval = -self.bar_angle_v_cov_min*0.5
        self.t_cov_v_x_interval = 0.18
        self.t_roll_cov_vel = pg.TextItem()
        self.t_roll_cov_vel.setText('roll', [255, 0, 0, 200])
        self.t_roll_cov_vel.setPos(1-self.t_cov_v_x_interval, 0-self.t_cov_v_y_interval)
        self.p_angle_v_cov.addItem(self.t_roll_cov_vel)

        self.t_pitch_cov_vel = pg.TextItem()
        self.t_pitch_cov_vel.setText('pitch', [0, 255, 0, 200])
        self.t_pitch_cov_vel.setPos(2-self.t_cov_v_x_interval-0.03, 0-self.t_cov_v_y_interval)
        self.p_angle_v_cov.addItem(self.t_pitch_cov_vel)

        self.t_yaw_cov_vel = pg.TextItem()
        self.t_yaw_cov_vel.setText('yaw', [0, 200, 255, 200])
        self.t_yaw_cov_vel.setPos(3-self.t_cov_v_x_interval, 0-self.t_cov_v_y_interval)
        self.p_angle_v_cov.addItem(self.t_yaw_cov_vel)

        self.t_roll_cov_v = pg.TextItem()
        self.t_roll_cov_v.setText('%0.3f' % (self.roll_cov_data[0]), [255, 0, 0, 200])
        self.t_roll_cov_v.setPos(1-self.t_cov_x_interval, self.roll_cov_data[0]+2*self.t_cov_y_interval)
        self.p_angle_cov.addItem(self.t_roll_cov_v)

        self.t_pitch_cov_v = pg.TextItem()
        self.t_pitch_cov_v.setText('%0.3f' % (self.pitch_cov_data[0]), [0, 255, 0, 200])
        self.t_pitch_cov_v.setPos(2-self.t_cov_x_interval, self.pitch_cov_data[0]+2*self.t_cov_y_interval)
        self.p_angle_cov.addItem(self.t_pitch_cov_v)

        self.t_yaw_cov_v = pg.TextItem()
        self.t_yaw_cov_v.setText('%0.3f' % (self.yaw_cov_data[0]), [0, 200, 255, 200])
        self.t_yaw_cov_v.setPos(3-self.t_cov_x_interval, self.yaw_cov_data[0]+2*self.t_cov_y_interval)
        self.p_angle_cov.addItem(self.t_yaw_cov_v)

        self.t_roll_vel_cov_v = pg.TextItem()
        self.t_roll_vel_cov_v.setText('%0.3f' % (self.roll_v_cov_data[0]), [255, 0, 0, 200])
        self.t_roll_vel_cov_v.setPos(1-self.t_cov_x_interval, self.roll_v_cov_data[0]+4*self.t_cov_v_y_interval)
        self.p_angle_v_cov.addItem(self.t_roll_vel_cov_v)

        self.t_pitch_vel_cov_v = pg.TextItem()
        self.t_pitch_vel_cov_v.setText('%0.3f' % (self.pitch_v_cov_data[0]), [0, 255, 0, 200])
        self.t_pitch_vel_cov_v.setPos(2-self.t_cov_x_interval, self.pitch_v_cov_data[0]+4*self.t_cov_v_y_interval)
        self.p_angle_v_cov.addItem(self.t_pitch_vel_cov_v)

        self.t_yaw_vel_cov_v = pg.TextItem()
        self.t_yaw_vel_cov_v.setText('%0.3f' % (self.yaw_v_cov_data[0]), [0, 200, 255, 200])
        self.t_yaw_vel_cov_v.setPos(3-self.t_cov_x_interval, self.yaw_v_cov_data[0]+4*self.t_cov_v_y_interval)
        self.p_angle_v_cov.addItem(self.t_yaw_vel_cov_v)


        self.w_controller_state.addWidget(self.p_attitude, row=0, col=0)
        self.w_controller_state.addWidget(self.p_angle_v, row=1, col=0)
        self.w_controller_state.addWidget(self.p_angle_cov, row=0, col=1)
        self.w_controller_state.addWidget(self.p_angle_v_cov,row=1, col=1)
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
        self.p_ntc_tempre = pg.PlotWidget(title='NTC Tempre')
        self.p_current = pg.PlotWidget(title='Current')
        self.p_input_v = pg.PlotWidget(title='Input Voltage')
        self.p_duty_cycle_now = pg.PlotWidget(title='Duty Cycle')
        self.p_rpm = pg.PlotWidget(title='RPM')
        self.p_tacho = pg.PlotWidget(title='Tacho')
        self.p_tach_abs = pg.PlotWidget(title='Tacho ABS')
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
                                                       pen=self.pen_blue, fillLevel=0, brush=self.fill_blue)
        self.curve_motor_current = self.p_current.plot(self.motor_current_data[0],
                                                       pen=self.pen_green, fillLevel=0, brush=self.fill_green)
        self.curve_input_v = self.p_input_v.plot(self.input_v_data[0],
                                                 pen=self.pen_blue, fillLevel=0, brush=self.fill_blue)
        self.curve_duty_cycle_now = self.p_duty_cycle_now.plot(self.duty_cycle_now_data[0],
                                                               pen=self.pen_blue, fillLevel=0, brush=self.fill_blue)
        self.curve_rpm = self.p_rpm.plot(self.rpm_data[0],
                                         pen=self.pen_blue, fillLevel=0, brush=self.fill_blue)
        self.curve_tacho = self.p_tacho.plot(self.tacho_data[0],
                                             pen=self.pen_blue, fillLevel=0, brush=self.fill_blue)

        self.w_motors_state.addWidget(self.p_ntc_tempre, row=0, col=0)
        self.w_motors_state.addWidget(self.p_current, row=1, col=0)
        self.w_motors_state.addWidget(self.p_input_v, row=2, col=0)
        self.w_motors_state.addWidget(self.p_rpm, row=0, col=1)
        self.w_motors_state.addWidget(self.p_tacho, row=1, col=1)
        self.w_motors_state.addWidget(self.p_duty_cycle_now, row=2, col=1)

        self.d_motors_state.addWidget(self.w_motors_state, row=0, col=0)

        # Controller Config Layout
        self.w_controller_config = pg.LayoutWidget()

        self.spins_roll = self.controller_config_spins()
        self.spins_pitch = self.controller_config_spins()
        self.spins_yaw = self.controller_config_spins()
        # Important parameter
        self.roll_labels = []
        self.roll_spins = []
        self.pitch_labels = []
        self.pitch_spins = []
        self.yaw_labels = []
        self.yaw_spins = []

        self.roll_label = QtGui.QLabel('ROLL')
        self.pitch_label = QtGui.QLabel('PITCH')
        self.yaw_label = QtGui.QLabel('YAW')
        self.roll_label.setMaximumSize(100, 20)
        self.pitch_label.setMaximumSize(100, 20)
        self.yaw_label.setMaximumSize(100, 20)
        self.w_controller_config.addWidget(self.roll_label, row=0, col=0)
        self.w_controller_config.addWidget(self.pitch_label, row=0, col=1)
        self.w_controller_config.addWidget(self.yaw_label, row=0, col=2)
        for i in range(len(self.spins_roll)):
            label = QtGui.QLabel(self.spins_roll[i][0])
            label.setMaximumSize(100, 12)
            self.spins_roll[i][1].setMaximumSize(100, 20)
            self.w_controller_config.addWidget(label, row=2*i+1, col=0)
            self.w_controller_config.addWidget(self.spins_roll[i][1], row=2*i+2, col=0)
            self.roll_labels.append(label)
            self.roll_spins.append(self.spins_roll[i][1])
        for i in range(len(self.spins_pitch)):
            label = QtGui.QLabel(self.spins_pitch[i][0])
            label.setMaximumSize(100, 12)
            self.spins_pitch[i][1].setMaximumSize(100, 20)
            self.w_controller_config.addWidget(label, row=2*i+1, col=1)
            self.w_controller_config.addWidget(self.spins_pitch[i][1], row=2*i+2, col=1)
            self.pitch_labels.append(label)
            self.pitch_spins.append(self.spins_pitch[i][1])
        self.loop_cnt = 0
        for i in range(len(self.spins_yaw)):
            label = QtGui.QLabel(self.spins_yaw[i][0])
            label.setMaximumSize(100, 12)
            self.spins_yaw[i][1].setMaximumSize(100, 20)
            self.w_controller_config.addWidget(label, row=2*i+1, col=2)
            self.w_controller_config.addWidget(self.spins_yaw[i][1], row=2*i+2, col=2)
            self.yaw_labels.append(label)
            self.yaw_spins.append(self.spins_yaw[i][1])
            self.loop_cnt = self.loop_cnt+1

        self.btn_controller_config_save = QtGui.QPushButton('Save Parameter')
        self.btn_controller_config_read = QtGui.QPushButton('Read Parameter')
        self.w_controller_config.addWidget(self.btn_controller_config_read, row=2*self.loop_cnt+3, col=0)
        self.w_controller_config.addWidget(self.btn_controller_config_save, row=2*self.loop_cnt+3, col=1)
        self.w_controller_config.layout.setHorizontalSpacing(120)
        self.w_controller_config.layout.setVerticalSpacing(1)

        self.d_controller_config.addWidget(self.w_controller_config, row=0, col=0)

        # Motors Config Layout
        self.w_motor_config = pg.LayoutWidget()
        self.spins_roll_motor = self.motors_config_spins()
        self.spins_pitch_motor = self.motors_config_spins()
        self.spins_yaw_motor = self.motors_config_spins()
        self.w_motor_config.addWidget(self.roll_label, row=0, col=0)
        self.w_motor_config.addWidget(self.pitch_label, row=0, col=2)
        self.w_motor_config.addWidget(self.yaw_label, row=0, col=4)
        # Important Data
        self.roll_motor_labels = []
        self.pitch_motor_labels = []
        self.yaw_motor_labels = []
        self.roll_motor_spins = []
        self.pitch_motor_spins = []
        self.yaw_motor_spins = []
        for i in range(len(self.spins_roll_motor)):
            label = QtGui.QLabel(self.spins_roll_motor[i][0])
            label.setMaximumSize(150, 20)
            self.spins_roll_motor[i][1].setMaximumSize(100, 20)
            self.w_motor_config.addWidget(label, row=i+1, col=0)
            self.w_motor_config.addWidget(self.spins_roll_motor[i][1], row=i+1, col=1)
            self.roll_motor_labels.append(label)
            self.roll_motor_spins.append(self.spins_roll_motor[i][1])
        for i in range(len(self.spins_pitch_motor)):
            label = QtGui.QLabel(self.spins_pitch_motor[i][0])
            label.setMaximumSize(150, 20)
            self.spins_pitch_motor[i][1].setMaximumSize(100, 20)
            self.w_motor_config.addWidget(label, row=i+1, col=2)
            self.w_motor_config.addWidget(self.spins_pitch_motor[i][1], row=i+1, col=3)
            self.pitch_motor_labels.append(label)
            self.pitch_motor_spins.append(self.spins_pitch_motor[i][1])
        self.loop_cnt = 0
        for i in range(len(self.spins_yaw_motor)):
            label = QtGui.QLabel(self.spins_yaw_motor[i][0])
            label.setMaximumSize(150, 20)
            self.spins_yaw_motor[i][1].setMaximumSize(100, 20)
            self.w_motor_config.addWidget(label, row=i+1, col=4)
            self.w_motor_config.addWidget(self.spins_yaw_motor[i][1], row=i+1, col=5)
            self.yaw_motor_labels.append(label)
            self.yaw_motor_spins.append(self.spins_yaw_motor[i][1])
            self.loop_cnt = self.loop_cnt+1
        self.btn_motor_config_read = QtGui.QPushButton('Read Parameter')
        self.btn_motor_config_save = QtGui.QPushButton('Save Parameter')
        self.w_motor_config.addWidget(self.btn_motor_config_read, row=self.loop_cnt+2, col=0)
        self.w_motor_config.addWidget(self.btn_motor_config_save, row=self.loop_cnt+2, col=1)

        self.d_motors_config.addWidget(self.w_motor_config, row=0, col=0)


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

    def controller_config_spins(self):
        spins = [
            ('Rate P', pg.SpinBox(value=0, bounds=[0, None], )),
            ('Rate I', pg.SpinBox(value=0, bounds=[0, None])),
            ('Rate D', pg.SpinBox(value=0, bounds=[0, None])),
            ('Rate Max I', pg.SpinBox(value=0, bounds=[0, None])),
            ('Filter HZ', pg.SpinBox(value=0, bounds=[0, None])),
            ('Stable P', pg.SpinBox(value=0, bounds=[0, None])),
            ('Stable D', pg.SpinBox(value=0, bounds=[0, None])),
        ]
        return spins

    def motors_config_spins(self):
        spins = [
            ('FOC Current KP', pg.SpinBox(value=0, bounds=[0, None])),
            ('FOC Current KI', pg.SpinBox(value=0, bounds=[0, None])),
            ('FOC F SW', pg.SpinBox(value=0, bounds=[0, None])),
            ('FOC DT us', pg.SpinBox(value=0, bounds=[0, None])),

            ('FOC Encoder Inverted', pg.SpinBox(value=0, bounds=[0, None])),
            ('FOC Encoder Offset', pg.SpinBox(value=0, bounds=[0, None])),
            ('FOC Encoder Ratio', pg.SpinBox(value=0, bounds=[0, None])),

            ('FOC PLL KP', pg.SpinBox(value=0, bounds=[0, None])),
            ('FOC PLL KI', pg.SpinBox(value=0, bounds=[0, None])),

            ('FOC Duty Downramp KP', pg.SpinBox(value=0, bounds=[0, None])),
            ('FOC Duty Downramp KI', pg.SpinBox(value=0, bounds=[0, None])),

            ('S PID KP', pg.SpinBox(value=0, bounds=[0, None])),
            ('S PID KI', pg.SpinBox(value=0, bounds=[0, None])),
            ('S PID KD', pg.SpinBox(value=0, bounds=[0, None])),
            ('S PID Min ERPM', pg.SpinBox(value=0, bounds=[0, None])),
            ('P PID KP', pg.SpinBox(value=0, bounds=[0, None])),
            ('P PID KI', pg.SpinBox(value=0, bounds=[0, None])),
            ('P PID KD', pg.SpinBox(value=0, bounds=[0, None])),
            ('P PID Ang Div', pg.SpinBox(value=0, bounds=[0, None])),
            ('M Fault Stop Time us', pg.SpinBox(value=0, bounds=[0, None])),
        ]
        return spins

    def close_gl_window(self):
        self.d_imu_attitude.close()

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

        self.bg_roll_cov.setOpts(height=self.roll_cov_data)
        self.bg_pitch_cov.setOpts(height=self.pitch_cov_data)
        self.bg_yaw_cov.setOpts(height=self.yaw_cov_data)
        self.bg_roll_v_cov.setOpts(height=self.roll_v_cov_data)
        self.bg_pitch_v_cov.setOpts(height=self.pitch_v_cov_data)
        self.bg_yaw_v_cov.setOpts(height=self.yaw_v_cov_data)

        self.t_roll_cov_v.setText('%0.3f' % (self.roll_cov_data[0]), [255, 0, 0, 200])
        self.t_roll_cov_v.setPos(1-self.t_cov_x_interval, self.roll_cov_data[0]+2*self.t_cov_y_interval)
        self.t_pitch_cov_v.setText('%0.3f' % (self.pitch_cov_data[0]), [0, 255, 0, 200])
        self.t_pitch_cov_v.setPos(2-self.t_cov_x_interval, self.pitch_cov_data[0]+2*self.t_cov_y_interval)
        self.t_yaw_cov_v.setText('%0.3f' % (self.yaw_cov_data[0]), [0, 200, 255, 200])
        self.t_yaw_cov_v.setPos(3 - self.t_cov_x_interval, self.yaw_cov_data[0] + 2 * self.t_cov_y_interval)

        self.t_roll_vel_cov_v.setText('%0.3f' % (self.roll_v_cov_data[0]), [255, 0, 0, 200])
        self.t_roll_vel_cov_v.setPos(1-self.t_cov_x_interval, self.roll_v_cov_data[0]+4*self.t_cov_v_y_interval)

        self.t_pitch_vel_cov_v.setText('%0.3f' % (self.pitch_v_cov_data[0]), [0, 255, 0, 200])
        self.t_pitch_vel_cov_v.setPos(2-self.t_cov_x_interval, self.pitch_v_cov_data[0]+4*self.t_cov_v_y_interval)

        self.t_yaw_vel_cov_v.setText('%0.3f' % (self.yaw_v_cov_data[0]), [0, 200, 255, 200])
        self.t_yaw_vel_cov_v.setPos(3-self.t_cov_x_interval, self.yaw_v_cov_data[0]+4*self.t_cov_v_y_interval)

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
        dock = ChDock(name, size=self.plot_size, )
        return dock

    def dock4console(self, name='console dock'):
        dock = Dock(name, size=self.console_size)
        return dock

    def run(self):
        self.win.show()
        self.timer_controller_state.start(33)
        self.app.exec_()


class Parameters:
    def __init__(self):
        self.controller_state_shape = (10, 150)
        self.motors_state_shape = (3, 8)
        self.controller_config_shape = (3, 7)
        self.motors_config_shape = (3, 20)

        self.controller_state_data = np.zeros(self.controller_state_shape)
        self.motors_state_data = np.zeros(self.motors_state_shape)
        self.controller_config_data = np.zeros(self.controller_config_shape)
        self.motors_config_data = np.zeros(self.motors_config_shape)

        self.quat2rpy_flag = False
        self.rpy2quat_flag = False

        self.data_calculate_len = 30
        self.controller_state_cov = np.zeros(6)

    def cal_controller_state_variance(self):
        for i in range(6):
            self.controller_state_cov[i] = np.std(self.controller_state_data[i][-self.data_calculate_len:])
        #print(self.controller_state_cov)


class Transceiver:
    def __init__(self):
        self.mavlink = mav.Mavlink()
        self.controller_mav = mav.Controller()
        self.gimbal_mav = mav.Gimbal()
        #self.can = can_pro.CAN4Gimbal(3, 0, 0)
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

        self.interface_connect()

        self.axis_text = ['Roll', 'Pitch', 'Yaw']

    # DATA OPERATING
    def data_operation(self):
        while True:
            len = self.transceiver.mavlink.serial2mavlink()
            #print(len)
            if len > 0:
                data_type = self.transceiver.mavlink.decode_msg(self.transceiver.controller_mav)
                if data_type == mav.MAVLINK_MSG_ID_ATTITUDE:
                    self.map_attitude()
                elif data_type == mav.MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                    self.map_attitude_quaternion()
                elif data_type == mav.MAVLINK_MSG_ID_CONTROLLER_CONFIG_DATA:
                    #print('read controller')
                    self.map_controller_config()
                    self.set_controller_spins_value_from_param()
                    #print('read controller completed')
                elif data_type == mav.MAVLINK_MSG_ID_MOTORS_CONFIG_DATA:
                    #print('read motor')
                    self.map_motors_config()
                    self.set_motors_spins_value_from_param()
                    #print('read motors completed')

    def interface_connect(self):
        self.human_interface.btn_controller_config_read.clicked.connect(self.read_controller_config)
        self.human_interface.btn_controller_config_save.clicked.connect(self.write_controller_config)
        self.human_interface.btn_motor_config_read.clicked.connect(self.read_motors_config)
        self.human_interface.btn_motor_config_save.clicked.connect(self.write_motors_config)

        self.human_interface.d_controller_state.communicate.link.connect(self.read_attitude_quat)

        self.spins_contrller_param_connect()
        self.spins_motor_param_connect()

    def spins_contrller_param_connect(self):

        self.human_interface.roll_spins[0].sigValueChanged.connect(lambda:
                                                                   self.set_controller_param_from_spins(
                                                                        self.human_interface.roll_spins[0], 0, 0))
        self.human_interface.roll_spins[1].sigValueChanged.connect(lambda:
                                                                   self.set_controller_param_from_spins(
                                                                        self.human_interface.roll_spins[1], 0, 1))
        self.human_interface.roll_spins[2].sigValueChanged.connect(lambda:
                                                                   self.set_controller_param_from_spins(
                                                                        self.human_interface.roll_spins[2], 0, 2))
        self.human_interface.roll_spins[3].sigValueChanged.connect(lambda:
                                                                   self.set_controller_param_from_spins(
                                                                        self.human_interface.roll_spins[3], 0, 3))
        self.human_interface.roll_spins[4].sigValueChanged.connect(lambda:
                                                                   self.set_controller_param_from_spins(
                                                                        self.human_interface.roll_spins[4], 0, 4))
        self.human_interface.roll_spins[5].sigValueChanged.connect(lambda:
                                                                   self.set_controller_param_from_spins(
                                                                        self.human_interface.roll_spins[5], 0, 5))
        self.human_interface.roll_spins[6].sigValueChanged.connect(lambda:
                                                                   self.set_controller_param_from_spins(
                                                                        self.human_interface.roll_spins[6], 0, 6))

        self.human_interface.pitch_spins[0].sigValueChanged.connect(lambda:
                                                                    self.set_controller_param_from_spins(
                                                                        self.human_interface.pitch_spins[0], 1, 0))
        self.human_interface.pitch_spins[1].sigValueChanged.connect(lambda:
                                                                    self.set_controller_param_from_spins(
                                                                        self.human_interface.pitch_spins[1], 1, 1))
        self.human_interface.pitch_spins[2].sigValueChanged.connect(lambda:
                                                                    self.set_controller_param_from_spins(
                                                                        self.human_interface.pitch_spins[2], 1, 2))
        self.human_interface.pitch_spins[3].sigValueChanged.connect(lambda:
                                                                    self.set_controller_param_from_spins(
                                                                        self.human_interface.pitch_spins[3], 1, 3))
        self.human_interface.pitch_spins[4].sigValueChanged.connect(lambda:
                                                                    self.set_controller_param_from_spins(
                                                                        self.human_interface.pitch_spins[4], 1, 4))
        self.human_interface.pitch_spins[5].sigValueChanged.connect(lambda:
                                                                    self.set_controller_param_from_spins(
                                                                        self.human_interface.pitch_spins[5], 1, 5))
        self.human_interface.pitch_spins[6].sigValueChanged.connect(lambda:
                                                                    self.set_controller_param_from_spins(
                                                                        self.human_interface.pitch_spins[6], 1, 6))


        self.human_interface.yaw_spins[0].sigValueChanged.connect(lambda:
                                                                  self.set_controller_param_from_spins(
                                                                        self.human_interface.yaw_spins[0], 2, 0))
        self.human_interface.yaw_spins[1].sigValueChanged.connect(lambda:
                                                                  self.set_controller_param_from_spins(
                                                                    self.human_interface.yaw_spins[1], 2, 1))
        self.human_interface.yaw_spins[2].sigValueChanged.connect(lambda:
                                                                  self.set_controller_param_from_spins(
                                                                    self.human_interface.yaw_spins[2], 2, 2))
        self.human_interface.yaw_spins[3].sigValueChanged.connect(lambda:
                                                                  self.set_controller_param_from_spins(
                                                                        self.human_interface.yaw_spins[3], 2, 3))
        self.human_interface.yaw_spins[4].sigValueChanged.connect(lambda:
                                                                  self.set_controller_param_from_spins(
                                                                        self.human_interface.yaw_spins[4], 2, 4))
        self.human_interface.yaw_spins[5].sigValueChanged.connect(lambda:
                                                                  self.set_controller_param_from_spins(
                                                                        self.human_interface.yaw_spins[5], 2, 5))
        self.human_interface.yaw_spins[6].sigValueChanged.connect(lambda:
                                                                  self.set_controller_param_from_spins(
                                                                        self.human_interface.yaw_spins[6], 2, 6))

    def spins_motor_param_connect(self):
        self.human_interface.roll_motor_spins[0].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[0],0,0))
        self.human_interface.roll_motor_spins[1].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[1],0,1))
        self.human_interface.roll_motor_spins[2].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[2],0,2))
        self.human_interface.roll_motor_spins[3].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[3],0,3))
        self.human_interface.roll_motor_spins[4].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[4],0,4))
        self.human_interface.roll_motor_spins[5].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[5],0,5))
        self.human_interface.roll_motor_spins[6].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[6],0,6))
        self.human_interface.roll_motor_spins[7].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[7],0,7))
        self.human_interface.roll_motor_spins[8].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[8],0,8))
        self.human_interface.roll_motor_spins[9].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[9],0,9))
        self.human_interface.roll_motor_spins[10].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[10],0,10))
        self.human_interface.roll_motor_spins[11].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[11],0,11))
        self.human_interface.roll_motor_spins[12].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[12],0,12))
        self.human_interface.roll_motor_spins[13].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[13],0,13))
        self.human_interface.roll_motor_spins[14].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[14],0,14))
        self.human_interface.roll_motor_spins[15].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[15],0,15))
        self.human_interface.roll_motor_spins[16].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[16],0,16))
        self.human_interface.roll_motor_spins[17].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[17],0,17))
        self.human_interface.roll_motor_spins[18].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[18],0,18))
        self.human_interface.roll_motor_spins[19].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.roll_motor_spins[19],0,19))

        self.human_interface.pitch_motor_spins[0].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[0],1,0))
        self.human_interface.pitch_motor_spins[1].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[1],1,1))
        self.human_interface.pitch_motor_spins[2].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[2],1,2))
        self.human_interface.pitch_motor_spins[3].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[3],1,3))
        self.human_interface.pitch_motor_spins[4].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[4],1,4))
        self.human_interface.pitch_motor_spins[5].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[5],1,5))
        self.human_interface.pitch_motor_spins[6].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[6],1,6))
        self.human_interface.pitch_motor_spins[7].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[7],1,7))
        self.human_interface.pitch_motor_spins[8].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[8],1,8))
        self.human_interface.pitch_motor_spins[9].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[9],1,9))
        self.human_interface.pitch_motor_spins[10].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[10],1,10))
        self.human_interface.pitch_motor_spins[11].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[11],1,11))
        self.human_interface.pitch_motor_spins[12].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[12],1,12))
        self.human_interface.pitch_motor_spins[13].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[13],1,13))
        self.human_interface.pitch_motor_spins[14].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[14],1,14))
        self.human_interface.pitch_motor_spins[15].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[15],1,15))
        self.human_interface.pitch_motor_spins[16].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[16],1,16))
        self.human_interface.pitch_motor_spins[17].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[17],1,17))
        self.human_interface.pitch_motor_spins[18].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[18],1,18))
        self.human_interface.pitch_motor_spins[19].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.pitch_motor_spins[19],1,19))

        self.human_interface.yaw_motor_spins[0].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[0],2,0))
        self.human_interface.yaw_motor_spins[1].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[1],2,1))
        self.human_interface.yaw_motor_spins[2].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[2],2,2))
        self.human_interface.yaw_motor_spins[3].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[3],2,3))
        self.human_interface.yaw_motor_spins[4].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[4],2,4))
        self.human_interface.yaw_motor_spins[5].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[5],2,5))
        self.human_interface.yaw_motor_spins[6].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[6],2,6))
        self.human_interface.yaw_motor_spins[7].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[7],2,7))
        self.human_interface.yaw_motor_spins[8].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[8],2,8))
        self.human_interface.yaw_motor_spins[9].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[9],2,9))
        self.human_interface.yaw_motor_spins[10].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[10],2,10))
        self.human_interface.yaw_motor_spins[11].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[11],2,11))
        self.human_interface.yaw_motor_spins[12].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[12],2,12))
        self.human_interface.yaw_motor_spins[13].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[13],2,13))
        self.human_interface.yaw_motor_spins[14].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[14],2,14))
        self.human_interface.yaw_motor_spins[15].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[15],2,15))
        self.human_interface.yaw_motor_spins[16].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[16],2,16))
        self.human_interface.yaw_motor_spins[17].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[17],2,17))
        self.human_interface.yaw_motor_spins[18].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[18],2,18))
        self.human_interface.yaw_motor_spins[19].sigValueChanged.connect(lambda:
                                                                         self.set_motor_param_from_spins(
                                                                             self.human_interface.yaw_motor_spins[19],2,19))

    def set_controller_param_from_spins(self, sb, ax, param_id):
        print('set controller param from spins')
        param_value = self.parameters.controller_config_data[ax][param_id] = sb.value()
        self.transceiver.mavlink.mavlink_write_param(mav.MAV_COMP_ID_CONTROLLER, param_id, param_value)
        print(self.axis_text[ax], ': ', self.human_interface.roll_labels[param_id].text(), ' ', param_value)

    def set_motor_param_from_spins(self, sb, ax, param_id):
        param_value = self.parameters.motors_config_data[ax][param_id] = sb.value()
        self.transceiver.mavlink.mavlink_write_param(mav.MAV_COMP_ID_ROLL_MOTOR+ax, param_id, param_value)
        print(self.axis_text[ax], 'motor', ': ', self.human_interface.roll_motor_labels[param_id].text(), ' ', param_value)

    def set_controller_spins_value_from_param(self):
        for i in range(len(self.human_interface.roll_spins)):
            self.human_interface.roll_spins[i].setValue(self.parameters.controller_config_data[0][i])
        for i in range(len(self.human_interface.roll_spins)):
            self.human_interface.pitch_spins[i].setValue(self.parameters.controller_config_data[1][i])
        for i in range(len(self.human_interface.roll_spins)):
            self.human_interface.yaw_spins[i].setValue(self.parameters.controller_config_data[2][i])

    def set_motors_spins_value_from_param(self):
        for i in range(len(self.human_interface.roll_motor_spins)):
            self.human_interface.roll_motor_spins[i].setValue(self.parameters.motors_config_data[0][i])
        for i in range(len(self.human_interface.roll_motor_spins)):
            self.human_interface.pitch_motor_spins[i].setValue(self.parameters.motors_config_data[1][i])
        for i in range(len(self.human_interface.roll_motor_spins)):
            self.human_interface.yaw_motor_spins[i].setValue(self.parameters.motors_config_data[2][i])

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

        self.parameters.cal_controller_state_variance()

        self.human_interface.cube_quat = quat

        self.human_interface.imu_angle_data = self.parameters.controller_state_data[:3]
        self.human_interface.imu_angle_v_data = self.parameters.controller_state_data[3:]

        self.human_interface.roll_cov_data = [self.parameters.controller_state_cov[0]]
        self.human_interface.pitch_cov_data = [self.parameters.controller_state_cov[1]]
        self.human_interface.yaw_cov_data = [self.parameters.controller_state_cov[2]]
        self.human_interface.roll_v_cov_data = [self.parameters.controller_state_cov[3]]
        self.human_interface.pitch_v_cov_data = [self.parameters.controller_state_cov[4]]
        self.human_interface.yaw_v_cov_data = [self.parameters.controller_state_cov[5]]

    def map_controller_config(self):
        print(self.transceiver.mavlink.mav_controller_config_pck.params)
        self.parameters.controller_config_data[0] = self.transceiver.mavlink.mav_controller_config_pck.params[:7]
        #print(self.parameters.controller_config_data[0])
        self.parameters.controller_config_data[1] = self.transceiver.mavlink.mav_controller_config_pck.params[7:14]
        self.parameters.controller_config_data[2] = self.transceiver.mavlink.mav_controller_config_pck.params[14:]

    def map_motors_config(self):
        self.parameters.motors_config_data[0] = self.transceiver.mavlink.mav_motors_config_pck.params[:20]
        #print(self.parameters.motors_config_data[0])
        self.parameters.motors_config_data[1] = self.transceiver.mavlink.mav_motors_config_pck.params[20:40]
        self.parameters.motors_config_data[2] = self.transceiver.mavlink.mav_motors_config_pck.params[40:]

    def map_read_param(self):
        dev_id = self.transceiver.mavlink.mav_read_param_pck.dev_id
        if dev_id == mav.MAV_COMP_ID_CONTROLLER:
            ax = int(self.transceiver.mavlink.mav_read_param_pck.param_id / 7)
            id = int(self.transceiver.mavlink.mav_read_param_pck.param_id % 7)
            self.parameters.controller_config_data[ax][id] = self.transceiver.mavlink.mav_read_param_pck.param_value
        elif dev_id == mav.MAV_COMP_ID_ROLL_MOTOR:
            ax = int(self.transceiver.mavlink.mav_read_param_pck.param_id / 20)
            id = int(self.transceiver.mavlink.mav_read_param_pck.param_id % 20)
            self.parameters.motors_config_data[ax][id] = self.transceiver.mavlink.mav_read_param_pck.param_value

    def read_controller_config(self):
        print('read controller config')
        self.transceiver.mavlink.mavlink_read_controller_config()

    def read_motors_config(self):
        self.transceiver.mavlink.mavlink_read_motors_config()

    def write_controller_config(self):
        self.transceiver.mavlink.mavlink_write_controller_config()

    def write_motors_config(self):
        self.transceiver.mavlink.mavlink_write_motors_config()

    def read_attitude_quat(self):
        self.transceiver.mavlink.mavlink_read_attitude_quat()

    def run(self):
        for t in self.threads:
            t.setDaemon(True)
            t.start()
        self.human_interface.run()


system = Main()
system.run()





















