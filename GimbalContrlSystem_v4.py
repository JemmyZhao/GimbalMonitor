# V1 add GUI
# V2 add CAN BUS PRO
# V3 reconstruction
import threading
import time
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import tkinter as tk
from tkinter.font import Font
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import can_pro_gimbal as can_pro
import ch_mavlink as mav


class GimMonitor:
    def __init__(self):
        self.mavlink = mav.Mavlink()
        self.controller_mav = mav.Controller()
        self.gimbal_mav = mav.Gimbal()
        self.can = can_pro.CAN4Gimbal(3, 0, 0)
        self.gui = GimGui(self.mavlink, self.controller_mav, self.gimbal_mav, self.can)

        self.threads = []
        t1 = threading.Thread(target=self.mav_update)
        t2 = threading.Thread(target=self.gui.lab_change)
        t3 = threading.Thread(target=self.gui.param_change)
        t4 = threading.Thread(target=self.can_update)
        self.threads.append(t1)
        self.threads.append(t2)
        self.threads.append(t3)
        self.threads.append(t4)

    # STEP 3
    def mav_update(self):
        while True:
            self.mavlink.serial2mavlink()
            ready = self.mavlink.decode_msg(self.controller_mav)
            if ready:
                self.gimbal_mav.update_attitude(self.mavlink)
                self.gui.coor_canvas.roll_data = np.append(np.delete(self.gui.coor_canvas.roll_data, 0),
                                                           self.gimbal_mav.attitude_euler[0])
                self.gui.coor_canvas.pitch_data = np.append(np.delete(self.gui.coor_canvas.pitch_data, 0),
                                                            self.gimbal_mav.attitude_euler[1])
                self.gui.coor_canvas.yaw_data = np.append(np.delete(self.gui.coor_canvas.yaw_data, 0),
                                                          self.gimbal_mav.attitude_euler[2])
                #time.sleep(0.001)


    def can_update(self):
        while True:
            self.can.high_level_read()
            self.gui.coor_canvas.roll_ctrl = np.append(np.delete(self.gui.coor_canvas.roll_ctrl, 0),
                                                       self.can.roll_ctrl)
            self.gui.coor_canvas.pitch_ctrl = np.append(np.delete(self.gui.coor_canvas.pitch_ctrl, 0),
                                                       self.can.pitch_ctrl)
            self.gui.coor_canvas.yaw_ctrl = np.append(np.delete(self.gui.coor_canvas.yaw_ctrl, 0),
                                                       self.can.yaw_ctrl)
            #time.sleep(0.00001)

    def data_init(self):
        for i in range(1000):
            self.mavlink.serial2mavlink()
            self.mavlink.decode_msg(self.controller_mav)
            self.gui.set_tk_attitude_var()
            self.gui.set_tk_control_param_var()

    def run(self):
        self.data_init()
        self.can.can_open()
        for t in self.threads:
            t.setDaemon(True)
            t.start()
        anim = animation.FuncAnimation(self.gui.coor_canvas.fig, self.gui.coor_canvas.animate,
                                       init_func=self.gui.coor_canvas.plot_init,
                                       interval=5, blit=True)
        self.gui.root.mainloop()


class CoordinateCanvas:
    def __init__(self):
        # Time range of the sensor
        self.time_len = 300
        self.time_wight = 0.8
        self.time_head = -self.time_len * self.time_wight
        self.time_tail = self.time_len * (1 - self.time_wight)
        self.time_range = np.arange(self.time_head, self.time_tail, 1)
        self.index_time_now = int(self.time_len*self.time_wight)
        self.index_time_head = 0
        self.index_time_tail = self.time_len - 1

        self.fig = plt.Figure(figsize=(5, 7), dpi=100)
        self.fig.patch.set_facecolor('dimgray')

        self.ax_bg = 'lightgray'
        self.angle_lim = 100
        self.angle_speed_lim = 10
        self.attitude_window = self.fig.add_subplot(2, 1, 1,
                                                    xlim=(self.time_head, self.time_tail),
                                                    ylim=(-self.angle_lim, self.angle_lim),
                                                    axisbg=self.ax_bg, title="Attitude")

        self.control_window = self.fig.add_subplot(2, 1, 2,
                                                   xlim=(0, 6000),
                                                   ylim=(-2000, 2000),
                                                   axisbg=self.ax_bg, title="Control")

        self.griding()

        # Data to plot
        self.roll_data = np.zeros(self.time_len)
        self.pitch_data = np.zeros(self.time_len)
        self.yaw_data = np.zeros(self.time_len)

        self.roll_ctrl = np.zeros(6000)
        self.pitch_ctrl = np.zeros(6000)
        self.yaw_ctrl = np.zeros(6000)

        # Line style
        # Set color
        # Line width
        self.line_width = 1.5
        self.line_alpha = 0.8
        self.line_roll, = self.attitude_window.plot(self.time_range, self.roll_data, label="Roll", color="red",
                                                    linewidth=self.line_width, alpha=self.line_alpha)
        self.line_pitch, = self.attitude_window.plot(self.time_range, self.pitch_data, label="Pitch", color="green",
                                                     linewidth=self.line_width, alpha=self.line_alpha)
        self.line_yaw, = self.attitude_window.plot(self.time_range, self.yaw_data, label="Yaw", color="dodgerblue",
                                                   linewidth=self.line_width, alpha=self.line_alpha)


        self.line_r_ctrl, = self.control_window.plot(range(6000), self.roll_ctrl, label="Roll", color="red",
                                                    linewidth=self.line_width, alpha=self.line_alpha)
        self.line_p_ctrl, = self.control_window.plot(range(6000), self.pitch_ctrl, label="Pitch", color="green",
                                                     linewidth=self.line_width, alpha=self.line_alpha)
        self.line_y_ctrl, = self.control_window.plot(range(6000), self.yaw_ctrl, label="Yaw",color="dodgerblue",
                                                   linewidth=self.line_width, alpha=self.line_alpha)


    def griding(self):
        # Set grid
        self.attitude_window.grid()
        gridlines = self.attitude_window.get_xgridlines() + self.attitude_window.get_ygridlines()
        for line in gridlines:
            line.set_linestyle('-')
            line.set_alpha(0.5)

        self.control_window.grid()
        gridlines = self.control_window.get_xgridlines() + self.control_window.get_ygridlines()
        for line in gridlines:
            line.set_linestyle('-')
            line.set_alpha(0.5)

    def fill_color(self):
        self.fill_roll_line = self.attitude_window.fill_between(self.time_range, 0,
                                                                self.roll_data, facecolor='red', alpha=0.4)

    def annotate(self):
        # Set annotate
        px_text = self.time_len + 1
        self.ann_roll = self.attitude_window.annotate('%.2f' % (self.roll_data[self.index_time_tail]),
                                                      xy=(self.time_len, self.roll_data[self.index_time_tail]),
                                                      xytext=(px_text, self.roll_data[self.index_time_tail]), color='red')
        self.ann_pitch = self.attitude_window.annotate('%.2f' % (self.pitch_data[self.index_time_tail]),
                                                       xy=(self.time_len, self.pitch_data[self.index_time_tail]),
                                                       xytext=(px_text, self.pitch_data[self.index_time_tail]), color='red')
        self.ann_yaw = self.attitude_window.annotate('%.2f' % (self.yaw_data[self.index_time_tail]),
                                                     xy=(self.time_len, self.yaw_data[self.index_time_tail]),
                                                     xytext=(px_text, self.yaw_data[self.index_time_tail]), color='red')

    def plot_init(self):
        self.line_roll.set_data(self.time_range, self.roll_data)
        self.line_pitch.set_data(self.time_range, self.pitch_data)
        self.line_yaw.set_data(self.time_range, self.yaw_data)
        return self.line_roll, self.line_pitch, self.line_yaw

    def animate(self,i):
        self.line_roll.set_ydata(self.roll_data)
        self.line_pitch.set_ydata(self.pitch_data)
        self.line_yaw.set_ydata(self.yaw_data)

        self.line_r_ctrl.set_ydata(self.roll_ctrl)
        self.line_p_ctrl.set_ydata(self.pitch_ctrl)
        self.line_y_ctrl.set_ydata(self.yaw_ctrl)

        return self.line_roll, self.line_pitch, self.line_yaw, self.line_r_ctrl, self.line_p_ctrl, self.line_y_ctrl


class GimGui:
    def __init__(self, mavling, controller_mav, gimbal_mav, can):
        if isinstance(mavling, mav.Mavlink):
            self.mavlink = mavling
        if isinstance(controller_mav, mav.Controller):
            self.controller_mav = controller_mav
        if isinstance(gimbal_mav, mav.Gimbal):
            self.gimbal_mav = gimbal_mav
        if isinstance(can, can_pro.CAN4Gimbal):
            self.can = can
        rootbg = 'gray21'
        self.serial_sleep_time = 0.01
        self.root = tk.Tk()
        self.root.config(bg=rootbg)
        self.root.geometry("1100x700")
        self.root.resizable(0, 0)

        self.tk_var_roll = tk.StringVar()
        self.tk_var_pitch = tk.StringVar()
        self.tk_var_yaw = tk.StringVar()

        self.tk_var_roll_rate_p = tk.StringVar()
        self.tk_var_roll_rate_i = tk.StringVar()
        self.tk_var_roll_rate_d = tk.StringVar()
        self.tk_var_roll_rate_i_max = tk.StringVar()
        self.tk_var_roll_ft_hz = tk.StringVar()
        self.tk_var_roll_stab_p = tk.StringVar()
        self.tk_var_roll_stab_d = tk.StringVar()

        self.tk_var_pitch_rate_p = tk.StringVar()
        self.tk_var_pitch_rate_i = tk.StringVar()
        self.tk_var_pitch_rate_d = tk.StringVar()
        self.tk_var_pitch_rate_i_max = tk.StringVar()
        self.tk_var_pitch_ft_hz = tk.StringVar()
        self.tk_var_pitch_stab_p = tk.StringVar()
        self.tk_var_pitch_stab_d = tk.StringVar()

        self.tk_var_yaw_rate_p = tk.StringVar()
        self.tk_var_yaw_rate_i = tk.StringVar()
        self.tk_var_yaw_rate_d = tk.StringVar()
        self.tk_var_yaw_rate_i_max = tk.StringVar()
        self.tk_var_yaw_ft_hz = tk.StringVar()
        self.tk_var_yaw_stab_p = tk.StringVar()
        self.tk_var_yaw_stab_d = tk.StringVar()

        self.lab_width = 10
        textfg = 'gray80'
        textbg = rootbg
        tfg_b = 'gray65'
        small_font = Font(family='Times New Roman', size=12)
        big_font = Font(family='Times New Roman', size=14)
        self.lab_root = tk.Label(self.root, font=('Times New Roman',14), fg=tfg_b, bg=textbg, text="GIMBAL MNG").grid(row=0, column=0, columnspan=19)
        self.lab_roll_text = tk.Label(self.root, font=small_font, fg='red', bg=textbg, text="Roll deg: ").grid(row=2, column=12, sticky='W')
        self.lab_pitch_text = tk.Label(self.root, font=small_font, fg='green', bg=textbg, text="Pitch deg: ").grid(row=3, column=12, sticky='W')
        self.lab_yaw_text = tk.Label(self.root, font=small_font, fg='dodgerblue', bg=textbg, text="Yaw deg: ").grid(row=4, column=12, sticky='W')
        self.lab_null = tk.Label(self.root, bg=textbg, text=' ').grid(row=1, column=11, rowspan=40)
        self.lab_attitude_text = tk.Label(self.root, font=small_font, fg=textfg, bg=textbg, text="Attitude").grid(row=1, column=12, columnspan=8)
        self.lab_ctrl_param_text = tk.Label(self.root, font=small_font, fg=textfg, bg=textbg, text="Control Parameter").grid(row=6, column=12, columnspan=8)

        self.entry_width = 8
        self.param_lab_width = 6

        self.lab_roll_param_text = tk.Label(self.root, fg=textfg, bg=textbg, text="Roll Param: ").grid(row=7, column=12, columnspan=8, sticky='W')
        self.lab_roll_rate_p = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate P: ").grid(row=8, column=12, sticky='W')
        self.ent_roll_rate_p = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_roll_rate_p,
                                   width=self.entry_width)  # grid(row=8, column=13, sticky='W')
        self.lab_roll_rate_i = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate I: ").grid(row=8, column=14, sticky='W')
        self.ent_roll_rate_i = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_roll_rate_i,
                                   width=self.entry_width)  # .grid(row=8, column=15, sticky='W')
        self.lab_roll_rate_d = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate D: ").grid(row=8, column=16, sticky='W')
        self.ent_roll_rate_d = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_roll_rate_d,
                                   width=self.entry_width)  # .grid(row=8, column=17, sticky='W')
        self.lab_roll_rate_i_max = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate Max I: ").grid(row=8, column=18, sticky='W')
        self.ent_roll_rate_i_max = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_roll_rate_i_max,
                                       width=self.entry_width)  # .grid(row=8, column=19, sticky='W')
        self.lab_roll_stab_p = tk.Label(self.root, fg=textfg, bg=textbg, text="Stabilize P: ").grid(row=9, column=12, sticky='W')
        self.ent_roll_stab_p = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_roll_stab_p,
                                   width=self.entry_width)  # .grid(row=9, column=13, sticky='W')
        self.lab_roll_stab_d = tk.Label(self.root, fg=textfg, bg=textbg, text="Stabilize D: ").grid(row=9, column=14, sticky='W')
        self.ent_roll_stab_d = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_roll_stab_d,
                                   width=self.entry_width)  # .grid(row=9, column=15, sticky='W')
        self.lab_roll_filter_hz = tk.Label(self.root, fg=textfg, bg=textbg, text="Filter Hz: ").grid(row=9, column=16, sticky='W')
        self.ent_roll_filter_hz = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_roll_ft_hz,
                                      width=self.entry_width)  # .grid(row=9, column=17, sticky='W')

        self.lab_pitch_param_text = tk.Label(self.root, fg=textfg, bg=textbg, text="Pitch Param: ").grid(row=10, column=12, columnspan=8, sticky='W')
        self.lab_pitch_rate_p = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate P: ").grid(row=11, column=12, sticky='W')
        self.ent_pitch_rate_p = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_pitch_rate_p,
                                    width=self.entry_width)  # .grid(row=11, column=13, sticky='W')
        self.lab_pitch_rate_i = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate I: ").grid(row=11, column=14, sticky='W')
        self.ent_pitch_rate_i = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_pitch_rate_i,
                                    width=self.entry_width)  # .grid(row=11, column=15, sticky='W')
        self.lab_pitch_rate_d = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate D: ").grid(row=11, column=16, sticky='W')
        self.ent_pitch_rate_d = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_pitch_rate_d,
                                    width=self.entry_width)  # .grid(row=11, column=17, sticky='W')
        self.lab_pitch_rate_i_max = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate Max I: ").grid(row=11, column=18, sticky='W')
        self.ent_pitch_rate_i_max = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_pitch_rate_i_max,
                                        width=self.entry_width)  # .grid(row=11, column=19, sticky='W')
        self.lab_pitch_stab_p = tk.Label(self.root, fg=textfg, bg=textbg, text="Stabilize P: ").grid(row=12, column=12, sticky='W')
        self.ent_pitch_stab_p = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_pitch_stab_p,
                                    width=self.entry_width)  # .grid(row=12, column=13, sticky='W')
        self.lab_pitch_stab_d = tk.Label(self.root, fg=textfg, bg=textbg, text="Stabilize D: ").grid(row=12, column=14, sticky='W')
        self.ent_pitch_stab_d = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_pitch_stab_d,
                                    width=self.entry_width)  # .grid(row=12, column=15, sticky='W')
        self.lab_pitch_filter_hz = tk.Label(self.root, fg=textfg, bg=textbg, text="Filter Hz: ").grid(row=12, column=16, sticky='W')
        self.ent_pitch_filter_hz = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_pitch_ft_hz,
                                       width=self.entry_width)  # .grid(row=12, column=17, sticky='W')

        self.lab_yaw_param_text = tk.Label(self.root, fg=textfg, bg=textbg, text="Yaw Param: ").grid(row=13, column=12, columnspan=8, sticky='W')
        self.lab_yaw_rate_p = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate P: ").grid(row=14, column=12, sticky='W')
        self.ent_yaw_rate_p = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_yaw_rate_p,
                                  width=self.entry_width)  # .grid(row=14, column=13, sticky='W')
        self.lab_yaw_rate_i = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate I: ").grid(row=14, column=14, sticky='W')
        self.ent_yaw_rate_i = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_yaw_rate_i,
                                  width=self.entry_width)  # .grid(row=14, column=15, sticky='W')
        self.lab_yaw_rate_d = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate D: ").grid(row=14, column=16, sticky='W')
        self.ent_yaw_rate_d = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_yaw_rate_d,
                                  width=self.entry_width)  # .grid(row=14, column=17, sticky='W')
        self.lab_yaw_rate_i_max = tk.Label(self.root, fg=textfg, bg=textbg, text="Rate Max I: ").grid(row=14, column=18, sticky='W')
        self.ent_yaw_rate_i_max = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_yaw_rate_i_max,
                                      width=self.entry_width)  # .grid(row=14, column=19, sticky='W')
        self.lab_yaw_stab_p = tk.Label(self.root, fg=textfg, bg=textbg, text="Stabilize P: ").grid(row=15, column=12, sticky='W')
        self.ent_yaw_stab_p = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_yaw_stab_p,
                                  width=self.entry_width)  # .grid(row=15, column=13, sticky='W')
        self.lab_yaw_stab_d = tk.Label(self.root, fg=textfg, bg=textbg, text="Stabilize D: ").grid(row=15, column=14, sticky='W')
        self.ent_yaw_stab_d = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_yaw_stab_d,
                                  width=self.entry_width)  # .grid(row=15, column=15, sticky='W')
        self.lab_yaw_filter_hz = tk.Label(self.root, fg=textfg, bg=textbg, text="Filter Hz: ").grid(row=15, column=16, sticky='W')
        self.ent_yaw_filter_hz = tk.Entry(self.root, fg=textfg, bg=textbg, textvariable=self.tk_var_yaw_ft_hz,
                                     width=self.entry_width)  # .grid(row=15, column=17, sticky='W')

        tk.Label(self.root, bg=textbg, text='  ').grid(row=2, column=20)

        self.lab_roll_var = tk.Label(self.root, fg='red', bg=textbg, textvariable=self.tk_var_roll).grid(row=2, column=13, sticky='W')
        self.lab_pitch_var = tk.Label(self.root, fg='green', bg=textbg, textvariable=self.tk_var_pitch).grid(row=3, column=13, sticky='W')
        self.lab_yaw_var = tk.Label(self.root, fg='dodgerblue', bg=textbg, textvariable=self.tk_var_yaw).grid(row=4, column=13, sticky='W')

        self.coor_canvas = CoordinateCanvas()
        self.canvas = FigureCanvasTkAgg(self.coor_canvas.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky='E', rowspan=30, columnspan=10)

        self.binding()
        self.griding()

        self.root.protocol('WM_DELETE_WINDOW', self.closing)

    def set_tk_attitude_var(self):
        self.tk_var_roll.set(format(self.gimbal_mav.attitude_euler[0], '.1f'))
        self.tk_var_pitch.set(format(self.gimbal_mav.attitude_euler[1], '.1f'))
        self.tk_var_yaw.set(format(self.gimbal_mav.attitude_euler[2], '.1f'))
#         rate_p    rate_i    rate_d   rate_i_max  rate_f_hz  stab_p    stab_d
#            0        1         2          3          4         5          6
    def set_tk_control_param_var(self):
        self.tk_var_roll_rate_p.set(self.controller_mav.get_param_str(0, 0))
        self.tk_var_roll_rate_i.set(self.controller_mav.get_param_str(0, 1))
        self.tk_var_roll_rate_d.set(self.controller_mav.get_param_str(0, 2))
        self.tk_var_roll_rate_i_max.set(self.controller_mav.get_param_str(0, 3))
        self.tk_var_roll_ft_hz.set(self.controller_mav.get_param_str(0, 4))
        self.tk_var_roll_stab_p.set(self.controller_mav.get_param_str(0, 5))
        self.tk_var_roll_stab_d.set(self.controller_mav.get_param_str(0, 6))

        self.tk_var_pitch_rate_p.set(self.controller_mav.get_param_str(1, 0))
        self.tk_var_pitch_rate_i.set(self.controller_mav.get_param_str(1, 1))
        self.tk_var_pitch_rate_d.set(self.controller_mav.get_param_str(1, 2))
        self.tk_var_pitch_rate_i_max.set(self.controller_mav.get_param_str(1, 3))
        self.tk_var_pitch_ft_hz.set(self.controller_mav.get_param_str(1, 4))
        self.tk_var_pitch_stab_p.set(self.controller_mav.get_param_str(1, 5))
        self.tk_var_pitch_stab_d.set(self.controller_mav.get_param_str(1, 6))

        self.tk_var_yaw_rate_p.set(self.controller_mav.get_param_str(2, 0))
        self.tk_var_yaw_rate_i.set(self.controller_mav.get_param_str(2, 1))
        self.tk_var_yaw_rate_d.set(self.controller_mav.get_param_str(2, 2))
        self.tk_var_yaw_rate_i_max.set(self.controller_mav.get_param_str(2, 3))
        self.tk_var_yaw_ft_hz.set(self.controller_mav.get_param_str(2, 4))
        self.tk_var_yaw_stab_p.set(self.controller_mav.get_param_str(2, 5))
        self.tk_var_yaw_stab_d.set(self.controller_mav.get_param_str(2, 6))

    def send_cmd(self, ax_id=0, param_id=0, value=0 ):
        self.controller_mav.set_param(ax_id, param_id, value)
        self.mavlink.serial_send_buf(self.controller_mav.pack_param(ax_id, param_id))
        time.sleep(self.serial_sleep_time)
        mavlink.serial_send_buf(self.controller_mav.pack_param(ax_id, param_id))
        time.sleep(self.serial_sleep_time)
        mavlink.serial_send_buf(self.controller_mav.pack_param(ax_id, param_id))

    # Bind entries to callbacks
    def ent_roll_rate_p_callback(self, var):
        value = float(var.get())
        #print(value)
        self.send_cmd(0, 0, value)
    def ent_roll_rate_i_callback(self, var):
        value = float(var.get())
        #print(value)
        self.send_cmd(0, 1, value)
    def ent_roll_rate_d_callback(self, var):
        value = float(var.get())
        #print(value)
        self.send_cmd(0, 2, value)
    def ent_roll_rate_i_max_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(0, 3, value)
    def ent_roll_ft_hz_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(0, 4, value)
    def ent_roll_stab_p_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(0, 5, value)
    def ent_roll_stab_d_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(0, 6, value)
    def ent_pitch_rate_p_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(1, 0, value)
    def ent_pitch_rate_i_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(1, 1, value)
    def ent_pitch_rate_d_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(1, 2, value)
    def ent_pitch_rate_i_max_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(1, 3, value)
    def ent_pitch_ft_hz_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(1, 4, value)
    def ent_pitch_stab_p_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(1, 5, value)
    def ent_pitch_stab_d_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(1, 6, value)
    def ent_yaw_rate_p_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(2, 0, value)
    def ent_yaw_rate_i_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(2, 1, value)
    def ent_yaw_rate_d_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(2, 2, value)
    def ent_yaw_rate_i_max_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(2, 3, value)
    def ent_yaw_ft_hz_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(2, 4, value)
    def ent_yaw_stab_p_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(2, 5, value)
    def ent_yaw_stab_d_callback(self,var):
        value = float(var.get())
        #print(value)
        self.send_cmd(2, 6, value)

    # Binding Callback function
    def binding(self):
        self.ent_roll_rate_p.bind('<Return>', (lambda _: self.ent_roll_rate_p_callback(self.ent_roll_rate_p)))
        self.ent_roll_rate_i.bind('<Return>', (lambda _: self.ent_roll_rate_i_callback(self.ent_roll_rate_i)))
        self.ent_roll_rate_d.bind('<Return>', (lambda _: self.ent_roll_rate_d_callback(self.ent_roll_rate_d)))
        self.ent_roll_rate_i_max.bind('<Return>', (lambda _: self.ent_roll_rate_i_max_callback(self.ent_roll_rate_i_max)))
        self.ent_roll_filter_hz.bind('<Return>', (lambda _: self.ent_roll_ft_hz_callback(self.ent_roll_filter_hz)))
        self.ent_roll_stab_p.bind('<Return>', (lambda _: self.ent_roll_stab_p_callback(self.ent_roll_stab_p)))
        self.ent_roll_stab_d.bind('<Return>', (lambda _: self.ent_roll_stab_d_callback(self.ent_roll_stab_d)))

        self.ent_pitch_rate_p.bind('<Return>', (lambda _: self.ent_pitch_rate_p_callback(self.ent_pitch_rate_p)))
        self.ent_pitch_rate_i.bind('<Return>', (lambda _: self.ent_pitch_rate_i_callback(self.ent_pitch_rate_i)))
        self.ent_pitch_rate_d.bind('<Return>', (lambda _: self.ent_pitch_rate_d_callback(self.ent_pitch_rate_d)))
        self.ent_pitch_rate_i_max.bind('<Return>', (lambda _: self.ent_pitch_rate_i_max_callback(self.ent_pitch_rate_i_max)))
        self.ent_pitch_filter_hz.bind('<Return>', (lambda _: self.ent_pitch_ft_hz_callback(self.ent_pitch_filter_hz)))
        self.ent_pitch_stab_p.bind('<Return>', (lambda _: self.ent_pitch_stab_p_callback(self.ent_pitch_stab_p)))
        self.ent_pitch_stab_d.bind('<Return>', (lambda _: self.ent_pitch_stab_d_callback(self.ent_pitch_stab_d)))

        self.ent_yaw_rate_p.bind('<Return>', (lambda _: self.ent_yaw_rate_p_callback(self.ent_yaw_rate_p)))
        self.ent_yaw_rate_i.bind('<Return>', (lambda _: self.ent_yaw_rate_i_callback(self.ent_yaw_rate_i)))
        self.ent_yaw_rate_d.bind('<Return>', (lambda _: self.ent_yaw_rate_d_callback(self.ent_yaw_rate_d)))
        self.ent_yaw_rate_i_max.bind('<Return>', (lambda _: self.ent_yaw_rate_i_max_callback(self.ent_yaw_rate_i_max)))
        self.ent_yaw_filter_hz.bind('<Return>', (lambda _: self.ent_yaw_ft_hz_callback(self.ent_yaw_filter_hz)))
        self.ent_yaw_stab_p.bind('<Return>', (lambda _: self.ent_yaw_stab_p_callback(self.ent_yaw_stab_p)))
        self.ent_yaw_stab_d.bind('<Return>', (lambda _: self.ent_yaw_stab_d_callback(self.ent_yaw_stab_d)))
    # Grid
    def griding(self):
        self.ent_roll_rate_p.grid(row=8, column=13, sticky='W')
        self.ent_roll_rate_i.grid(row=8, column=15, sticky='W')
        self.ent_roll_rate_d.grid(row=8, column=17, sticky='W')
        self.ent_roll_rate_i_max.grid(row=8, column=19, sticky='W')
        self.ent_roll_stab_p.grid(row=9, column=13, sticky='W')
        self.ent_roll_stab_d.grid(row=9, column=15, sticky='W')
        self.ent_roll_filter_hz.grid(row=9, column=17, sticky='W')

        self.ent_pitch_rate_p.grid(row=11, column=13, sticky='W')
        self.ent_pitch_rate_i.grid(row=11, column=15, sticky='W')
        self.ent_pitch_rate_d.grid(row=11, column=17, sticky='W')
        self.ent_pitch_rate_i_max.grid(row=11, column=19, sticky='W')
        self.ent_pitch_stab_p.grid(row=12, column=13, sticky='W')
        self.ent_pitch_stab_d.grid(row=12, column=15, sticky='W')
        self.ent_pitch_filter_hz.grid(row=12, column=17, sticky='W')

        self.ent_yaw_rate_p.grid(row=14, column=13, sticky='W')
        self.ent_yaw_rate_i.grid(row=14, column=15, sticky='W')
        self.ent_yaw_rate_d.grid(row=14, column=17, sticky='W')
        self.ent_yaw_rate_i_max.grid(row=14, column=19, sticky='W')
        self.ent_yaw_stab_p.grid(row=15, column=13, sticky='W')
        self.ent_yaw_stab_d.grid(row=15, column=15, sticky='W')
        self.ent_yaw_filter_hz.grid(row=15, column=17, sticky='W')

    def closing(self):
        self.can.close()
        self.root.destroy()

    def lab_change(self):
        while True:
            self.set_tk_attitude_var()
            #set_tk_control_param_var()
            time.sleep(0.01)
            #root.update()
    def param_change(self):
        while True:
            self.set_tk_control_param_var()
            time.sleep(1)


monitor = GimMonitor()
monitor.run()






















