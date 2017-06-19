import threading
import can_pro_gimbal
import tkinter as tk

class CanGui:
    def __init__(self):
        self.gim = can_pro_gimbal.CAN4Gimbal()

        self.gim.can_open()

        self.root = tk.Tk()

        self.b_p_enc = tk.Button(self.root, text='P ENC', command=self.send_p_enc).pack()
        self.b_r_enc = tk.Button(self.root, text='R ENC', command=self.send_r_enc).pack()
        self.b_y_enc = tk.Button(self.root, text='Y ENC', command=self.send_y_enc).pack()
        self.b_r_rps = tk.Button(self.root, text='R RPS', command=self.send_r_rps_mcu).pack()
        self.b_p_rps = tk.Button(self.root, text='P RPS', command=self.send_p_rps_mcu).pack()
        self.b_y_rps = tk.Button(self.root, text='Y RPS', command=self.send_y_rps_mcu).pack()
        self.b_r_rpl_n = tk.Button(self.root, text='R RPL N', command=self.send_r_rpl_mcu_n).pack()
        self.b_r_rpl_h = tk.Button(self.root, text='R RPL H', command=self.send_r_rpl_mcu_h).pack()
        self.b_p_rpl_n = tk.Button(self.root, text='P RPL N', command=self.send_p_rpl_mcu_n).pack()
        self.b_p_rpl_h = tk.Button(self.root, text='P RPL H', command=self.send_p_rpl_mcu_h).pack()
        self.b_y_rpl_n = tk.Button(self.root, text='Y RPL N', command=self.send_y_rpl_mcu_n).pack()
        self.b_y_rpl_h = tk.Button(self.root, text='Y RPL H', command=self.send_y_rpl_mcu_h).pack()
        self.b_r_rs = tk.Button(self.root, text='R RS', command=self.send_r_rs_mcu).pack()
        self.b_p_rs = tk.Button(self.root, text='P RS', command=self.send_p_rs_mcu).pack()
        self.b_y_rs = tk.Button(self.root, text='Y RS', command=self.send_y_rs_mcu).pack()
        self.b_r_rl = tk.Button(self.root, text='R RL', command=self.send_r_rl_mcu).pack()
        self.b_p_rl = tk.Button(self.root, text='P RL', command=self.send_p_rl_mcu).pack()
        self.b_y_rl = tk.Button(self.root, text='Y RL', command=self.send_y_rl_mcu).pack()

        self.root.protocol('WM_DELETE_WINDOW', self.closing)

        self.ready2decode = 0

        self.rx_mail = []

        self.threads = []
        t1 = threading.Thread(target=self.recv)
        t2 = threading.Thread(target=self.decode)
        self.threads.append(t1)
        self.threads.append(t2)

    def closing(self):
        self.gim.close()
        self.root.destroy()

    def run(self):
        for t in self.threads:
            if isinstance(t, threading.Thread):
                t.setDaemon(True)
                t.start()
        self.root.mainloop()

    def recv(self):
        while True:
            self.gim.high_level_read()
            #if self.gim.can_read() > 0:
            #    self.rx_mail.append(self.gim.rx_msg)
                #print(len(self.rx_mail))

    def decode(self):
        return
        #while True:
            #if len(self.rx_mail) > 0:
                #print('decode')
            #    self.gim.can_decode(self.rx_mail.pop(0))

    def send_test(self, msgid):
        self.gim.tx_msg.id = msgid
        self.gim.can_write()

    def send_p_enc(self):
        self.send_test(can_pro_gimbal.PITCH_ENC)

    def send_r_enc(self):
        self.send_test(can_pro_gimbal.ROLL_ENC)

    def send_y_enc(self):
        self.send_test(can_pro_gimbal.YAW_ENC)

    def send_r_rps_mcu(self):
        self.send_test(can_pro_gimbal.ROLL_RPS_MCU)

    def send_p_rps_mcu(self):
        self.send_test(can_pro_gimbal.PITCH_RPS_MCU)

    def send_y_rps_mcu(self):
        self.send_test(can_pro_gimbal.YAW_RPS_MCU)

    def send_r_rpl_mcu_n(self):
        self.send_test(can_pro_gimbal.ROLL_RPL_MCU_NORMAL)

    def send_r_rpl_mcu_h(self):
        self.send_test(can_pro_gimbal.ROLL_RPL_MCU_HEAD)

    def send_p_rpl_mcu_n(self):
        self.send_test(can_pro_gimbal.PITCH_RPL_MCU_NORMAL)

    def send_p_rpl_mcu_h(self):
        self.send_test(can_pro_gimbal.PITCH_RPL_MCU_HEAD)

    def send_y_rpl_mcu_n(self):
        self.send_test(can_pro_gimbal.YAW_RPL_MCU_NORMAL)

    def send_y_rpl_mcu_h(self):
        self.send_test(can_pro_gimbal.YAW_RPL_MCU_HEAD)

    def send_r_rs_mcu(self):
        self.send_test(can_pro_gimbal.ROLL_RS_MCU)

    def send_p_rs_mcu(self):
        self.send_test(can_pro_gimbal.PITCH_RS_MCU)

    def send_y_rs_mcu(self):
        self.send_test(can_pro_gimbal.YAW_RS_MCU)

    def send_r_rl_mcu(self):
        self.send_test(can_pro_gimbal.ROLL_RL_MCU)

    def send_p_rl_mcu(self):
        self.send_test(can_pro_gimbal.PITCH_RL_MCU)

    def send_y_rl_mcu(self):
        self.send_test(can_pro_gimbal.YAW_RL_MCU)

gui = CanGui()
gui.run()


























