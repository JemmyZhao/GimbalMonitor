

#---------Imports
from numpy import arange, sin, pi
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as Tk
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import threading
#---------End of imports

fig = plt.Figure()

x = np.arange(0, 2*np.pi, 0.01)        # x-array

def animate(i):
    line.set_ydata(np.sin(x+i/10.0))  # update the data
    return line,

root = Tk.Tk()
var = Tk.StringVar()
var.set('x')

label = Tk.Label(root, text="SHM Simulation").grid(column=0, row=0)
lab1 = Tk.Label(root, textvariable=var).grid(column=0, row=2)
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().grid(column=0, row=1)

def change_x():
    i = 0
    while True:
        i = i+1
        var.set(format(i, '.1f'))
        root.update()
        time.sleep(0.01)

ax = fig.add_subplot(111)
line, = ax.plot(x, np.sin(x))

threads = []
t1 = threading.Thread(target=change_x)
threads.append(t1)



if __name__ == '__main__':
    for t in threads:
        t.start()

    ani = animation.FuncAnimation(fig, animate, np.arange(1, 200), interval=25, blit=False)
    Tk.mainloop()

