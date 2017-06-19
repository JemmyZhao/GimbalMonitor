import pyqtgraph as pg
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore



app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="My text")
win.resize(1000, 500)
win.setWindowTitle('Win Test')

pg.setConfigOptions(antialias=True)
pg.setConfigOption('background', [255, 255, 255])
pg.setConfigOption('foreground', [255, 255, 0, 200])
x1 = np.linspace(0, 5, 100)
y1 = np.sin(x1)

pen1 = pg.mkPen('0F0', width=3)
p1 = win.addPlot(title='Sin1')
curve1 = p1.plot(pen='g', fillLevel=0, brush=(0, 255, 0, 100))
p1.showGrid(x=True, y=True,)
def update1():
    global curve1, x1, y1, p1
    curve1.setData(y1)
    x1 = np.delete(x1, 0)
    x1 = np.append(x1, x1[-1]+0.05)
    y1 = np.sin(x1)
timer1 = QtCore.QTimer()
timer1.timeout.connect(update1)
timer1.start(5)

pen1 = pg.mkPen('F00', width=3)
x2 = np.linspace(0, 5, 100)
y2 = np.sin(x2)
p2 = win.addPlot(title='Sin2')
curve2 = p2.plot(pen=pen1, fillLevel=0, brush=[255, 0, 0, 100])
p2.showGrid(x=True, y=True)
def update2():
    global curve2, x2, y2, p2
    curve2.setData(y2)
    x2 = np.delete(x2, 0)
    x2 = np.append(x2, x2[-1]+0.05)
    y2 = np.sin(x2)
timer2 = QtCore.QTimer()
timer2.timeout.connect(update2)
timer2.start(7)

x3 = np.linspace(0, 5, 100)
y3 = np.sin(x3)
p3 = win.addPlot(title='Sin3')
curve3 = p3.plot(pen='b')
p3.showGrid(x=True, y=True)
def update3():
    global curve3, x3, y3, p3
    curve3.setData(y3)
    x3 = np.delete(x3, 0)
    x3 = np.append(x3, x3[-1]+0.05)
    y3 = np.sin(x3)
timer3 = QtCore.QTimer()
timer3.timeout.connect(update3)
timer3.start(9)

win.nextRow()

pen4 = pg.mkPen('00F', width=3)
x4 = np.linspace(0, 5, 100)
y4 = np.sin(x4)
p4 = win.addPlot(title='Sin4')
curve4 = p4.plot(pen=pen4, width=6, fillLevel=0, brush=(0, 0, 255, 100))
p4.showGrid(x=True, y=True)
def update4():
    global curve4, x4, y4, p4
    curve4.setData(y4)
    x4 = np.delete(x4, 0)
    x4 = np.append(x4, x4[-1]+0.05)
    y4 = np.sin(x4)
timer4 = QtCore.QTimer()
timer4.timeout.connect(update4)
timer4.start(11)

if __name__ == '__main__':
    import sys
    if(sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()