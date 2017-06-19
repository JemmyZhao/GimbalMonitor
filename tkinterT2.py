import tkinter as tk
import time
import threading

root = tk.Tk()

var = tk.StringVar()
var1 = tk.StringVar()
var.set('0')
var1.set('1')
print(var)
lab = tk.Label(root, textvariable=var).grid(row=0, column=0)
lab1 = tk.Label(root, textvariable=var1).grid(row=1, column=0)
print(lab)
def change():
    for i in range(100):
        var.set(format(i, '.2f'))
        var1.set(format(-i, '.2f'))
        root.update()
        time.sleep(0.1)

threads = []
t1 = threading.Thread(target=change)

threads.append(t1)

if __name__ == '__main__':
    for t in threads:
        t.setDaemon(True)
        t1.start()

    root.mainloop()