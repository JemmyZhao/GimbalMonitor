from tkinter import *

def callback(sv):
    print(sv.get())

root = Tk()

sv = StringVar()
e = Entry(root, textvariable=sv)
e.bind('<Return>', (lambda _: callback(e)))

e.grid(row=0, column=0)
root.mainloop()