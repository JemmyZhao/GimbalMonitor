import threading
import time


def me():
    for i in range(100):
        print('me\n')
        time.sleep(0.001)


def you():
    for i in range(100):
        print('you\n')
        time.sleep(0.002)

tme = threading.Thread(name='me', target=me)
tyou = threading.Thread(name='you', target=you)

tme.start()
tyou.start()