import  time, threading
global val
val = 0


def foo():
    global val
    val += 1
    print(time.ctime(), ' ', val)
    threading.Timer(2, foo).start()

foo()

for x in range(10):
    time.sleep(5)
    val = 0
    print('for loop val ', val)
