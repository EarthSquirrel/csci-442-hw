import  time, threading
global val
val = 0


def foo():
    global val
    val += 1
    print(time.ctime(), ' ', val)
    # threading.Timer(2, foo).start()

foo()
test = threading.Timer(2, foo)
test.start()

for x in range(10):
    time.sleep(5)
    val = 0
    print('for loop val ', val)

    if x == 4:
        print('stoping test')
        test.cancel()
