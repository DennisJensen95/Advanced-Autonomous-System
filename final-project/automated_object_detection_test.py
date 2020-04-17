from subprocess import Popen, PIPE
import select
import time


def open_ulmsserver():
    """
    Open ulmserver
    :return:
    """
    ulmsserver = Popen(['ulmsserver'], stdout=PIPE, stdin=PIPE, universal_newlines=True)
    out = select.poll()
    out.register(ulmsserver.stdout, select.POLLIN)
    return ulmsserver, out

ulmsserver, out = open_ulmsserver()
start = time.time()
timeout = 15
while True:
    if out.poll(1):
        print(ulmsserver.stdout.readline())
    else:
        print("")
        time.sleep(0.5)

    if time.time() - start > timeout:
        ulmsserver.stdin.write('q')
        time.sleep(1)
        ulmsserver.terminate()
        break

print("Done")