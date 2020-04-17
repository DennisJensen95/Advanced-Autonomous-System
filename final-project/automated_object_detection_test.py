from subprocess import Popen, PIPE
import select
import time


def open_ulmsserver():
    """
    Open ulmserver
    :return:
    """
    ulmsserver = Popen(['ulmsserver'], stdout=PIPE, stderr=PIPE, universal_newlines=True)
    out = select.poll()
    out.register(ulmsserver.stdout, select.POLLIN)
    return ulmsserver, out

ulmsserver, out = open_ulmsserver()
start = time.time()
timeout = 15
while True:
    if out.poll(1):
        print(ulmsserver.stdout.readline())
        ulmsserver.stdout.flush()
    else:
        print("")
        time.sleep(0.5)

    if time.time() - start > timeout:
        time.sleep(1)
        ulmsserver.terminate()
        break

print("Done")