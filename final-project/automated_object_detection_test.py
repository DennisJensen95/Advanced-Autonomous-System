from subprocess import Popen, PIPE
import time

def open_ulmsserver():
    ulmsserver = Popen(['ulmsserver'], stdout=PIPE, stderr=PIPE, universal_newlines=True)
    return ulmsserver


ulmsserver = open_ulmsserver()
start = time.time()
timeout = 15
while True:
    print(ulmsserver.stdout.readline())

    if time.time() - start > timeout:
        ulmsserver.kill()
        break


print("Done")