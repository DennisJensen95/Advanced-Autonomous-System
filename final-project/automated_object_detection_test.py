from subprocess import Popen, PIPE
import threading
import time

def open_ulmsserver():
    """
    Open ulmserver
    :return:
    """
    ulmsserver = Popen(['ulmsserver'], stdout=PIPE, stdin=PIPE, universal_newlines=True)
    return ulmsserver

ulmsserver= open_ulmsserver()
start = time.time()
timeout = 15
while True:
    output = ulmsserver.stdout.read() # <---- hangs here
    if not output:
        print("No more data")
        break
    if output:
        print(output.strip())

    if time.time() - start > timeout:
        time.sleep(1)
        ulmsserver.terminate()
        break

print("Done")